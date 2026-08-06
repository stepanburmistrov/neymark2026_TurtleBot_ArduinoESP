#!/usr/bin/env python3
"""Автономный проезд по точкам ArUco-маркеров ID 1, 2 и 3.

Камера определяет положение робота по маркеру ID 4. При запуске миссии
координаты целевых маркеров ID 1, 2 и 3 запоминаются. После этого сами
целевые маркеры могут быть закрыты корпусом робота: движение продолжается
по сохранённым координатам.

Управление:
    Space — запомнить цели и запустить миссию / пауза / продолжение;
    R     — сбросить миссию и сохранённые точки;
    Esc   — безопасно остановить робота и завершить программу.

Перед запуском:
    1. Подключить компьютер к Wi-Fi сети ESP32.
    2. Проверить ROBOT_ADDRESS.
    3. Откалибровать пороги датчиков линии.
    4. Закрепить маркер ID 4 стороной P0–P1 вперёд по корпусу.
    5. Не перемещать камеру после сохранения целей.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import socket
import time

import cv2
import numpy as np


CAMERA_INDEX = 0
ROBOT_MARKER_ID = 4
TARGET_SEQUENCE = (1, 2, 3)
ROBOT_ADDRESS = ("192.168.4.1", 8888)

LINEAR_SPEED_MM_S = 210
ANGULAR_SPEED_MRAD_S = 3200
TARGET_TOLERANCE_PX = 30
ANGLE_TOLERANCE_RAD = math.radians(10)
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.8

# Белое поле даёт значение выше порога, чёрная граница — ниже.
# Значения обязательно проверить для каждого робота.
LEFT_LINE_THRESHOLD = 5
RIGHT_LINE_THRESHOLD = 5
OBSTACLE_STOP_CM = 14

LINE_SAFETY_ENABLED = True
OBSTACLE_SAFETY_ENABLED = True

INFO_PANEL_HEIGHT = 185
TEXT_FONT = cv2.FONT_HERSHEY_COMPLEX


@dataclass
class SafetyData:
    """Поля телеметрии, которые используются для аварийной остановки."""

    line_left: int = 1023
    line_right: int = 1023
    ir_cm: int = 60


def parse_safety(line: str) -> SafetyData | None:
    """Из полной строки TEL извлекает датчики линии и дальномер."""
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return SafetyData(
            line_left=int(parts[8]),
            line_right=int(parts[9]),
            ir_cm=int(parts[10]),
        )
    except ValueError:
        return None


def send(connection: socket.socket, command: str) -> None:
    """Отправляет одну строковую команду роботу."""
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except (BlockingIOError, OSError):
        pass


def receive_telemetry(
    connection: socket.socket,
    buffer: bytes,
    current: SafetyData,
) -> tuple[bytes, SafetyData, bool]:
    """Читает все доступные байты и сохраняет неполную строку в buffer."""
    received = False

    while True:
        try:
            packet = connection.recv(2048)
        except BlockingIOError:
            break

        if not packet:
            raise ConnectionError("TCP-соединение закрыто")
        buffer += packet

    while b"\n" in buffer:
        raw_line, buffer = buffer.split(b"\n", 1)
        parsed = parse_safety(
            raw_line.decode("ascii", errors="replace").strip()
        )
        if parsed is not None:
            current = parsed
            received = True

    return buffer, current, received


def marker_geometry(corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Возвращает центр маркера и единичный вектор его стороны P0–P1."""
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)

    front = 0.5 * (points[0] + points[1])
    heading = front - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)

    return center, heading


def signed_angle(heading: np.ndarray, vector: np.ndarray) -> float:
    """Возвращает угол от heading к vector в диапазоне [-pi, pi]."""
    # В изображении Y направлен вниз. Для привычной геометрии разворачиваем Y.
    hx, hy = float(heading[0]), -float(heading[1])
    vx, vy = float(vector[0]), -float(vector[1])

    cross = hx * vy - hy * vx
    dot = hx * vx + hy * vy
    return math.atan2(cross, dot)


def remember_target_points(
    markers: dict[int, np.ndarray],
) -> tuple[dict[int, np.ndarray], tuple[int, ...]]:
    """Запоминает центры всех целей, если они одновременно видны."""
    missing = tuple(
        marker_id
        for marker_id in TARGET_SEQUENCE
        if marker_id not in markers
    )
    if missing:
        return {}, missing

    points: dict[int, np.ndarray] = {}
    for marker_id in TARGET_SEQUENCE:
        center, _heading = marker_geometry(markers[marker_id])
        points[marker_id] = center.copy()

    return points, ()


def draw_saved_targets(
    frame: np.ndarray,
    target_points: dict[int, np.ndarray],
    current_id: int | None,
) -> None:
    """Рисует сохранённые точки и номер каждой цели."""
    for marker_id, point in target_points.items():
        pixel = tuple(np.rint(point).astype(int))
        current = marker_id == current_id
        radius = TARGET_TOLERANCE_PX if current else 18
        thickness = 3 if current else 2

        cv2.circle(frame, pixel, radius, (0, 170, 255), thickness)
        cv2.circle(frame, pixel, 4, (0, 170, 255), -1)
        cv2.putText(
            frame,
            str(marker_id),
            (pixel[0] + 8, pixel[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 170, 255),
            2,
            cv2.LINE_AA,
        )


def make_display_frame(
    frame: np.ndarray,
    state: str,
    command: str,
    target_index: int,
    target_points: dict[int, np.ndarray],
    safety: SafetyData,
) -> np.ndarray:
    """Добавляет снизу информационную панель.

    Встроенные шрифты OpenCV не поддерживают кириллицу, поэтому текст панели
    намеренно выводится на английском. README и комментарии остаются русскими.
    """
    _height, width = frame.shape[:2]
    panel = np.full(
        (INFO_PANEL_HEIGHT, width, 3),
        (24, 24, 24),
        dtype=np.uint8,
    )
    cv2.line(panel, (0, 0), (width - 1, 0), (90, 90, 90), 1)

    status_color = (80, 230, 80) if command != "STOP" else (80, 120, 255)
    current_id = (
        TARGET_SEQUENCE[target_index]
        if target_index < len(TARGET_SEQUENCE)
        else None
    )
    saved_ids = tuple(target_points.keys())

    lines = [
        (f"STATE: {state}", 32, 0.68, status_color, 2),
        (
            f"ROUTE: {TARGET_SEQUENCE}    CURRENT: {current_id}    "
            f"SAVED: {saved_ids if saved_ids else 'none'}",
            67,
            0.52,
            (240, 240, 240),
            1,
        ),
        (
            f"SENSORS: L={safety.line_left} R={safety.line_right} "
            f"IR={safety.ir_cm}cm    COMMAND: {command}",
            102,
            0.52,
            (240, 240, 240),
            1,
        ),
        ("SPACE: start/pause    R: reset    ESC: stop", 137, 0.52, (210, 210, 210), 1),
        ("Do not move the camera after saving targets", 168, 0.46, (0, 180, 255), 1),
    ]

    for text, y, scale, color, thickness in lines:
        cv2.putText(
            panel,
            text,
            (18, y),
            TEXT_FONT,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    return np.vstack((frame, panel))


def main() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        cv2.aruco.DetectorParameters(),
    )

    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    telemetry_buffer = b""
    safety = SafetyData()
    target_points: dict[int, np.ndarray] = {}
    target_index = 0
    paused = True
    state = "SHOW MARKERS 1, 2, 3 AND PRESS SPACE"

    previous_send = 0.0
    last_telemetry_time = 0.0
    trajectory: list[tuple[int, int]] = []

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Не удалось получить кадр камеры")

            telemetry_buffer, safety, telemetry_received = receive_telemetry(
                connection,
                telemetry_buffer,
                safety,
            )
            if telemetry_received:
                last_telemetry_time = time.monotonic()

            corners, ids, _rejected = detector.detectMarkers(frame)
            markers: dict[int, np.ndarray] = {}

            if ids is not None:
                markers = {
                    int(marker_id): marker_corners
                    for marker_corners, marker_id in zip(
                        corners,
                        ids.flatten(),
                    )
                }
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            command = "STOP"
            current_id = (
                TARGET_SEQUENCE[target_index]
                if target_index < len(TARGET_SEQUENCE)
                else None
            )

            telemetry_stale = (
                time.monotonic() - last_telemetry_time
                > TELEMETRY_TIMEOUT_SECONDS
            )
            black_border = LINE_SAFETY_ENABLED and (
                safety.line_left < LEFT_LINE_THRESHOLD
                or safety.line_right < RIGHT_LINE_THRESHOLD
            )
            obstacle = (
                OBSTACLE_SAFETY_ENABLED
                and 0 < safety.ir_cm <= OBSTACLE_STOP_CM
            )

            if telemetry_stale:
                state = "WAITING FOR TELEMETRY"
            elif target_index >= len(TARGET_SEQUENCE):
                state = "MISSION COMPLETE"
                paused = True
            elif black_border:
                state = "BLACK BORDER: STOP"
                paused = True
            elif obstacle:
                state = f"OBSTACLE AT {safety.ir_cm} CM"
            elif paused:
                # Текст состояния уже установлен обработчиком клавиш.
                pass
            elif ROBOT_MARKER_ID not in markers:
                state = f"WAITING FOR ROBOT MARKER {ROBOT_MARKER_ID}"
            elif current_id not in target_points:
                state = "TARGETS ARE NOT SAVED"
                paused = True
            else:
                robot, heading = marker_geometry(markers[ROBOT_MARKER_ID])
                target = target_points[current_id]
                vector = target - robot
                distance = float(np.linalg.norm(vector))
                error = signed_angle(heading, vector)

                robot_point = tuple(np.rint(robot).astype(int))
                target_point = tuple(np.rint(target).astype(int))

                if not trajectory or math.dist(robot_point, trajectory[-1]) >= 3:
                    trajectory.append(robot_point)
                    trajectory = trajectory[-600:]

                cv2.arrowedLine(
                    frame,
                    robot_point,
                    target_point,
                    (0, 170, 255),
                    3,
                )

                if distance <= TARGET_TOLERANCE_PX:
                    target_index += 1
                    state = f"TARGET {current_id} REACHED"
                elif abs(error) > ANGLE_TOLERANCE_RAD:
                    angular = (
                        ANGULAR_SPEED_MRAD_S
                        if error > 0
                        else -ANGULAR_SPEED_MRAD_S
                    )
                    command = f"VEL 0 {angular}"
                    state = (
                        f"TURN TO {current_id}: "
                        f"{math.degrees(error):.1f} DEG"
                    )
                else:
                    command = f"VEL {LINEAR_SPEED_MM_S} 0"
                    state = f"DRIVE TO {current_id}: {distance:.0f} PX"

            draw_saved_targets(frame, target_points, current_id)

            if len(trajectory) >= 2:
                cv2.polylines(
                    frame,
                    [np.array(trajectory, dtype=np.int32)],
                    False,
                    (255, 150, 0),
                    2,
                )

            display_frame = make_display_frame(
                frame=frame,
                state=state,
                command=command,
                target_index=target_index,
                target_points=target_points,
                safety=safety,
            )

            now = time.monotonic()
            if now - previous_send >= SEND_PERIOD_SECONDS:
                send(connection, command)
                previous_send = now

            cv2.imshow("ArUco route mission", display_frame)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key in (ord("r"), ord("R")):
                send(connection, "STOP")
                paused = True
                target_index = 0
                target_points.clear()
                trajectory.clear()
                state = "SHOW MARKERS 1, 2, 3 AND PRESS SPACE"

            if key == ord(" "):
                if paused:
                    if not target_points:
                        saved_points, missing = remember_target_points(markers)
                        if missing:
                            missing_text = ", ".join(map(str, missing))
                            state = f"MISSING TARGET MARKERS: {missing_text}"
                            send(connection, "STOP")
                        else:
                            target_points = saved_points
                            target_index = 0
                            trajectory.clear()
                            paused = False
                            state = "MISSION STARTED"
                    else:
                        paused = False
                        state = "MISSION CONTINUED"
                else:
                    paused = True
                    state = "PAUSED"
                    send(connection, "STOP")

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

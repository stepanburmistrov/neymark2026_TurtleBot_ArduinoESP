#!/usr/bin/env python3
"""Робот самостоятельно едет к крупнейшему объекту выбранного цвета.

Клавиши:
    1 — красная цель;
    2 — зелёная цель;
    3 — синяя цель;
    Space — пауза/продолжение;
    Esc — безопасная остановка и выход.

Положение робота определяется по ArUco ID 4. Положение цели определяется
по крупнейшему контуру цветовой маски. Это пример объединения HSV, контуров,
ArUco и управления движением в одном проекте.
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
ROBOT_ADDRESS = ("192.168.4.1", 8888)

LINEAR_SPEED_MM_S = 190
ANGULAR_SPEED_MRAD_S = 3000
TARGET_TOLERANCE_PX = 65
ANGLE_TOLERANCE_RAD = math.radians(10)
MIN_CONTOUR_AREA = 1200
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.8

LEFT_LINE_THRESHOLD = 5
RIGHT_LINE_THRESHOLD = 5
OBSTACLE_STOP_CM = 5


HSV_RANGES: dict[str, tuple[tuple[np.ndarray, np.ndarray], ...]] = {
    "RED": (
        (np.array([0, 100, 70]), np.array([10, 255, 255])),
        (np.array([170, 100, 70]), np.array([179, 255, 255])),
    ),
    "GREEN": (
        (np.array([40, 70, 55]), np.array([85, 255, 255])),
    ),
    "BLUE": (
        (np.array([95, 80, 55]), np.array([135, 255, 255])),
    ),
}

DRAW_COLORS = {
    "RED": (0, 0, 255),
    "GREEN": (0, 210, 0),
    "BLUE": (255, 80, 0),
}


@dataclass
class SafetyData:
    line_left: int = 1023
    line_right: int = 1023
    ir_cm: int = 60


def parse_safety(line: str) -> SafetyData | None:
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
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except (BlockingIOError, OSError):
        pass


def receive_telemetry(
    connection: socket.socket,
    buffer: bytes,
    current: SafetyData,
) -> tuple[bytes, SafetyData, bool]:
    received = False
    while True:
        try:
            packet = connection.recv(4096)
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
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)
    front = 0.5 * (points[0] + points[1])
    heading = front - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)
    return center, heading


def signed_angle(heading: np.ndarray, vector: np.ndarray) -> float:
    hx, hy = float(heading[0]), -float(heading[1])
    vx, vy = float(vector[0]), -float(vector[1])
    return math.atan2(
        hx * vy - hy * vx,
        hx * vx + hy * vy,
    )


def make_mask(frame: np.ndarray, color_name: str) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    for lower, upper in HSV_RANGES[color_name]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_largest_target(
    frame: np.ndarray,
    color_name: str,
) -> tuple[np.ndarray | None, tuple[int, int, int, int] | None, np.ndarray]:
    mask = make_mask(frame, color_name)
    contours, _hierarchy = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    valid = [
        contour
        for contour in contours
        if cv2.contourArea(contour) >= MIN_CONTOUR_AREA
    ]
    if not valid:
        return None, None, mask

    contour = max(valid, key=cv2.contourArea)
    x, y, width, height = cv2.boundingRect(contour)

    moments = cv2.moments(contour)
    if abs(moments["m00"]) > 1e-9:
        center = np.array(
            [
                moments["m10"] / moments["m00"],
                moments["m01"] / moments["m00"],
            ],
            dtype=np.float32,
        )
    else:
        center = np.array(
            [x + width / 2, y + height / 2],
            dtype=np.float32,
        )

    return center, (x, y, width, height), mask


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

    selected_color = "RED"
    paused = True
    telemetry_buffer = b""
    safety = SafetyData()
    last_telemetry_time = 0.0
    previous_send = 0.0

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Не удалось получить кадр камеры")

            telemetry_buffer, safety, received = receive_telemetry(
                connection,
                telemetry_buffer,
                safety,
            )
            if received:
                last_telemetry_time = time.monotonic()

            target, box, mask = find_largest_target(frame, selected_color)
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
            state = "PAUSED" if paused else "SEARCH"

            telemetry_stale = (
                time.monotonic() - last_telemetry_time
                > TELEMETRY_TIMEOUT_SECONDS
            )
            black_border = (
                safety.line_left < LEFT_LINE_THRESHOLD
                or safety.line_right < RIGHT_LINE_THRESHOLD
            )
            obstacle = 0 < safety.ir_cm <= OBSTACLE_STOP_CM

            if telemetry_stale:
                state = "WAITING FOR TELEMETRY"
            elif black_border:
                state = "BLACK BORDER: STOP"
                paused = True
            elif obstacle:
                state = f"OBSTACLE AT {safety.ir_cm} CM"
            elif paused:
                state = "PAUSED"
            elif target is None:
                state = f"NO {selected_color} TARGET"
            elif ROBOT_MARKER_ID not in markers:
                state = f"NO ROBOT MARKER {ROBOT_MARKER_ID}"
            else:
                robot, heading = marker_geometry(markers[ROBOT_MARKER_ID])
                vector = target - robot
                distance = float(np.linalg.norm(vector))
                error = signed_angle(heading, vector)

                robot_pixel = tuple(np.rint(robot).astype(int))
                target_pixel = tuple(np.rint(target).astype(int))
                cv2.arrowedLine(
                    frame,
                    robot_pixel,
                    target_pixel,
                    DRAW_COLORS[selected_color],
                    3,
                )

                if distance <= TARGET_TOLERANCE_PX:
                    state = "TARGET REACHED"
                elif abs(error) > ANGLE_TOLERANCE_RAD:
                    angular = (
                        ANGULAR_SPEED_MRAD_S
                        if error > 0
                        else -ANGULAR_SPEED_MRAD_S
                    )
                    command = f"VEL 0 {angular}"
                    state = f"TURN {math.degrees(error):.1f} DEG"
                else:
                    command = f"VEL {LINEAR_SPEED_MM_S} 0"
                    state = f"DRIVE {distance:.0f} PX"

            if target is not None and box is not None:
                x, y, width, height = box
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + width, y + height),
                    DRAW_COLORS[selected_color],
                    3,
                )
                center_pixel = tuple(np.rint(target).astype(int))
                cv2.circle(
                    frame,
                    center_pixel,
                    6,
                    DRAW_COLORS[selected_color],
                    -1,
                )

            now = time.monotonic()
            if now - previous_send >= SEND_PERIOD_SECONDS:
                send(connection, command)
                previous_send = now

            cv2.putText(
                frame,
                f"COLOR: {selected_color}  STATE: {state}",
                (18, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                DRAW_COLORS[selected_color],
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"COMMAND: {command} | 1 red 2 green 3 blue | SPACE pause",
                (18, 66),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.53,
                (230, 230, 230),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Color target mission", frame)
            cv2.imshow("Color mask", mask)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break
            if key == ord("1"):
                selected_color = "RED"
            if key == ord("2"):
                selected_color = "GREEN"
            if key == ord("3"):
                selected_color = "BLUE"
            if key == ord(" "):
                paused = not paused
                if paused:
                    send(connection, "STOP")

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

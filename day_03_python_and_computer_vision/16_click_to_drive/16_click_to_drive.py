#!/usr/bin/env python3
"""Щелчок по изображению задаёт цель роботу с ArUco-маркером ID 4.

Левая кнопка мыши задаёт новую цель.
Пробел или правая кнопка мыши удаляет цель и останавливает робота.
Esc останавливает робота и завершает программу.
"""

import math
import socket
import time

import cv2
import numpy as np


CAMERA_INDEX = 0
ROBOT_MARKER_ID = 4
ROBOT_ADDRESS = ("192.168.4.1", 8888)

LINEAR_SPEED_MM_S = 220
ANGULAR_SPEED_MRAD_S = 3500
TARGET_TOLERANCE_PX = 70
ANGLE_TOLERANCE_RAD = math.radians(10)
SEND_PERIOD_SECONDS = 0.05


target_pixel: tuple[int, int] | None = None


def send(connection: socket.socket, command: str) -> None:
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except OSError:
        pass


def drain_telemetry(connection: socket.socket) -> None:
    """Освобождает входной TCP-буфер от телеметрии, не разбирая её."""
    try:
        while connection.recv(2048):
            pass
    except BlockingIOError:
        pass


def mouse_callback(
    event: int,
    x: int,
    y: int,
    _flags: int,
    _data: object,
) -> None:
    global target_pixel

    if event == cv2.EVENT_LBUTTONDOWN:
        target_pixel = (x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        target_pixel = None


def robot_geometry(corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Возвращает центр маркера и единичный вектор направления робота."""
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)

    # Верхняя сторона ArUco должна быть направлена вперёд по корпусу робота.
    front = 0.5 * (points[0] + points[1])
    heading = front - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)
    return center, heading


def signed_angle(heading: np.ndarray, target_vector: np.ndarray) -> float:
    """Возвращает угол от направления робота к цели со знаком."""
    # Координаты изображения: X вправо, Y вниз.
    # Для математического расчёта разворачиваем ось Y вверх.
    hx, hy = float(heading[0]), -float(heading[1])
    tx, ty = float(target_vector[0]), -float(target_vector[1])

    cross = hx * ty - hy * tx
    dot = hx * tx + hy * ty
    return math.atan2(cross, dot)


def main() -> None:
    global target_pixel

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        cv2.aruco.DetectorParameters(),
    )

    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась.")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    window = "Click target"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, mouse_callback)
    previous_send_time = 0.0

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            drain_telemetry(connection)
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
            state = "CLICK TARGET"

            if target_pixel is not None:
                cv2.circle(
                    frame,
                    target_pixel,
                    TARGET_TOLERANCE_PX,
                    (0, 170, 255),
                    2,
                )

                if ROBOT_MARKER_ID not in markers:
                    state = f"WAIT FOR ROBOT MARKER ID {ROBOT_MARKER_ID}"
                else:
                    robot_center, robot_heading = robot_geometry(
                        markers[ROBOT_MARKER_ID]
                    )
                    target = np.array(target_pixel, dtype=np.float32)
                    target_vector = target - robot_center
                    distance = float(np.linalg.norm(target_vector))
                    angle_error = signed_angle(robot_heading, target_vector)

                    robot_pixel = tuple(np.rint(robot_center).astype(int))
                    cv2.arrowedLine(
                        frame,
                        robot_pixel,
                        target_pixel,
                        (255, 80, 0),
                        3,
                    )

                    if distance <= TARGET_TOLERANCE_PX:
                        command = "STOP"
                        state = "TARGET REACHED"
                    elif abs(angle_error) > ANGLE_TOLERANCE_RAD:
                        angular = (
                            ANGULAR_SPEED_MRAD_S
                            if angle_error > 0
                            else -ANGULAR_SPEED_MRAD_S
                        )
                        command = f"VEL 0 {angular}"
                        state = "TURN"
                    else:
                        command = f"VEL {LINEAR_SPEED_MM_S} 0"
                        state = "DRIVE"

                    cv2.putText(
                        frame,
                        f"distance={distance:.0f}px  "
                        f"error={math.degrees(angle_error):.1f}deg",
                        (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (255, 255, 255),
                        2,
                    )

            cv2.putText(
                frame,
                state,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 0) if command != "STOP" else (0, 100, 255),
                2,
            )
            cv2.putText(
                frame,
                "LMB: target  RMB/SPACE: stop  ESC: exit",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
            )

            now = time.monotonic()
            if now - previous_send_time >= SEND_PERIOD_SECONDS:
                send(connection, command)
                previous_send_time = now

            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord(" "):
                target_pixel = None
                send(connection, "STOP")

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

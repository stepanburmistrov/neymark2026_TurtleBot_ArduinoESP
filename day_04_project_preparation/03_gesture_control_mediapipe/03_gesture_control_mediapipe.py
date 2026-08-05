#!/usr/bin/env python3
"""Управление роботом жестами одной руки через MediaPipe.

Большой палец намеренно не участвует. Используются четыре состояния:

    указательный                         -> поворот влево
    мизинец                              -> поворот вправо
    указательный + средний + безымянный + мизинец -> движение вперёд
    указательный + мизинец              -> движение назад
    любое другое сочетание              -> STOP

Команды отправляются компьютером по TCP на ESP32, а не напрямую в Arduino.
"""

from __future__ import annotations

from collections import deque
import math
import socket
import time
from typing import Any

import cv2
import numpy as np

try:
    import mediapipe as mp
except ImportError as exc:
    raise SystemExit(
        "Не установлен mediapipe. Выполните: "
        "python -m pip install -r requirements.txt"
    ) from exc


CAMERA_INDEX = 0
ROBOT_ADDRESS = ("192.168.4.1", 8888)

LINEAR_SPEED_MM_S = 220
ANGULAR_SPEED_MRAD_S = 3300
SEND_PERIOD_SECONDS = 0.08
STABLE_FRAMES = 4

# Минимальный угол в суставах для признания пальца разогнутым.
PIP_EXTENSION_ANGLE_DEG = 150.0
DIP_EXTENSION_ANGLE_DEG = 145.0

# Индексы точек MediaPipe: MCP, PIP, DIP, TIP.
FINGER_LANDMARKS = {
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}

COMMANDS = {
    "LEFT": f"VEL 0 {ANGULAR_SPEED_MRAD_S}",
    "RIGHT": f"VEL 0 {-ANGULAR_SPEED_MRAD_S}",
    "FORWARD": f"VEL {LINEAR_SPEED_MM_S} 0",
    "BACKWARD": f"VEL {-LINEAR_SPEED_MM_S} 0",
    "STOP": "STOP",
}


# Соединения между 21 точкой кисти для отрисовки скелета.
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def as_vector(point: Any) -> np.ndarray:
    """Преобразует landmark MediaPipe в трёхмерный вектор NumPy."""
    return np.array([point.x, point.y, point.z], dtype=np.float64)


def joint_angle(a: Any, b: Any, c: Any) -> float:
    """Возвращает угол ABC в градусах."""
    vector_ba = as_vector(a) - as_vector(b)
    vector_bc = as_vector(c) - as_vector(b)

    denominator = float(
        np.linalg.norm(vector_ba) * np.linalg.norm(vector_bc)
    )
    if denominator < 1e-9:
        return 0.0

    cosine = float(np.dot(vector_ba, vector_bc) / denominator)
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def is_finger_extended(
    landmarks: list[Any],
    indices: tuple[int, int, int, int],
) -> bool:
    """Определяет разгибание пальца по углам PIP и DIP.

    Этот способ меньше зависит от расстояния руки до камеры, чем простая
    проверка расстояния между основанием и кончиком пальца.
    """
    mcp_index, pip_index, dip_index, tip_index = indices

    pip_angle = joint_angle(
        landmarks[mcp_index],
        landmarks[pip_index],
        landmarks[dip_index],
    )
    dip_angle = joint_angle(
        landmarks[pip_index],
        landmarks[dip_index],
        landmarks[tip_index],
    )

    wrist = as_vector(landmarks[0])
    pip_distance = float(
        np.linalg.norm(as_vector(landmarks[pip_index]) - wrist)
    )
    tip_distance = float(
        np.linalg.norm(as_vector(landmarks[tip_index]) - wrist)
    )

    return (
        pip_angle >= PIP_EXTENSION_ANGLE_DEG
        and dip_angle >= DIP_EXTENSION_ANGLE_DEG
        and tip_distance > pip_distance
    )


def read_finger_states(landmarks: list[Any]) -> tuple[int, int, int, int]:
    """Возвращает состояния: указательный, средний, безымянный, мизинец."""
    return tuple(
        int(is_finger_extended(landmarks, indices))
        for indices in FINGER_LANDMARKS.values()
    )  # type: ignore[return-value]


def classify_gesture(states: tuple[int, int, int, int]) -> str:
    """Преобразует четыре двоичных состояния в название команды."""
    mapping = {
        (1, 0, 0, 0): "LEFT",
        (0, 0, 0, 1): "RIGHT",
        (1, 1, 1, 1): "FORWARD",
        (1, 0, 0, 1): "BACKWARD",
    }
    return mapping.get(states, "STOP")


def send(connection: socket.socket, command: str) -> None:
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except (BlockingIOError, OSError):
        pass


def drain_telemetry(connection: socket.socket) -> None:
    """Удаляет входящую телеметрию, чтобы TCP-буфер не переполнялся."""
    while True:
        try:
            packet = connection.recv(4096)
        except BlockingIOError:
            return
        if not packet:
            raise ConnectionError("TCP-соединение закрыто")


def draw_hand(frame: np.ndarray, landmarks: list[Any]) -> None:
    height, width = frame.shape[:2]
    pixels = [
        (int(point.x * width), int(point.y * height))
        for point in landmarks
    ]

    for start, end in HAND_CONNECTIONS:
        cv2.line(frame, pixels[start], pixels[end], (180, 180, 180), 2)
    for index, pixel in enumerate(pixels):
        radius = 7 if index in (8, 12, 16, 20) else 4
        cv2.circle(frame, pixel, radius, (0, 220, 255), -1)


def main() -> None:
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    RunningMode = mp.tasks.vision.RunningMode

    model_path = str(
        (__import__('pathlib').Path(__file__).parent / 'hand_landmarker.task')
    )
    if not __import__('pathlib').Path(model_path).exists():
        raise SystemExit(
            "Не найден hand_landmarker.task. Запустите download_model.py "
            "или скачайте модель по инструкции в README."
        )

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.65,
        min_hand_presence_confidence=0.65,
        min_tracking_confidence=0.55,
    )

    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    gesture_history: deque[str] = deque(maxlen=STABLE_FRAMES)
    active_gesture = "STOP"
    previous_send = 0.0
    start_time = time.monotonic()

    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Не удалось получить кадр камеры")

                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb,
                )
                timestamp_ms = int((time.monotonic() - start_time) * 1000)
                result = landmarker.detect_for_video(mp_image, timestamp_ms)

                states = (0, 0, 0, 0)
                detected_gesture = "STOP"

                if result.hand_landmarks:
                    landmarks = result.hand_landmarks[0]
                    states = read_finger_states(landmarks)
                    detected_gesture = classify_gesture(states)
                    draw_hand(frame, landmarks)

                gesture_history.append(detected_gesture)
                if (
                    len(gesture_history) == STABLE_FRAMES
                    and len(set(gesture_history)) == 1
                ):
                    active_gesture = gesture_history[0]

                command = COMMANDS[active_gesture]
                now = time.monotonic()
                if now - previous_send >= SEND_PERIOD_SECONDS:
                    send(connection, command)
                    previous_send = now

                drain_telemetry(connection)

                state_text = " ".join(map(str, states))
                cv2.putText(
                    frame,
                    f"INDEX MIDDLE RING PINKY: {state_text}",
                    (20, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (255, 220, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"GESTURE: {active_gesture}    COMMAND: {command}",
                    (20, 72),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (0, 255, 0) if active_gesture != "STOP" else (0, 100, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    "ESC: safe stop",
                    (20, frame.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )

                cv2.imshow("MediaPipe gesture control", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

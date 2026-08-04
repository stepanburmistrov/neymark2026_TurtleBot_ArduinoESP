#!/usr/bin/env python3
"""Движение по чёрной линии: решение принимает Python."""

from dataclasses import dataclass
import socket
import time


ROBOT_ADDRESS = ("192.168.4.1", 8888)
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.7

# Белая поверхность даёт значение выше порога, чёрная линия — ниже.
# Эти значения обязательно заменяются результатами калибровки конкретного робота.
LEFT_BLACK_THRESHOLD = 500
RIGHT_BLACK_THRESHOLD = 500

FORWARD_SPEED_MM_S = 220
CORRECTION_LINEAR_MM_S = 120
TURN_SPEED_MRAD_S = 2800


@dataclass
class LineTelemetry:
    left: int = 1023
    right: int = 1023


def parse_line_telemetry(line: str) -> LineTelemetry | None:
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return LineTelemetry(
            left=int(parts[8]),
            right=int(parts[9]),
        )
    except ValueError:
        return None


def send(connection: socket.socket, command: str) -> None:
    connection.sendall((command + "\n").encode("ascii"))


def receive_available(
    connection: socket.socket,
    buffer: bytes,
    telemetry: LineTelemetry,
) -> tuple[bytes, LineTelemetry, bool]:
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
        parsed = parse_line_telemetry(
            raw_line.decode("ascii", errors="replace").strip()
        )
        if parsed is not None:
            telemetry = parsed
            received = True

    return buffer, telemetry, received


def choose_command(data: LineTelemetry) -> tuple[str, str]:
    black_left = data.left < LEFT_BLACK_THRESHOLD
    black_right = data.right < RIGHT_BLACK_THRESHOLD

    if black_left and black_right:
        return f"VEL {FORWARD_SPEED_MM_S} 0", "оба датчика видят линию"
    if black_left:
        return (
            f"VEL {CORRECTION_LINEAR_MM_S} {TURN_SPEED_MRAD_S}",
            "линия слева — поворот влево",
        )
    if black_right:
        return (
            f"VEL {CORRECTION_LINEAR_MM_S} {-TURN_SPEED_MRAD_S}",
            "линия справа — поворот вправо",
        )

    return f"VEL {FORWARD_SPEED_MM_S} 0", "линия между датчиками — прямо"


def main() -> None:
    print("Установите робота над линией. Для остановки нажмите Ctrl+C.")
    input("Нажмите Enter для запуска...")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    buffer = b""
    telemetry = LineTelemetry()
    last_telemetry_time = 0.0
    last_state = ""

    try:
        while True:
            buffer, telemetry, received = receive_available(
                connection,
                buffer,
                telemetry,
            )
            if received:
                last_telemetry_time = time.monotonic()

            if time.monotonic() - last_telemetry_time > TELEMETRY_TIMEOUT_SECONDS:
                command = "STOP"
                state = "ожидание телеметрии"
            else:
                command, state = choose_command(telemetry)

            send(connection, command)

            if state != last_state:
                print(
                    f"{state:34s}  "
                    f"L={telemetry.left:4d}  R={telemetry.right:4d}  "
                    f"команда={command}"
                )
                last_state = state

            time.sleep(SEND_PERIOD_SECONDS)

    except KeyboardInterrupt:
        print("\nОстановка.")
    finally:
        try:
            send(connection, "STOP")
        finally:
            connection.close()


if __name__ == "__main__":
    main()

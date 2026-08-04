#!/usr/bin/env python3
"""Движение вперёд до стены с остановкой по ИК-дальномеру."""

from dataclasses import dataclass
import socket
import time


ROBOT_ADDRESS = ("192.168.4.1", 8888)
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.7

FORWARD_SPEED_MM_S = 180
STOP_DISTANCE_CM = 22
REQUIRED_CLOSE_MEASUREMENTS = 3


@dataclass
class DistanceTelemetry:
    ir_cm: int = 60


def parse_distance(line: str) -> DistanceTelemetry | None:
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return DistanceTelemetry(ir_cm=int(parts[10]))
    except ValueError:
        return None


def send(connection: socket.socket, command: str) -> None:
    connection.sendall((command + "\n").encode("ascii"))


def receive_available(
    connection: socket.socket,
    buffer: bytes,
    current: DistanceTelemetry,
) -> tuple[bytes, DistanceTelemetry, bool]:
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
        parsed = parse_distance(
            raw_line.decode("ascii", errors="replace").strip()
        )
        if parsed is not None:
            current = parsed
            received = True

    return buffer, current, received


def main() -> None:
    print(
        f"Робот поедет вперёд и остановится примерно в {STOP_DISTANCE_CM} см "
        "от препятствия."
    )
    input("Нажмите Enter для запуска...")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    buffer = b""
    telemetry = DistanceTelemetry()
    last_telemetry_time = 0.0
    close_count = 0
    last_print_time = 0.0

    try:
        while True:
            buffer, telemetry, received = receive_available(
                connection,
                buffer,
                telemetry,
            )

            now = time.monotonic()
            if received:
                last_telemetry_time = now
                if 2 <= telemetry.ir_cm <= STOP_DISTANCE_CM:
                    close_count += 1
                else:
                    close_count = 0

            if now - last_telemetry_time > TELEMETRY_TIMEOUT_SECONDS:
                send(connection, "STOP")
                print("Телеметрия потеряна — STOP")
                time.sleep(SEND_PERIOD_SECONDS)
                continue

            if close_count >= REQUIRED_CLOSE_MEASUREMENTS:
                send(connection, "STOP")
                print(f"Стена обнаружена: {telemetry.ir_cm} см. Робот остановлен.")
                break

            send(connection, f"VEL {FORWARD_SPEED_MM_S} 0")

            if now - last_print_time >= 0.2:
                print(f"Расстояние: {telemetry.ir_cm:3d} см", end="\r")
                last_print_time = now

            time.sleep(SEND_PERIOD_SECONDS)

    except KeyboardInterrupt:
        print("\nОстановка оператором.")
    finally:
        try:
            send(connection, "STOP")
        finally:
            connection.close()


if __name__ == "__main__":
    main()

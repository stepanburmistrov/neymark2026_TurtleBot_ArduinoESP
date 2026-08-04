#!/usr/bin/env python3
"""Простейшее чтение и разбор телеметрии робота по TCP."""

from dataclasses import dataclass
import socket


ROBOT_ADDRESS = ("192.168.4.1", 8888)


@dataclass
class Telemetry:
    x_mm: int
    y_mm: int
    theta_mrad: int
    linear_mm_s: int
    angular_mrad_s: int
    encoder_left: int
    encoder_right: int
    line_left: int
    line_right: int
    ir_cm: int
    servo_deg: int


def parse_telemetry(line: str) -> Telemetry | None:
    """Преобразует строку TEL в объект Telemetry."""
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return Telemetry(*map(int, parts[1:]))
    except ValueError:
        return None


def main() -> None:
    print(f"Подключение к {ROBOT_ADDRESS[0]}:{ROBOT_ADDRESS[1]}...")

    with socket.create_connection(ROBOT_ADDRESS, timeout=3.0) as connection:
        # Просим Arduino немедленно отправить текущее состояние.
        connection.sendall(b"GET\n")

        # makefile() позволяет читать TCP-поток как обычный текстовый файл.
        with connection.makefile("r", encoding="ascii", newline="\n") as stream:
            try:
                for raw_line in stream:
                    line = raw_line.strip()
                    telemetry = parse_telemetry(line)

                    if telemetry is None:
                        if line:
                            print("Сообщение:", line)
                        continue

                    angle_deg = telemetry.theta_mrad / 1000 * 180 / 3.14159265
                    print(
                        f"X={telemetry.x_mm:5d} мм  "
                        f"Y={telemetry.y_mm:5d} мм  "
                        f"θ={angle_deg:6.1f}°  "
                        f"линия={telemetry.line_left:4d}/"
                        f"{telemetry.line_right:4d}  "
                        f"ИК={telemetry.ir_cm:3d} см"
                    )
            except KeyboardInterrupt:
                print("\nЧтение завершено.")


if __name__ == "__main__":
    main()

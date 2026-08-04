#!/usr/bin/env python3
"""Передача текстовых команд роботу по TCP из консоли."""

import socket
import threading


ROBOT_ADDRESS = ("192.168.4.1", 8888)

ALIASES = {
    "w": "VEL 250 0",
    "s": "VEL -250 0",
    "a": "VEL 0 3500",
    "d": "VEL 0 -3500",
    "x": "STOP",
}


def receive_lines(connection: socket.socket) -> None:
    """Параллельно печатает ответы и телеметрию робота."""
    buffer = b""

    try:
        while True:
            packet = connection.recv(2048)
            if not packet:
                return
            buffer += packet

            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                line = raw_line.decode("ascii", errors="replace").strip()
                if line:
                    print(f"\n< {line}")
                    print("> ", end="", flush=True)
    except OSError:
        return


def main() -> None:
    print(f"Подключение к {ROBOT_ADDRESS[0]}:{ROBOT_ADDRESS[1]}...")

    with socket.create_connection(ROBOT_ADDRESS, timeout=3.0) as connection:
        reader = threading.Thread(
            target=receive_lines,
            args=(connection,),
            daemon=True,
        )
        reader.start()

        print("Быстрые клавиши: w — вперёд, s — назад, a/d — поворот, x — стоп")
        print("Можно вводить полные команды: VEL 200 0, SERVO 90, GET")
        print("Для выхода: quit")

        try:
            while True:
                user_text = input("> ").strip()
                if not user_text:
                    continue
                if user_text.lower() in {"quit", "exit"}:
                    break

                command = ALIASES.get(user_text.lower(), user_text)
                connection.sendall((command + "\n").encode("ascii"))
                print(f">> {command}")
        except (KeyboardInterrupt, EOFError):
            pass
        finally:
            try:
                connection.sendall(b"STOP\n")
            except OSError:
                pass


if __name__ == "__main__":
    main()

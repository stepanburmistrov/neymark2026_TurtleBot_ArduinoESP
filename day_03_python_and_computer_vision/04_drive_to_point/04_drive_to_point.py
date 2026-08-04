#!/usr/bin/env python3
"""Последовательное движение по точкам, вводимым из консоли.

Для каждой новой точки робот выполняет только две фазы:

1. Поворачивается на месте в сторону цели.
2. Едет прямо с постоянной линейной скоростью.

Во время прямого движения курс больше не корректируется. После остановки
робот также не доворачивается. Следующая введённая точка запускает новый
поворот и новый прямой отрезок маршрута.
"""

from dataclasses import dataclass
import math
import socket
import time


ROBOT_ADDRESS = ("192.168.4.1", 8888)
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.7
POSE_WAIT_TIMEOUT_SECONDS = 3.0

# Скорость прямого движения постоянна на всём отрезке.
LINEAR_SPEED_MM_S = 220

# Поворот выполняется отдельно, до начала прямого движения.
MAX_TURN_SPEED_MRAD_S = 3200
MIN_TURN_SPEED_MRAD_S = 900
TURN_GAIN = 2600.0
ANGLE_TOLERANCE_RAD = math.radians(7)
TURN_SETTLE_SECONDS = 0.15

POSITION_TOLERANCE_MM = 55
SEGMENT_TIME_RESERVE = 3.0


@dataclass
class Pose:
    x_mm: int = 0
    y_mm: int = 0
    theta_mrad: int = 0

    @property
    def theta_rad(self) -> float:
        return self.theta_mrad / 1000.0


@dataclass
class Segment:
    start_x_mm: float
    start_y_mm: float
    target_x_mm: float
    target_y_mm: float
    length_mm: float
    unit_x: float
    unit_y: float


def parse_pose(line: str) -> Pose | None:
    """Извлекает X, Y и угол из строки телеметрии TEL."""
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None

    try:
        return Pose(
            x_mm=int(parts[1]),
            y_mm=int(parts[2]),
            theta_mrad=int(parts[3]),
        )
    except ValueError:
        return None


def normalize_angle(angle: float) -> float:
    """Приводит угол к диапазону от -π до +π."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def send(connection: socket.socket, command: str) -> None:
    connection.sendall((command + "\n").encode("ascii"))


def receive_available(
    connection: socket.socket,
    buffer: bytes,
    current_pose: Pose,
) -> tuple[bytes, Pose, bool]:
    """Читает все доступные пакеты и возвращает последнюю позу."""
    received_pose = False

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
        line = raw_line.decode("ascii", errors="replace").strip()
        parsed = parse_pose(line)
        if parsed is not None:
            current_pose = parsed
            received_pose = True

    return buffer, current_pose, received_pose


def wait_for_pose(
    connection: socket.socket,
    buffer: bytes,
    current_pose: Pose,
    timeout_seconds: float = POSE_WAIT_TIMEOUT_SECONDS,
) -> tuple[bytes, Pose]:
    """Запрашивает и ожидает свежую телеметрию."""
    deadline = time.monotonic() + timeout_seconds
    send(connection, "GET")

    while time.monotonic() < deadline:
        buffer, current_pose, received = receive_available(
            connection,
            buffer,
            current_pose,
        )
        if received:
            return buffer, current_pose
        time.sleep(0.01)

    send(connection, "STOP")
    raise TimeoutError("Нет свежей телеметрии от робота")


def turn_speed(angle_error_rad: float) -> int:
    """Вычисляет угловую скорость только для фазы поворота."""
    value = clamp(
        TURN_GAIN * angle_error_rad,
        -MAX_TURN_SPEED_MRAD_S,
        MAX_TURN_SPEED_MRAD_S,
    )

    # Небольшой минимальный ШИМ нужен, чтобы робот не остановился раньше,
    # чем преодолеет трение механики.
    if 0 < abs(value) < MIN_TURN_SPEED_MRAD_S:
        value = math.copysign(MIN_TURN_SPEED_MRAD_S, value)

    return int(value)


def turn_to_target(
    connection: socket.socket,
    buffer: bytes,
    pose: Pose,
    target_x_mm: float,
    target_y_mm: float,
) -> tuple[bytes, Pose]:
    """Один раз разворачивает робота в сторону новой точки."""
    dx = target_x_mm - pose.x_mm
    dy = target_y_mm - pose.y_mm
    target_heading = math.atan2(dy, dx)
    last_telemetry_time = time.monotonic()

    print("Поворот к новой точке...")

    while True:
        buffer, pose, received = receive_available(connection, buffer, pose)
        now = time.monotonic()
        if received:
            last_telemetry_time = now

        if now - last_telemetry_time > TELEMETRY_TIMEOUT_SECONDS:
            send(connection, "STOP")
            raise TimeoutError("Телеметрия пропала во время поворота")

        angle_error = normalize_angle(target_heading - pose.theta_rad)

        if abs(angle_error) <= ANGLE_TOLERANCE_RAD:
            send(connection, "STOP")
            print(
                f"Направление задано: θ={math.degrees(pose.theta_rad):.1f}°, "
                f"ошибка={math.degrees(angle_error):.1f}°"
            )
            time.sleep(TURN_SETTLE_SECONDS)
            return wait_for_pose(connection, buffer, pose)

        angular_speed = turn_speed(angle_error)
        send(connection, f"VEL 0 {angular_speed}")

        print(
            f"  θ={math.degrees(pose.theta_rad):7.1f}°  "
            f"ошибка={math.degrees(angle_error):7.1f}°  "
            f"VEL 0 {angular_speed:5d}",
            end="\r",
        )
        time.sleep(SEND_PERIOD_SECONDS)


def make_segment(pose: Pose, target_x_mm: float, target_y_mm: float) -> Segment:
    """Фиксирует прямой отрезок после завершения поворота."""
    dx = target_x_mm - pose.x_mm
    dy = target_y_mm - pose.y_mm
    length = math.hypot(dx, dy)

    if length <= POSITION_TOLERANCE_MM:
        unit_x = 1.0
        unit_y = 0.0
    else:
        unit_x = dx / length
        unit_y = dy / length

    return Segment(
        start_x_mm=pose.x_mm,
        start_y_mm=pose.y_mm,
        target_x_mm=target_x_mm,
        target_y_mm=target_y_mm,
        length_mm=length,
        unit_x=unit_x,
        unit_y=unit_y,
    )


def segment_progress(segment: Segment, pose: Pose) -> tuple[float, float, float]:
    """Возвращает прогресс, остаток по направлению и боковое отклонение."""
    moved_x = pose.x_mm - segment.start_x_mm
    moved_y = pose.y_mm - segment.start_y_mm

    progress = moved_x * segment.unit_x + moved_y * segment.unit_y
    remaining = segment.length_mm - progress

    # Поперечное расстояние от текущей точки до линии отрезка.
    lateral = -moved_x * segment.unit_y + moved_y * segment.unit_x
    return progress, remaining, lateral


def drive_straight_to_target(
    connection: socket.socket,
    buffer: bytes,
    pose: Pose,
    target_x_mm: float,
    target_y_mm: float,
) -> tuple[bytes, Pose]:
    """Едет к точке прямо, без поворотов и подруливания."""
    segment = make_segment(pose, target_x_mm, target_y_mm)

    if segment.length_mm <= POSITION_TOLERANCE_MM:
        send(connection, "STOP")
        print("Точка уже находится внутри допуска.")
        return buffer, pose

    # Ограничение времени — страховка на случай пробуксовки или потери робота.
    expected_time = segment.length_mm / max(LINEAR_SPEED_MM_S, 1)
    deadline = time.monotonic() + expected_time * SEGMENT_TIME_RESERVE + 3.0
    last_telemetry_time = time.monotonic()

    print(
        f"Прямой отрезок: {segment.length_mm:.0f} мм, "
        f"скорость: {LINEAR_SPEED_MM_S} мм/с"
    )

    while True:
        buffer, pose, received = receive_available(connection, buffer, pose)
        now = time.monotonic()
        if received:
            last_telemetry_time = now

        if now - last_telemetry_time > TELEMETRY_TIMEOUT_SECONDS:
            send(connection, "STOP")
            raise TimeoutError("Телеметрия пропала во время движения")

        progress, remaining, lateral = segment_progress(segment, pose)
        distance_to_target = math.hypot(
            target_x_mm - pose.x_mm,
            target_y_mm - pose.y_mm,
        )

        # Остановка происходит около цели либо после прохождения поперечной
        # линии, проведённой через цель. Это не даёт роботу бесконечно ехать,
        # если из-за механической погрешности он прошёл немного сбоку.
        target_reached = (
            distance_to_target <= POSITION_TOLERANCE_MM
            or remaining <= POSITION_TOLERANCE_MM
        )

        if target_reached:
            send(connection, "STOP")
            print(
                f"\nТочка достигнута: X={pose.x_mm} мм, Y={pose.y_mm} мм, "
                f"остаток={max(remaining, 0.0):.0f} мм, "
                f"боковое отклонение={lateral:.0f} мм"
            )
            return buffer, pose

        if now >= deadline:
            send(connection, "STOP")
            raise TimeoutError("Превышено безопасное время движения к точке")

        # ВАЖНО: после начала этой фазы команда не меняется.
        # Угловая скорость равна нулю, поэтому робот не дёргается и не
        # пытается повторно повернуть во время движения.
        send(connection, f"VEL {LINEAR_SPEED_MM_S} 0")

        print(
            f"  X={pose.x_mm:6d}  Y={pose.y_mm:6d}  "
            f"пройдено={progress:7.0f}/{segment.length_mm:.0f} мм  "
            f"до цели={distance_to_target:6.0f} мм  "
            f"отклонение={lateral:6.0f} мм",
            end="\r",
        )
        time.sleep(SEND_PERIOD_SECONDS)


def go_to_point(
    connection: socket.socket,
    buffer: bytes,
    pose: Pose,
    target_x_mm: float,
    target_y_mm: float,
) -> tuple[bytes, Pose]:
    """Выполняет один шаг маршрута: поворот, затем прямое движение."""
    buffer, pose = wait_for_pose(connection, buffer, pose)

    distance = math.hypot(target_x_mm - pose.x_mm, target_y_mm - pose.y_mm)
    if distance <= POSITION_TOLERANCE_MM:
        send(connection, "STOP")
        print("Робот уже находится около этой точки.")
        return buffer, pose

    buffer, pose = turn_to_target(
        connection,
        buffer,
        pose,
        target_x_mm,
        target_y_mm,
    )
    return drive_straight_to_target(
        connection,
        buffer,
        pose,
        target_x_mm,
        target_y_mm,
    )


def read_target() -> tuple[float, float] | str | None:
    """Читает следующую абсолютную точку или консольную команду."""
    raw = input("\nСледующая точка X Y в мм > ").strip()
    if not raw:
        return None

    command = raw.lower()
    if command in {"q", "quit", "exit", "выход"}:
        return "exit"
    if command in {"r", "reset", "сброс"}:
        return "reset"
    if command in {"s", "stop", "стоп"}:
        return "stop"
    if command in {"p", "pose", "позиция"}:
        return "pose"

    parts = raw.replace(",", ".").split()
    if len(parts) != 2:
        print("Нужно ввести две координаты, например: 1000 500")
        return None

    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        print("Координаты должны быть числами, например: -300 800")
        return None


def print_help() -> None:
    print(
        "\nКоманды:\n"
        "  X Y    — ехать к абсолютной точке одометрии, например 1000 500\n"
        "  pose   — показать текущие координаты\n"
        "  reset  — принять текущее положение за (0, 0, 0)\n"
        "  stop   — отправить STOP\n"
        "  exit   — остановиться и завершить программу\n"
        "\nПример квадратного маршрута:\n"
        "  1000 0\n"
        "  1000 700\n"
        "  0 700\n"
        "  0 0\n"
    )


def main() -> None:
    print("Последовательное движение по точкам из консоли")
    print(f"Подключение к {ROBOT_ADDRESS[0]}:{ROBOT_ADDRESS[1]}...")

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)

    buffer = b""
    pose = Pose()

    try:
        send(connection, "STOP")
        print_help()
        input(
            "Установите робота в начало координат, направьте вдоль +X "
            "и нажмите Enter..."
        )
        send(connection, "RESET_ODOM")
        time.sleep(0.4)
        buffer, pose = wait_for_pose(connection, buffer, pose)
        print("Одометрия сброшена: X=0, Y=0, θ=0.")

        while True:
            target = read_target()
            if target is None:
                continue

            if target == "exit":
                break
            if target == "stop":
                send(connection, "STOP")
                print("Робот остановлен.")
                continue
            if target == "reset":
                send(connection, "STOP")
                send(connection, "RESET_ODOM")
                time.sleep(0.4)
                buffer, pose = wait_for_pose(connection, buffer, pose)
                print("Текущее положение принято за X=0, Y=0, θ=0.")
                continue
            if target == "pose":
                buffer, pose = wait_for_pose(connection, buffer, pose)
                print(
                    f"X={pose.x_mm} мм, Y={pose.y_mm} мм, "
                    f"θ={math.degrees(pose.theta_rad):.1f}°"
                )
                continue

            target_x_mm, target_y_mm = target
            print(
                f"\nНовый шаг маршрута: "
                f"({pose.x_mm}, {pose.y_mm}) → "
                f"({target_x_mm:.0f}, {target_y_mm:.0f})"
            )

            try:
                buffer, pose = go_to_point(
                    connection,
                    buffer,
                    pose,
                    target_x_mm,
                    target_y_mm,
                )
            except TimeoutError as error:
                send(connection, "STOP")
                print(f"\nБезопасная остановка: {error}")

    except KeyboardInterrupt:
        print("\nОстановка оператором.")
    finally:
        try:
            send(connection, "STOP")
        finally:
            connection.close()


if __name__ == "__main__":
    main()

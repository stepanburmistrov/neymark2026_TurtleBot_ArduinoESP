#!/usr/bin/env python3
"""День 4. Движение к цели с обходом нарисованных препятствий.

Камера сверху видит реального робота с ArUco-маркером ID 4. Пользователь
рисует виртуальные запрещённые области непосредственно поверх видеокадра,
а правой кнопкой мыши задаёт цель. Программа строит путь алгоритмом A* и
последовательно ведёт физического робота по промежуточным точкам маршрута.

Управление:
    ЛКМ и перетаскивание         — рисовать препятствие;
    Shift + ЛКМ или СКМ          — стирать препятствие;
    ПКМ                          — задать новую цель и запустить движение;
    Space                        — пауза / продолжение;
    C                            — очистить все препятствия;
    R                            — удалить цель и маршрут;
    Esc                          — остановить робота и завершить программу.

Перед запуском:
    1. Камера должна быть неподвижной и смотреть сверху на всё поле.
    2. Сторона P0–P1 маркера ID 4 должна быть направлена вперёд по роботу.
    3. Компьютер должен быть подключён к сети ESP32-C3.
    4. Необходимо проверить пороги датчиков линии и дальномера.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import argparse
import heapq
import math
from pathlib import Path
import socket
import time

import cv2
import numpy as np


CAMERA_INDEX = 0
ROBOT_MARKER_ID = 4
ROBOT_ADDRESS = ("192.168.4.1", 8888)

# Скорости робота.
LINEAR_SPEED_MM_S = 220
ANGULAR_SPEED_MRAD_S = 3000

# Допуски визуального управления.
ANGLE_TOLERANCE_RAD = math.radians(11)
GOAL_TOLERANCE_PX = 42
WAYPOINT_TOLERANCE_PX = 28

# Параметры карты и планировщика.
GRID_CELL_SIZE_PX = 25
WALL_BRUSH_RADIUS_PX = 18
ROBOT_SAFETY_RADIUS_PX = 40

# Если центр робота уже оказался в оранжевой зоне, планировщик должен
# построить путь наружу, а не заблокировать стартовую клетку.
# Красная область препятствия при этом никогда не очищается.
START_ESCAPE_RADIUS_PX = ROBOT_SAFETY_RADIUS_PX + GRID_CELL_SIZE_PX

REPLAN_PERIOD_SECONDS = 0.65

# Параметры обмена и безопасности.
SEND_PERIOD_SECONDS = 0.05
TELEMETRY_TIMEOUT_SECONDS = 0.8

# Белое поле обычно даёт значение выше порога, чёрная граница — ниже.
# Значения необходимо проверить на конкретном роботе.
LEFT_LINE_THRESHOLD = 5
RIGHT_LINE_THRESHOLD = 5
OBSTACLE_STOP_CM = 14

INFO_PANEL_HEIGHT = 150

Cell = tuple[int, int]  # row, column


@dataclass
class SafetyData:
    """Поля телеметрии, которые могут немедленно остановить робота."""

    line_left: int = 1023
    line_right: int = 1023
    ir_cm: int = 60


@dataclass
class PlannerState:
    """Состояние пользовательской карты и рассчитанного маршрута."""

    obstacle_mask: np.ndarray
    frame_width: int
    frame_height: int
    goal_pixel: np.ndarray | None = None
    path_points: list[np.ndarray] = field(default_factory=list)
    waypoint_index: int = 0
    paused: bool = True
    drawing: bool = False
    erasing: bool = False
    map_changed: bool = True
    last_plan_time: float = 0.0
    status: str = "RIGHT CLICK: SET GOAL"


def parse_safety(line: str) -> SafetyData | None:
    """Из полной строки TEL выбирает датчики линии и дальномер."""
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


def receive_telemetry(
    connection: socket.socket,
    buffer: bytes,
    current: SafetyData,
) -> tuple[bytes, SafetyData, bool]:
    """Читает все доступные байты, сохраняя неполную строку в буфере."""
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
        line = raw_line.decode("ascii", errors="replace").strip()
        parsed = parse_safety(line)
        if parsed is not None:
            current = parsed
            received = True

    return buffer, current, received


def send(connection: socket.socket, command: str) -> None:
    """Отправляет роботу одну строковую команду."""
    try:
        connection.sendall((command + "\n").encode("ascii"))
    except OSError:
        pass


def marker_geometry(corners: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Возвращает центр маркера и единичный вектор направления P0–P1."""
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)

    front = 0.5 * (points[0] + points[1])
    heading = front - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)

    return center.astype(np.float32), heading.astype(np.float32)


def signed_angle(heading: np.ndarray, vector: np.ndarray) -> float:
    """Возвращает угол от направления робота к целевому вектору со знаком."""
    hx, hy = float(heading[0]), -float(heading[1])
    vx, vy = float(vector[0]), -float(vector[1])

    cross = hx * vy - hy * vx
    dot = hx * vx + hy * vy
    return math.atan2(cross, dot)


def grid_shape(width: int, height: int) -> tuple[int, int]:
    """Возвращает количество строк и столбцов сетки планирования."""
    rows = math.ceil(height / GRID_CELL_SIZE_PX)
    cols = math.ceil(width / GRID_CELL_SIZE_PX)
    return rows, cols


def pixel_to_cell(
    point: np.ndarray | tuple[int, int],
    rows: int,
    cols: int,
) -> Cell:
    """Переводит координату пикселя в клетку сетки."""
    x = int(point[0])
    y = int(point[1])
    row = max(0, min(rows - 1, y // GRID_CELL_SIZE_PX))
    col = max(0, min(cols - 1, x // GRID_CELL_SIZE_PX))
    return row, col


def cell_center(cell: Cell, width: int, height: int) -> np.ndarray:
    """Возвращает координату центра клетки в пикселях изображения."""
    row, col = cell
    x = min(width - 1, col * GRID_CELL_SIZE_PX + GRID_CELL_SIZE_PX / 2)
    y = min(height - 1, row * GRID_CELL_SIZE_PX + GRID_CELL_SIZE_PX / 2)
    return np.array([x, y], dtype=np.float32)


def inflate_obstacles(mask: np.ndarray) -> np.ndarray:
    """Расширяет нарисованные препятствия на безопасный радиус робота."""
    diameter = 2 * ROBOT_SAFETY_RADIUS_PX + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (diameter, diameter),
    )
    return cv2.dilate(mask, kernel)


def blocked_cells_from_mask(mask: np.ndarray) -> tuple[set[Cell], int, int]:
    """Преобразует пиксельную маску препятствий в множество занятых клеток."""
    height, width = mask.shape
    rows, cols = grid_shape(width, height)
    blocked: set[Cell] = set()

    for row in range(rows):
        y0 = row * GRID_CELL_SIZE_PX
        y1 = min(height, y0 + GRID_CELL_SIZE_PX)
        for col in range(cols):
            x0 = col * GRID_CELL_SIZE_PX
            x1 = min(width, x0 + GRID_CELL_SIZE_PX)
            if np.any(mask[y0:y1, x0:x1] > 0):
                blocked.add((row, col))

    return blocked, rows, cols


def neighbors(cell: Cell, rows: int, cols: int) -> list[tuple[Cell, float]]:
    """Возвращает восемь соседей и стоимость перехода к каждому."""
    row, col = cell
    result: list[tuple[Cell, float]] = []

    for delta_row in (-1, 0, 1):
        for delta_col in (-1, 0, 1):
            if delta_row == 0 and delta_col == 0:
                continue

            candidate = (row + delta_row, col + delta_col)
            if not (0 <= candidate[0] < rows and 0 <= candidate[1] < cols):
                continue

            cost = math.sqrt(2.0) if delta_row and delta_col else 1.0
            result.append((candidate, cost))

    return result


def heuristic(a: Cell, b: Cell) -> float:
    """Евклидова нижняя оценка оставшегося расстояния."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reconstruct_path(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(
    start: Cell,
    goal: Cell,
    blocked: set[Cell],
    rows: int,
    cols: int,
) -> list[Cell] | None:
    """Ищет путь по восьмисвязной сетке алгоритмом A*."""
    if goal in blocked:
        return None

    # Робот уже физически находится в стартовой клетке, поэтому старт разрешаем.
    blocked = set(blocked)
    blocked.discard(start)

    queue: list[tuple[float, float, Cell]] = []
    heapq.heappush(queue, (heuristic(start, goal), 0.0, start))

    came_from: dict[Cell, Cell] = {}
    best_cost: dict[Cell, float] = {start: 0.0}

    while queue:
        _priority, current_cost, current = heapq.heappop(queue)

        if current == goal:
            return reconstruct_path(came_from, current)

        if current_cost > best_cost.get(current, float("inf")):
            continue

        for candidate, step_cost in neighbors(current, rows, cols):
            if candidate in blocked:
                continue

            delta_row = candidate[0] - current[0]
            delta_col = candidate[1] - current[1]

            # Запрещаем срезать диагональ через угол двух занятых клеток.
            if delta_row and delta_col:
                side_a = (current[0] + delta_row, current[1])
                side_b = (current[0], current[1] + delta_col)
                if side_a in blocked or side_b in blocked:
                    continue

            new_cost = current_cost + step_cost
            if new_cost >= best_cost.get(candidate, float("inf")):
                continue

            best_cost[candidate] = new_cost
            came_from[candidate] = current
            priority = new_cost + heuristic(candidate, goal)
            heapq.heappush(queue, (priority, new_cost, candidate))

    return None


def line_is_free(
    point_a: np.ndarray,
    point_b: np.ndarray,
    obstacle_mask: np.ndarray,
) -> bool:
    """Проверяет, пересекает ли отрезок расширенную маску препятствий."""
    distance = max(float(np.linalg.norm(point_b - point_a)), 1.0)
    sample_count = max(2, int(distance / 3.0) + 1)

    xs = np.linspace(point_a[0], point_b[0], sample_count)
    ys = np.linspace(point_a[1], point_b[1], sample_count)

    height, width = obstacle_mask.shape
    xi = np.clip(np.rint(xs).astype(int), 0, width - 1)
    yi = np.clip(np.rint(ys).astype(int), 0, height - 1)

    return not bool(np.any(obstacle_mask[yi, xi] > 0))


def simplify_path(
    points: list[np.ndarray],
    obstacle_mask: np.ndarray,
) -> list[np.ndarray]:
    """Удаляет лишние точки, если между дальними точками есть прямая видимость."""
    if len(points) <= 2:
        return points

    simplified = [points[0]]
    current_index = 0

    while current_index < len(points) - 1:
        next_index = len(points) - 1
        while next_index > current_index + 1:
            if line_is_free(
                points[current_index],
                points[next_index],
                obstacle_mask,
            ):
                break
            next_index -= 1

        simplified.append(points[next_index])
        current_index = next_index

    return simplified


def plan_path(
    robot_pixel: np.ndarray,
    goal_pixel: np.ndarray,
    obstacle_mask: np.ndarray,
) -> tuple[list[np.ndarray] | None, np.ndarray]:
    """Строит безопасный маршрут от текущего положения робота к цели."""
    height, width = obstacle_mask.shape
    inflated = inflate_obstacles(obstacle_mask)

    # Оранжевая область является запасом безопасности для планировщика.
    # Если робот уже оказался внутри неё, необходимо разрешить ему выехать наружу.
    #
    # Для этого вокруг текущего положения временно очищается стартовая область,
    # но только там, где нет самой нарисованной красной стены. Реальное
    # препятствие в obstacle_mask никогда не удаляется.
    planning_mask = inflated.copy()
    robot_point = tuple(np.rint(robot_pixel).astype(int))

    escape_zone = np.zeros_like(planning_mask)
    cv2.circle(
        escape_zone,
        robot_point,
        START_ESCAPE_RADIUS_PX,
        255,
        -1,
    )

    may_clear = (escape_zone > 0) & (obstacle_mask == 0)
    planning_mask[may_clear] = 0

    blocked, rows, cols = blocked_cells_from_mask(planning_mask)
    start = pixel_to_cell(robot_pixel, rows, cols)
    goal = pixel_to_cell(goal_pixel, rows, cols)

    cells = astar(start, goal, blocked, rows, cols)
    if cells is None:
        return None, inflated

    points = [cell_center(cell, width, height) for cell in cells]
    points[0] = robot_pixel.astype(np.float32).copy()
    points[-1] = goal_pixel.astype(np.float32).copy()

    return simplify_path(points, planning_mask), inflated


def select_waypoint(state: PlannerState, robot_pixel: np.ndarray) -> np.ndarray | None:
    """Переключает пройденные промежуточные точки и возвращает текущую."""
    while state.waypoint_index < len(state.path_points):
        waypoint = state.path_points[state.waypoint_index]
        distance = float(np.linalg.norm(waypoint - robot_pixel))

        is_last = state.waypoint_index == len(state.path_points) - 1
        tolerance = GOAL_TOLERANCE_PX if is_last else WAYPOINT_TOLERANCE_PX

        if distance > tolerance:
            return waypoint
        state.waypoint_index += 1

    return None


def apply_brush(state: PlannerState, x: int, y: int) -> None:
    """Добавляет или стирает круглую область маски препятствий."""
    if not (0 <= x < state.frame_width and 0 <= y < state.frame_height):
        return

    if state.drawing:
        cv2.circle(
            state.obstacle_mask,
            (x, y),
            WALL_BRUSH_RADIUS_PX,
            255,
            -1,
        )
        state.map_changed = True

    if state.erasing:
        cv2.circle(
            state.obstacle_mask,
            (x, y),
            WALL_BRUSH_RADIUS_PX + 5,
            0,
            -1,
        )
        state.map_changed = True


def mouse_callback(
    event: int,
    x: int,
    y: int,
    flags: int,
    state: PlannerState,
) -> None:
    """Обрабатывает рисование стен и выбор цели поверх кадра камеры."""
    if y >= state.frame_height:
        return

    if event == cv2.EVENT_RBUTTONDOWN:
        state.goal_pixel = np.array([x, y], dtype=np.float32)
        state.path_points.clear()
        state.waypoint_index = 0
        state.map_changed = True
        state.paused = False
        state.status = "NEW GOAL"
        return

    if event == cv2.EVENT_MBUTTONDOWN:
        state.erasing = True
        state.drawing = False
        apply_brush(state, x, y)
        return

    if event == cv2.EVENT_MBUTTONUP:
        state.erasing = False
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        erase = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
        state.drawing = not erase
        state.erasing = erase
        apply_brush(state, x, y)
        return

    if event == cv2.EVENT_LBUTTONUP:
        state.drawing = False
        state.erasing = False
        return

    if event == cv2.EVENT_MOUSEMOVE:
        left_pressed = bool(flags & cv2.EVENT_FLAG_LBUTTON)
        middle_pressed = bool(flags & cv2.EVENT_FLAG_MBUTTON)

        if not left_pressed and not middle_pressed:
            state.drawing = False
            state.erasing = False
            return

        if middle_pressed:
            state.drawing = False
            state.erasing = True
        elif left_pressed:
            erase = bool(flags & cv2.EVENT_FLAG_SHIFTKEY)
            state.drawing = not erase
            state.erasing = erase

        apply_brush(state, x, y)


def overlay_mask(
    frame: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
    alpha: float,
) -> None:
    """Полупрозрачно окрашивает выбранные пиксели кадра."""
    selected = mask > 0
    if not np.any(selected):
        return

    color_layer = np.empty_like(frame)
    color_layer[:] = color
    blended = cv2.addWeighted(frame, 1.0 - alpha, color_layer, alpha, 0)
    frame[selected] = blended[selected]


def render_display(
    frame: np.ndarray,
    state: PlannerState,
    inflated_mask: np.ndarray,
    robot_center: np.ndarray | None,
    robot_heading: np.ndarray | None,
    current_waypoint: np.ndarray | None,
    command: str,
    safety: SafetyData,
) -> np.ndarray:
    """Рисует препятствия, маршрут и информационную панель."""
    display = frame.copy()

    # Сначала безопасная зона, затем сами препятствия.
    safety_only = cv2.subtract(inflated_mask, state.obstacle_mask)
    overlay_mask(display, safety_only, (0, 170, 255), 0.20)
    overlay_mask(display, state.obstacle_mask, (40, 40, 230), 0.48)

    if len(state.path_points) >= 2:
        points = np.rint(np.array(state.path_points)).astype(np.int32)
        cv2.polylines(display, [points], False, (255, 170, 0), 3)
        for point in points[1:-1]:
            cv2.circle(display, tuple(point), 5, (255, 170, 0), -1)

    if state.goal_pixel is not None:
        goal = tuple(np.rint(state.goal_pixel).astype(int))
        cv2.circle(display, goal, GOAL_TOLERANCE_PX, (60, 210, 60), 2)
        cv2.circle(display, goal, 7, (60, 210, 60), -1)

    if current_waypoint is not None:
        waypoint = tuple(np.rint(current_waypoint).astype(int))
        cv2.circle(display, waypoint, 10, (0, 230, 255), 2)

    if robot_center is not None and robot_heading is not None:
        center = tuple(np.rint(robot_center).astype(int))
        front = tuple(
            np.rint(robot_center + robot_heading * 55).astype(int)
        )
        cv2.circle(display, center, 5, (255, 255, 255), -1)
        cv2.arrowedLine(display, center, front, (255, 255, 255), 3)

    panel = np.full(
        (INFO_PANEL_HEIGHT, state.frame_width, 3),
        (24, 24, 24),
        dtype=np.uint8,
    )

    cv2.putText(
        panel,
        f"STATE: {state.status}",
        (16, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        (90, 230, 90) if command != "STOP" else (80, 150, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"COMMAND: {command}    LINE: {safety.line_left}/{safety.line_right}"
        f"    IR: {safety.ir_cm} cm",
        (16, 63),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "LMB draw | Shift+LMB/MMB erase | RMB goal | Space pause",
        (16, 98),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.49,
        (210, 210, 210),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "C clear obstacles | R clear goal | Esc stop",
        (16, 128),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.49,
        (210, 210, 210),
        1,
        cv2.LINE_AA,
    )

    return np.vstack((display, panel))


def build_demo_image(output_path: Path) -> None:
    """Создаёт пример кадра для README без подключения камеры и робота."""
    width, height = 960, 540
    frame = np.full((height, width, 3), (180, 188, 190), dtype=np.uint8)

    # Условный фон полигона.
    for y in range(0, height, 45):
        cv2.line(frame, (0, y), (width, y), (165, 172, 175), 1)
    for x in range(0, width, 60):
        cv2.line(frame, (x, 0), (x, height), (170, 177, 180), 1)

    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.line(mask, (350, 40), (350, 335), 255, 34)
    cv2.line(mask, (350, 335), (690, 335), 255, 34)
    cv2.line(mask, (650, 150), (900, 150), 255, 34)

    robot = np.array([120.0, 430.0], dtype=np.float32)
    goal = np.array([850.0, 70.0], dtype=np.float32)
    path, inflated = plan_path(robot, goal, mask)

    state = PlannerState(mask, width, height)
    state.goal_pixel = goal
    state.path_points = path or []
    state.status = "DRIVE TO WAYPOINT"

    display = render_display(
        frame,
        state,
        inflated,
        robot,
        np.array([1.0, -0.2], dtype=np.float32),
        path[1] if path and len(path) > 1 else None,
        "VEL 180 0",
        SafetyData(),
    )

    # Условный маркер робота.
    robot_point = tuple(np.rint(robot).astype(int))
    cv2.rectangle(
        display,
        (robot_point[0] - 24, robot_point[1] - 24),
        (robot_point[0] + 24, robot_point[1] + 24),
        (20, 20, 20),
        3,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), display):
        raise RuntimeError(f"Не удалось сохранить {output_path}")


def self_test() -> None:
    """Проверяет поиск пути и отсутствие пересечений с препятствиями."""
    width, height = 640, 480
    mask = np.zeros((height, width), dtype=np.uint8)

    # Вертикальная стена с достаточно широким проходом.
    cv2.rectangle(mask, (300, 0), (340, 155), 255, -1)
    cv2.rectangle(mask, (300, 325), (340, height - 1), 255, -1)

    start = np.array([80.0, 240.0], dtype=np.float32)
    goal = np.array([560.0, 240.0], dtype=np.float32)

    path, inflated = plan_path(start, goal, mask)
    assert path is not None, "Через проход должен существовать маршрут"
    assert len(path) >= 2

    planning_mask = inflated.copy()
    cv2.circle(
        planning_mask,
        tuple(np.rint(start).astype(int)),
        GRID_CELL_SIZE_PX,
        0,
        -1,
    )

    for point_a, point_b in zip(path, path[1:]):
        assert line_is_free(point_a, point_b, planning_mask)

    # Полная стена от верхнего до нижнего края должна перекрыть путь.
    closed_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(closed_mask, (300, 0), (340, height - 1), 255, -1)
    no_path, _ = plan_path(start, goal, closed_mask)
    assert no_path is None, "Через сплошную стену маршрута быть не должно"

    heading = np.array([1.0, 0.0], dtype=np.float32)
    assert signed_angle(heading, np.array([1.0, -1.0])) > 0
    assert signed_angle(heading, np.array([1.0, 1.0])) < 0

    print(f"SELF-TEST OK: simplified path points = {len(path)}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--demo-image", type=Path)
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return

    if args.demo_image:
        build_demo_image(args.demo_image)
        print(args.demo_image)
        return

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

    ok, first_frame = camera.read()
    if not ok:
        camera.release()
        raise RuntimeError("Не удалось получить первый кадр")

    height, width = first_frame.shape[:2]
    state = PlannerState(
        obstacle_mask=np.zeros((height, width), dtype=np.uint8),
        frame_width=width,
        frame_height=height,
    )

    connection = socket.create_connection(ROBOT_ADDRESS, timeout=3.0)
    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connection.setblocking(False)
    send(connection, "GET")

    window = "Camera path planner"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, mouse_callback, state)

    telemetry_buffer = b""
    safety = SafetyData()
    last_telemetry_time = 0.0
    previous_send_time = 0.0
    use_first_frame = True

    try:
        while True:
            if use_first_frame:
                frame = first_frame
                use_first_frame = False
            else:
                ok, frame = camera.read()
                if not ok:
                    break

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
            robot_center: np.ndarray | None = None
            robot_heading: np.ndarray | None = None
            current_waypoint: np.ndarray | None = None
            inflated_mask = inflate_obstacles(state.obstacle_mask)
            now = time.monotonic()

            telemetry_stale = (
                last_telemetry_time == 0.0
                or now - last_telemetry_time > TELEMETRY_TIMEOUT_SECONDS
            )
            black_border = (
                safety.line_left < LEFT_LINE_THRESHOLD
                or safety.line_right < RIGHT_LINE_THRESHOLD
            )
            physical_obstacle = safety.ir_cm <= OBSTACLE_STOP_CM

            if telemetry_stale:
                state.status = "WAIT FOR TELEMETRY"

            elif black_border:
                state.status = "BLACK BORDER: STOP"
                state.paused = True

            elif physical_obstacle:
                state.status = f"IR OBSTACLE {safety.ir_cm} CM: STOP"

            elif state.paused:
                state.status = "PAUSED"

            elif state.goal_pixel is None:
                state.status = "RIGHT CLICK: SET GOAL"

            elif ROBOT_MARKER_ID not in markers:
                state.status = f"WAIT FOR ARUCO ID {ROBOT_MARKER_ID}"

            else:
                robot_center, robot_heading = marker_geometry(
                    markers[ROBOT_MARKER_ID]
                )

                robot_x = int(round(float(robot_center[0])))
                robot_y = int(round(float(robot_center[1])))
                robot_x = max(0, min(width - 1, robot_x))
                robot_y = max(0, min(height - 1, robot_y))

                inside_drawn_wall = (
                    state.obstacle_mask[robot_y, robot_x] > 0
                )
                inside_safety_zone = (
                    inflated_mask[robot_y, robot_x] > 0
                )

                # Красная зона — само виртуальное препятствие. Если центр робота
                # находится в ней, движение запрещается.
                #
                # Оранжевая зона — только запас безопасности для планировщика.
                # Нахождение центра робота в ней больше не вызывает STOP:
                # планировщик строит маршрут наружу и продолжает движение.
                if inside_drawn_wall:
                    state.status = "ROBOT INSIDE VIRTUAL WALL"
                    state.map_changed = True

                else:
                    need_replan = (
                        state.map_changed
                        or state.last_plan_time == 0.0
                        or now - state.last_plan_time >= REPLAN_PERIOD_SECONDS
                    )

                    if need_replan:
                        path, inflated_mask = plan_path(
                            robot_center,
                            state.goal_pixel,
                            state.obstacle_mask,
                        )
                        state.last_plan_time = now
                        state.map_changed = False

                        if path is None:
                            state.path_points.clear()
                            state.waypoint_index = 0
                            state.status = "NO SAFE PATH"
                        else:
                            state.path_points = path
                            state.waypoint_index = 1 if len(path) > 1 else 0

                    goal_distance = float(
                        np.linalg.norm(state.goal_pixel - robot_center)
                    )

                    if goal_distance <= GOAL_TOLERANCE_PX:
                        state.status = "GOAL REACHED"
                        state.paused = True

                    elif state.path_points:
                        current_waypoint = select_waypoint(
                            state,
                            robot_center,
                        )

                        if current_waypoint is None:
                            state.status = "REPLAN"
                            state.map_changed = True
                        else:
                            vector = current_waypoint - robot_center
                            angle_error = signed_angle(
                                robot_heading,
                                vector,
                            )

                            if abs(angle_error) > ANGLE_TOLERANCE_RAD:
                                angular = (
                                    ANGULAR_SPEED_MRAD_S
                                    if angle_error > 0
                                    else -ANGULAR_SPEED_MRAD_S
                                )
                                command = f"VEL 0 {angular}"
                                state.status = (
                                    "TURN LEFT"
                                    if angle_error > 0
                                    else "TURN RIGHT"
                                )
                            else:
                                command = f"VEL {LINEAR_SPEED_MM_S} 0"
                                state.status = "DRIVE TO WAYPOINT"

                    # Сообщение носит информационный характер: робот продолжает
                    # выполнять найденный маршрут и выезжает из оранжевой зоны.
                    if inside_safety_zone and command != "STOP":
                        state.status = (
                            "LEAVING SAFETY ZONE | " + state.status
                        )

            display = render_display(
                frame,
                state,
                inflated_mask,
                robot_center,
                robot_heading,
                current_waypoint,
                command,
                safety,
            )

            if now - previous_send_time >= SEND_PERIOD_SECONDS:
                send(connection, command)
                previous_send_time = now

            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            if key == ord(" "):
                state.paused = not state.paused
                if state.paused:
                    send(connection, "STOP")
                else:
                    state.map_changed = True

            if key in (ord("c"), ord("C")):
                state.obstacle_mask.fill(0)
                state.path_points.clear()
                state.waypoint_index = 0
                state.map_changed = True

            if key in (ord("r"), ord("R")):
                send(connection, "STOP")
                state.goal_pixel = None
                state.path_points.clear()
                state.waypoint_index = 0
                state.paused = True
                state.map_changed = True
                state.status = "RIGHT CLICK: SET GOAL"

    finally:
        send(connection, "STOP")
        connection.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Пульт дня 2: управление Arduino по USB Serial и карта одометрии."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:
    raise SystemExit(
        "Не установлен пакет pyserial.\n"
        "Установите его командой:\n"
        "python -m pip install pyserial"
    ) from error


BAUD_RATE = 115200
SEND_PERIOD_MS = 50
LINEAR_SPEED_MM_S = 350
ANGULAR_SPEED_MRAD_S = 5000
MAP_SCALE = 0.45


@dataclass
class Telemetry:
    x_mm: int = 0
    y_mm: int = 0
    theta_mrad: int = 0
    linear_mm_s: int = 0
    angular_mrad_s: int = 0
    encoder_left: int = 0
    encoder_right: int = 0
    line_left: int = 0
    line_right: int = 0
    ir_cm: int = 60
    servo_deg: int = 90


def parse_telemetry(line: str) -> Telemetry | None:
    parts = line.split()
    if len(parts) != 12 or parts[0] != "TEL":
        return None
    try:
        return Telemetry(*map(int, parts[1:]))
    except ValueError:
        return None


def port_label(port: object) -> str:
    device = getattr(port, "device", "")
    description = getattr(port, "description", "")
    return f"{device} — {description}" if description else device


def choose_serial_port(root: tk.Tk, requested_port: str | None) -> str:
    if requested_port:
        return requested_port

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError(
            "Последовательные порты не найдены.\n"
            "Подключите Arduino по USB и запустите программу снова."
        )
    if len(ports) == 1:
        return ports[0].device

    selected: dict[str, str | None] = {"port": None}
    dialog = tk.Toplevel(root)
    dialog.title("Выбор Arduino")
    dialog.resizable(False, False)
    dialog.transient(root)
    dialog.grab_set()

    frame = ttk.Frame(dialog, padding=14)
    frame.grid(sticky="nsew")
    ttk.Label(
        frame,
        text="Выберите последовательный порт Arduino:",
    ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

    labels = [port_label(port) for port in ports]
    combo = ttk.Combobox(
        frame,
        values=labels,
        state="readonly",
        width=52,
    )
    combo.current(0)
    combo.grid(row=1, column=0, columnspan=2, sticky="ew")

    def connect() -> None:
        index = combo.current()
        if index >= 0:
            selected["port"] = ports[index].device
            dialog.destroy()

    def cancel() -> None:
        dialog.destroy()

    ttk.Button(frame, text="Подключиться", command=connect).grid(
        row=2, column=0, sticky="ew", padx=(0, 4), pady=(12, 0)
    )
    ttk.Button(frame, text="Отмена", command=cancel).grid(
        row=2, column=1, sticky="ew", padx=(4, 0), pady=(12, 0)
    )
    dialog.protocol("WM_DELETE_WINDOW", cancel)
    dialog.wait_window()

    if selected["port"] is None:
        raise RuntimeError("Подключение отменено.")
    return selected["port"]


class OdometryMap:
    def __init__(
        self,
        root: tk.Tk,
        serial_port: str,
    ) -> None:
        self.root = root
        self.serial_port = serial_port
        self.root.title(
            f"NEYMARK Robot — карта одометрии — UART {serial_port}"
        )
        self.root.geometry("1100x720")

        self.serial = serial.Serial(
            port=serial_port,
            baudrate=BAUD_RATE,
            timeout=0,
            write_timeout=0.5,
        )
        # Arduino Nano/Uno обычно перезапускается при открытии USB-порта.
        # Небольшая пауза позволяет загрузчику закончить работу.
        time.sleep(1.8)
        self.serial.reset_input_buffer()

        self.receive_buffer = b""
        self.keys: set[str] = set()
        self.telemetry = Telemetry()
        self.last_motion = (0, 0)
        self.trajectory: list[tuple[int, int]] = [(0, 0)]
        self.last_sent_servo_angle = 90

        self.pose_var = tk.StringVar(value="X=0  Y=0  θ=0°")
        self.motion_var = tk.StringVar(value="Команда: VEL 0 0")
        self.speed_var = tk.StringVar(
            value="Измерено: v=0 мм/с  ω=0 мрад/с"
        )
        self.encoder_var = tk.StringVar(value="Энкодеры: L=0  R=0")
        self.sensor_var = tk.StringVar(
            value="Линия 0/0   ИК 60 см   Серва 90°"
        )
        self.status_var = tk.StringVar(
            value=(
                f"Подключено: {serial_port}, {BAUD_RATE} бод. "
                "Управление: WASD или стрелки."
            )
        )

        self._build()
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(20, self._receive_loop)
        self.root.after(SEND_PERIOD_MS, self._send_motion_loop)
        self.root.after(100, lambda: self.send("GET"))
        self._draw_map()

    def _build(self) -> None:
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        panel = ttk.Frame(self.root, padding=10)
        panel.grid(row=0, column=0, sticky="ns")
        ttk.Label(
            panel,
            text="Ручное управление по UART",
            font=("TkDefaultFont", 13, "bold"),
        ).grid(row=0, column=0, columnspan=3, pady=(0, 8))

        button_data = (
            ("↖", {"forward", "left"}, 1, 0),
            ("↑", {"forward"}, 1, 1),
            ("↗", {"forward", "right"}, 1, 2),
            ("←", {"left"}, 2, 0),
            ("STOP", set(), 2, 1),
            ("→", {"right"}, 2, 2),
            ("↙", {"back", "left"}, 3, 0),
            ("↓", {"back"}, 3, 1),
            ("↘", {"back", "right"}, 3, 2),
        )
        for text, keys, row, column in button_data:
            button = tk.Button(panel, text=text, width=7, height=2)
            button.grid(row=row, column=column, padx=2, pady=2)
            button.bind(
                "<ButtonPress-1>",
                lambda _event, selected=keys: self._button_motion(selected),
            )
            button.bind(
                "<ButtonRelease-1>",
                lambda _event: self._button_motion(set()),
            )

        ttk.Separator(panel).grid(
            row=4, column=0, columnspan=3, sticky="ew", pady=10
        )
        for row, variable in enumerate(
            (
                self.motion_var,
                self.speed_var,
                self.encoder_var,
                self.pose_var,
                self.sensor_var,
                self.status_var,
            ),
            start=5,
        ):
            ttk.Label(
                panel,
                textvariable=variable,
                wraplength=280,
            ).grid(row=row, column=0, columnspan=3, sticky="w", pady=3)

        ttk.Label(panel, text="Сервопривод").grid(
            row=11, column=0, columnspan=3, sticky="w", pady=(12, 0)
        )
        servo = ttk.Scale(
            panel,
            from_=20,
            to=160,
            orient="horizontal",
            command=self._servo_changed,
        )
        servo.set(90)
        servo.grid(row=12, column=0, columnspan=3, sticky="ew")

        ttk.Button(
            panel,
            text="Сбросить положение в (0, 0, 0)",
            command=self._reset_odometry,
        ).grid(row=13, column=0, columnspan=3, sticky="ew", pady=(12, 3))
        ttk.Button(
            panel,
            text="Очистить только след на карте",
            command=self._clear_path,
        ).grid(row=14, column=0, columnspan=3, sticky="ew", pady=3)

        self.canvas = tk.Canvas(
            self.root,
            bg="white",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        self.canvas.bind("<Configure>", lambda _event: self._draw_map())

    def send(self, command: str) -> None:
        try:
            self.serial.write((command + "\n").encode("ascii"))
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            self.status_var.set("Последовательное соединение потеряно")

    def _servo_changed(self, value: str) -> None:
        angle = round(float(value))
        if angle != self.last_sent_servo_angle:
            self.last_sent_servo_angle = angle
            self.send(f"SERVO {angle}")

    def _button_motion(self, keys: set[str]) -> None:
        self.keys = set(keys)
        if not keys:
            self.send("STOP")

    def _key_press(self, event: tk.Event) -> None:
        mapping = {
            "w": "forward",
            "up": "forward",
            "s": "back",
            "down": "back",
            "a": "left",
            "left": "left",
            "d": "right",
            "right": "right",
        }
        key = event.keysym.lower()
        if key in mapping:
            self.keys.add(mapping[key])
        elif key == "space":
            self.keys.clear()
            self.send("STOP")

    def _key_release(self, event: tk.Event) -> None:
        mapping = {
            "w": "forward",
            "up": "forward",
            "s": "back",
            "down": "back",
            "a": "left",
            "left": "left",
            "d": "right",
            "right": "right",
        }
        key = event.keysym.lower()
        if key in mapping:
            self.keys.discard(mapping[key])

    def _motion(self) -> tuple[int, int]:
        linear = 0
        angular = 0
        if "forward" in self.keys:
            linear += LINEAR_SPEED_MM_S
        if "back" in self.keys:
            linear -= LINEAR_SPEED_MM_S
        if "left" in self.keys:
            angular += ANGULAR_SPEED_MRAD_S
        if "right" in self.keys:
            angular -= ANGULAR_SPEED_MRAD_S
        return linear, angular

    def _send_motion_loop(self) -> None:
        motion = self._motion()
        if motion != (0, 0):
            self.send(f"VEL {motion[0]} {motion[1]}")
        elif self.last_motion != (0, 0):
            self.send("STOP")
        self.last_motion = motion
        self.motion_var.set(f"Команда: VEL {motion[0]} {motion[1]}")
        self.root.after(SEND_PERIOD_MS, self._send_motion_loop)

    def _receive_loop(self) -> None:
        try:
            waiting = self.serial.in_waiting
            if waiting:
                self.receive_buffer += self.serial.read(waiting)
        except (serial.SerialException, OSError):
            self.status_var.set("Последовательное соединение потеряно")
            self.root.after(100, self._receive_loop)
            return

        while b"\n" in self.receive_buffer:
            raw_line, self.receive_buffer = self.receive_buffer.split(
                b"\n", 1
            )
            line = raw_line.decode("utf-8", errors="replace").strip()
            telemetry = parse_telemetry(line)
            if telemetry is None:
                if line:
                    self.status_var.set(line)
                continue

            self.telemetry = telemetry
            point = (telemetry.x_mm, telemetry.y_mm)
            previous = self.trajectory[-1]
            if math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            ) >= 5:
                self.trajectory.append(point)
            self._show_telemetry()
            self._draw_map()

        self.root.after(20, self._receive_loop)

    def _show_telemetry(self) -> None:
        data = self.telemetry
        self.pose_var.set(
            f"X={data.x_mm}  Y={data.y_mm}  "
            f"θ={math.degrees(data.theta_mrad / 1000):.1f}°"
        )
        self.speed_var.set(
            f"Измерено: v={data.linear_mm_s} мм/с  "
            f"ω={data.angular_mrad_s} мрад/с"
        )
        self.encoder_var.set(
            f"Энкодеры: L={data.encoder_left}  R={data.encoder_right}"
        )
        self.sensor_var.set(
            f"Линия {data.line_left}/{data.line_right}   "
            f"ИК {data.ir_cm} см   Серва {data.servo_deg}°"
        )

    def _reset_odometry(self) -> None:
        self.keys.clear()
        self.send("RESET_ODOM")
        self.telemetry = Telemetry()
        self.trajectory = [(0, 0)]
        self._draw_map()

    def _clear_path(self) -> None:
        self.trajectory = [
            (self.telemetry.x_mm, self.telemetry.y_mm)
        ]
        self._draw_map()

    def _draw_map(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 400)
        height = max(canvas.winfo_height(), 400)

        points = self.trajectory + [
            (self.telemetry.x_mm, self.telemetry.y_mm),
            (0, 0),
        ]
        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        margin_mm = 180
        world_width = max(max_x - min_x + 2 * margin_mm, 1)
        world_height = max(max_y - min_y + 2 * margin_mm, 1)
        scale = min(
            MAP_SCALE,
            max(width - 40, 1) / world_width,
            max(height - 40, 1) / world_height,
        )
        view_center_x = 0.5 * (min_x + max_x)
        view_center_y = 0.5 * (min_y + max_y)
        origin_x = width / 2 - view_center_x * scale
        origin_y = height / 2 + view_center_y * scale

        grid_step_mm = 100 if 100 * scale >= 28 else 500
        grid_px = grid_step_mm * scale
        x = origin_x % grid_px
        while x < width:
            canvas.create_line(x, 0, x, height, fill="#edf0f4")
            x += grid_px
        y = origin_y % grid_px
        while y < height:
            canvas.create_line(0, y, width, y, fill="#edf0f4")
            y += grid_px

        canvas.create_line(
            0, origin_y, width, origin_y, fill="#798493", arrow=tk.LAST
        )
        canvas.create_line(
            origin_x, height, origin_x, 0, fill="#798493", arrow=tk.LAST
        )

        pixels: list[float] = []
        for x_mm, y_mm in self.trajectory:
            pixels.extend(
                (
                    origin_x + x_mm * scale,
                    origin_y - y_mm * scale,
                )
            )
        if len(pixels) >= 4:
            canvas.create_line(*pixels, fill="#f28c18", width=3)

        robot_x = origin_x + self.telemetry.x_mm * scale
        robot_y = origin_y - self.telemetry.y_mm * scale
        radius = 22
        theta = self.telemetry.theta_mrad / 1000
        canvas.create_oval(
            robot_x - radius,
            robot_y - radius,
            robot_x + radius,
            robot_y + radius,
            fill="#2878d0",
            outline="#114f8a",
            width=2,
        )
        canvas.create_line(
            robot_x,
            robot_y,
            robot_x + 34 * math.cos(theta),
            robot_y - 34 * math.sin(theta),
            fill="white",
            width=4,
            arrow=tk.LAST,
        )

    def _close(self) -> None:
        try:
            self.keys.clear()
            self.send("STOP")
        finally:
            if self.serial.is_open:
                self.serial.close()
            self.root.destroy()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Управление Arduino и карта одометрии по USB Serial."
    )
    parser.add_argument(
        "--port",
        help="Последовательный порт, например COM5 или /dev/ttyUSB0.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    root = tk.Tk()
    root.withdraw()

    try:
        selected_port = choose_serial_port(root, args.port)
        app = OdometryMap(root, selected_port)
    except (
        RuntimeError,
        serial.SerialException,
        serial.SerialTimeoutException,
        OSError,
    ) as error:
        messagebox.showerror("Ошибка подключения", str(error), parent=root)
        root.destroy()
        return

    root.deiconify()
    root.mainloop()


if __name__ == "__main__":
    main()

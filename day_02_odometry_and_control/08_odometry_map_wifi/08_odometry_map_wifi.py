#!/usr/bin/env python3
"""Финальный пульт дня 2: ручное управление и карта одометрии."""

from __future__ import annotations

from dataclasses import dataclass
import math
import socket
import tkinter as tk
from tkinter import ttk


ROBOT_ADDRESS = ("192.168.4.1", 8888)
SEND_PERIOD_MS = 50
LINEAR_SPEED_MM_S = 320
ANGULAR_SPEED_MRAD_S = 4500
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


class OdometryMap:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("NEYMARK Robot — карта одометрии")
        self.root.geometry("1100x720")
        self.socket = socket.create_connection(
            ROBOT_ADDRESS, timeout=3.0
        )
        self.socket.setsockopt(
            socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
        )
        self.socket.setblocking(False)
        self.receive_buffer = b""
        self.keys: set[str] = set()
        self.telemetry = Telemetry()
        self.last_motion = (0, 0)
        self.trajectory: list[tuple[int, int]] = [(0, 0)]
        self.last_sent_servo_angle = 90

        self.pose_var = tk.StringVar(value="X=0  Y=0  θ=0°")
        self.motion_var = tk.StringVar(value="VEL 0 0")
        self.sensor_var = tk.StringVar(
            value="Линия 0/0   ИК 60 см   Серва 90°"
        )
        self.status_var = tk.StringVar(
            value="Подключитесь к NEYMARK_01. Управление: WASD."
        )

        self._build()
        self.send("GET")
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(20, self._receive_loop)
        self.root.after(SEND_PERIOD_MS, self._send_motion_loop)
        self._draw_map()

    def _build(self) -> None:
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)

        panel = ttk.Frame(self.root, padding=10)
        panel.grid(row=0, column=0, sticky="ns")
        ttk.Label(
            panel,
            text="Ручное управление",
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
                self.pose_var,
                self.sensor_var,
                self.status_var,
            ),
            start=5,
        ):
            ttk.Label(
                panel,
                textvariable=variable,
                wraplength=260,
            ).grid(row=row, column=0, columnspan=3, sticky="w", pady=3)

        ttk.Label(panel, text="Сервопривод").grid(
            row=9, column=0, columnspan=3, sticky="w", pady=(12, 0)
        )
        servo = ttk.Scale(
            panel,
            from_=20,
            to=160,
            orient="horizontal",
            command=self._servo_changed,
        )
        servo.set(90)
        servo.grid(row=10, column=0, columnspan=3, sticky="ew")

        ttk.Button(
            panel,
            text="Сбросить положение в (0, 0, 0)",
            command=self._reset_odometry,
        ).grid(row=11, column=0, columnspan=3, sticky="ew", pady=(12, 3))
        ttk.Button(
            panel,
            text="Очистить только след на карте",
            command=self._clear_path,
        ).grid(row=12, column=0, columnspan=3, sticky="ew", pady=3)

        self.canvas = tk.Canvas(
            self.root,
            bg="white",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=(0, 10), pady=10)
        self.canvas.bind("<Configure>", lambda _event: self._draw_map())

    def send(self, command: str) -> None:
        try:
            self.socket.sendall((command + "\n").encode("ascii"))
        except OSError:
            self.status_var.set("TCP-соединение потеряно")

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
        self.motion_var.set(f"VEL {motion[0]} {motion[1]}")
        self.root.after(SEND_PERIOD_MS, self._send_motion_loop)

    def _receive_loop(self) -> None:
        while True:
            try:
                packet = self.socket.recv(2048)
            except BlockingIOError:
                break
            if not packet:
                self.status_var.set("TCP-соединение закрыто")
                break
            self.receive_buffer += packet

        while b"\n" in self.receive_buffer:
            raw_line, self.receive_buffer = self.receive_buffer.split(
                b"\n", 1
            )
            line = raw_line.decode(
                "utf-8", errors="replace"
            ).strip()
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
            self.send("STOP")
        finally:
            self.socket.close()
            self.root.destroy()


def main() -> None:
    root = tk.Tk()
    OdometryMap(root)
    root.mainloop()


if __name__ == "__main__":
    main()

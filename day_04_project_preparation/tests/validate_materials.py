#!/usr/bin/env python3
"""Локальная проверка структуры, синтаксиса и основных алгоритмов дня 4."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Не удалось загрузить {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def check_python_syntax() -> None:
    for path in ROOT.rglob("*.py"):
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        print("SYNTAX OK:", path.relative_to(ROOT))


def check_local_markdown_links() -> None:
    pattern = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    for readme in ROOT.rglob("*.md"):
        if readme.name == "README_ROOT_UPDATE.md":
            continue
        text = readme.read_text(encoding="utf-8")
        for target in pattern.findall(text):
            if target.startswith(("http://", "https://", "#", "../")):
                continue
            target = target.split("#", 1)[0]
            if not target:
                continue
            resolved = (readme.parent / target).resolve()
            # Ссылки на дни 2/3 проверяются только после интеграции папки в репозиторий.
            if "day_03_python_and_computer_vision" in target:
                continue
            if not resolved.exists():
                raise AssertionError(
                    f"Битая ссылка в {readme.relative_to(ROOT)}: {target}"
                )
    print("MARKDOWN LINKS OK")


def check_route_math() -> None:
    route = load_module(
        ROOT / "02_marker_route_mission" / "02_marker_route_mission.py",
        "route_module",
    )
    import numpy as np
    heading = np.array([1.0, 0.0])
    left = np.array([1.0, -1.0])  # вверх изображения = влево в математике
    right = np.array([1.0, 1.0])
    assert route.signed_angle(heading, left) > 0
    assert route.signed_angle(heading, right) < 0
    print("ROUTE MATH OK")


def check_gesture_mapping_without_mediapipe_import() -> None:
    path = ROOT / "03_gesture_control_mediapipe" / "03_gesture_control_mediapipe.py"
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "classify_gesture"
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace: dict[str, object] = {}
    exec(compile(module, str(path), "exec"), namespace)
    classify = namespace["classify_gesture"]
    assert classify((1, 0, 0, 0)) == "LEFT"
    assert classify((0, 0, 0, 1)) == "RIGHT"
    assert classify((1, 1, 1, 1)) == "FORWARD"
    assert classify((1, 0, 0, 1)) == "BACKWARD"
    assert classify((0, 1, 0, 0)) == "STOP"
    print("GESTURE MAPPING OK")


def check_camera_planner() -> None:
    script = (
        ROOT
        / "04_camera_virtual_obstacles"
        / "04_camera_virtual_obstacles.py"
    )
    subprocess.run(
        [sys.executable, str(script), "--self-test"],
        check=True,
    )


def main() -> None:
    check_python_syntax()
    check_local_markdown_links()
    check_route_math()
    check_gesture_mapping_without_mediapipe_import()
    check_camera_planner()
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()

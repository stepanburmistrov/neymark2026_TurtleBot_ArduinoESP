#!/usr/bin/env python3
"""Загружает официальную модель MediaPipe Hand Landmarker."""

from pathlib import Path
import urllib.request


MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
OUTPUT = Path(__file__).with_name("hand_landmarker.task")


def main() -> None:
    print("Загрузка модели MediaPipe...")
    try:
        urllib.request.urlretrieve(MODEL_URL, OUTPUT)
    except Exception as exc:
        if OUTPUT.exists():
            OUTPUT.unlink()
        raise SystemExit(f"Не удалось загрузить модель: {exc}") from exc

    size_mb = OUTPUT.stat().st_size / (1024 * 1024)
    print(f"Готово: {OUTPUT} ({size_mb:.1f} МБ)")


if __name__ == "__main__":
    main()

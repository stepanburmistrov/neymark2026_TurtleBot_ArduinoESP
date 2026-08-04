#!/usr/bin/env python3
"""Читает PNG 3×3 пикселя и печатает массив BGR."""

from pathlib import Path

import cv2


IMAGE_PATH = Path(__file__).with_name("sample_3x3.png")


def main() -> None:
    image = cv2.imread(str(IMAGE_PATH), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Не удалось прочитать {IMAGE_PATH}")

    print("Размер shape:", image.shape)
    print("Тип данных dtype:", image.dtype)
    print("Массив изображения в порядке BGR:")
    print(image)

    print("\nОтдельные пиксели:")
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):
            blue, green, red = map(int, image[y, x])
            print(f"pixel[y={y}, x={x}] = B:{blue:3d} G:{green:3d} R:{red:3d}")


if __name__ == "__main__":
    main()

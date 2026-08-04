#!/usr/bin/env python3
"""Получает кадры с камеры и выводит их на экран."""

import cv2


CAMERA_INDEX = 0


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась. Проверьте CAMERA_INDEX.")

    print("Для выхода нажмите Esc в окне изображения.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Не удалось получить кадр с камеры.")

            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

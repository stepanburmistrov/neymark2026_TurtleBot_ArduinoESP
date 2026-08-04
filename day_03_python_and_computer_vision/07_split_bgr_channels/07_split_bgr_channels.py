#!/usr/bin/env python3
"""Разделяет кадр камеры на каналы Blue, Green и Red."""

import cv2


CAMERA_INDEX = 0


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            blue, green, red = cv2.split(frame)

            cv2.imshow("Original BGR", frame)
            cv2.imshow("Blue channel", blue)
            cv2.imshow("Green channel", green)
            cv2.imshow("Red channel", red)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

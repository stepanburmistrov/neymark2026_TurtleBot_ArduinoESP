#!/usr/bin/env python3
"""Считает сумму яркостей пикселей в каналах B, G и R."""

import cv2
import numpy as np


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
            sums = {
                "BLUE": int(np.sum(blue, dtype=np.uint64)),
                "GREEN": int(np.sum(green, dtype=np.uint64)),
                "RED": int(np.sum(red, dtype=np.uint64)),
            }
            dominant = max(sums, key=sums.get)

            cv2.rectangle(frame, (10, 10), (510, 145), (20, 20, 20), -1)
            cv2.putText(
                frame,
                f"BLUE  sum = {sums['BLUE']}",
                (25, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 180, 80),
                2,
            )
            cv2.putText(
                frame,
                f"GREEN sum = {sums['GREEN']}",
                (25, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (80, 255, 80),
                2,
            )
            cv2.putText(
                frame,
                f"RED   sum = {sums['RED']}",
                (25, 115),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (80, 80, 255),
                2,
            )
            cv2.putText(
                frame,
                f"DOMINANT: {dominant}",
                (25, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                1,
            )

            cv2.imshow("Channel sums", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

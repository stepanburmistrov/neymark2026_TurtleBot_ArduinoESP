#!/usr/bin/env python3
"""Поиск green объекта по HSV, контурам и boundingRect."""

import cv2
import numpy as np


CAMERA_INDEX = 0
MIN_CONTOUR_AREA = 500
HSV_RANGES = [
    (np.array((40, 80, 60), dtype=np.uint8), np.array((85, 255, 255), dtype=np.uint8))
]


def build_mask(hsv: np.ndarray) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = build_mask(hsv)
            contours, _hierarchy = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < MIN_CONTOUR_AREA:
                    continue

                x, y, width, height = cv2.boundingRect(contour)
                center_x = x + width // 2
                center_y = y + height // 2

                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + width, y + height),
                    (0, 255, 255),
                    2,
                )
                cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)
                cv2.putText(
                    frame,
                    f"GREEN: x={center_x}, y={center_y}, area={area:.0f}",
                    (x, max(y - 10, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (255, 255, 255),
                    2,
                )

            cv2.imshow("GREEN objects", frame)
            cv2.imshow("Mask", mask)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

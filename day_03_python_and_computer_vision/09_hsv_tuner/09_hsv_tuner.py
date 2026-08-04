#!/usr/bin/env python3
"""Ползунки для подбора одного диапазона HSV."""

import cv2
import numpy as np


CAMERA_INDEX = 0
CONTROL_WINDOW = "HSV controls"


def nothing(_value: int) -> None:
    pass


def main() -> None:
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась.")

    cv2.namedWindow(CONTROL_WINDOW)
    cv2.createTrackbar("H min", CONTROL_WINDOW, 0, 179, nothing)
    cv2.createTrackbar("H max", CONTROL_WINDOW, 179, 179, nothing)
    cv2.createTrackbar("S min", CONTROL_WINDOW, 0, 255, nothing)
    cv2.createTrackbar("S max", CONTROL_WINDOW, 255, 255, nothing)
    cv2.createTrackbar("V min", CONTROL_WINDOW, 0, 255, nothing)
    cv2.createTrackbar("V max", CONTROL_WINDOW, 255, 255, nothing)

    print("Esc — выход, S — напечатать текущий диапазон.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            h_min = cv2.getTrackbarPos("H min", CONTROL_WINDOW)
            h_max = cv2.getTrackbarPos("H max", CONTROL_WINDOW)
            s_min = cv2.getTrackbarPos("S min", CONTROL_WINDOW)
            s_max = cv2.getTrackbarPos("S max", CONTROL_WINDOW)
            v_min = cv2.getTrackbarPos("V min", CONTROL_WINDOW)
            v_max = cv2.getTrackbarPos("V max", CONTROL_WINDOW)

            lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
            upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(frame, frame, mask=mask)

            text = (
                f"H {h_min}-{h_max}  "
                f"S {s_min}-{s_max}  "
                f"V {v_min}-{v_max}"
            )
            cv2.putText(
                frame,
                text,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            cv2.imshow("Original", frame)
            cv2.imshow("Mask", mask)
            cv2.imshow("Selected color", result)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key in (ord("s"), ord("S")):
                print(
                    f"lower = ({h_min}, {s_min}, {v_min}), "
                    f"upper = ({h_max}, {s_max}, {v_max})"
                )
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

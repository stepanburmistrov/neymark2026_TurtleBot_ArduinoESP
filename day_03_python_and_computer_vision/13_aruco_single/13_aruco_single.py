#!/usr/bin/env python3
"""Поиск одного ArUco-маркера из словаря DICT_4X4_50."""

import cv2
import numpy as np


CAMERA_INDEX = 0


def main() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(
        dictionary,
        cv2.aruco.DetectorParameters(),
    )

    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        raise RuntimeError("Камера не открылась.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            corners, ids, _rejected = detector.detectMarkers(frame)

            if ids is not None:
                # Для первого найденного маркера показываем все четыре вершины.
                points = np.rint(corners[0].reshape(4, 2)).astype(int)
                marker_id = int(ids[0, 0])

                for index, point in enumerate(points):
                    pixel = tuple(point)
                    cv2.circle(frame, pixel, 6, (0, 255, 255), -1)
                    cv2.putText(
                        frame,
                        str(index),
                        (pixel[0] + 8, pixel[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                    )

                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"ID {marker_id}",
                    tuple(points[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

            cv2.imshow("Single ArUco", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

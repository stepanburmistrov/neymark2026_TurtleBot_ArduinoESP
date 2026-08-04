#!/usr/bin/env python3
"""Определяет центр и угол ориентации ArUco-маркера."""

import math

import cv2
import numpy as np


CAMERA_INDEX = 0


def marker_geometry(corners: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    points = corners.reshape(4, 2)
    center = points.mean(axis=0)

    # Вершины 0 и 1 образуют верхнюю сторону маркера.
    top_middle = 0.5 * (points[0] + points[1])
    heading = top_middle - center

    # В изображении Y направлена вниз. Для привычного математического угла
    # меняем знак вертикальной компоненты.
    angle_rad = math.atan2(-float(heading[1]), float(heading[0]))
    return center, top_middle, angle_rad


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
                marker_corners = corners[0]
                marker_id = int(ids[0, 0])
                center, top_middle, angle_rad = marker_geometry(marker_corners)

                points = np.rint(marker_corners.reshape(4, 2)).astype(np.int32)
                center_px = tuple(np.rint(center).astype(int))
                top_px = tuple(np.rint(top_middle).astype(int))

                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                cv2.circle(frame, center_px, 6, (0, 0, 255), -1)
                cv2.arrowedLine(frame, center_px, top_px, (255, 100, 0), 3)
                cv2.putText(
                    frame,
                    f"ID={marker_id} center=({center_px[0]}, {center_px[1]})",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"angle={math.degrees(angle_rad):.1f} deg",
                    (20, 68),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            cv2.imshow("ArUco center and angle", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Поиск нескольких ArUco-маркеров DICT_4X4_50."""

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
            found: list[str] = []

            if ids is not None:
                for marker_corners, marker_id_array in zip(corners, ids):
                    marker_id = int(marker_id_array[0])
                    points = marker_corners.reshape(4, 2)
                    center = points.mean(axis=0)

                    polygon = np.rint(points).astype(np.int32)
                    center_px = tuple(np.rint(center).astype(int))
                    cv2.polylines(frame, [polygon], True, (0, 255, 0), 2)
                    cv2.circle(frame, center_px, 5, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"ID {marker_id}: ({center_px[0]}, {center_px[1]})",
                        (center_px[0] + 8, center_px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        2,
                    )
                    found.append(str(marker_id))

            cv2.putText(
                frame,
                "IDs: " + (", ".join(sorted(found)) if found else "none"),
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

            cv2.imshow("Multiple ArUco", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

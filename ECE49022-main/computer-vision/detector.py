"""Marker detector used by the computer-vision subsystem.

This file is relevant because it converts raw camera frames into target marker
geometry, which is the first step in producing angle and distance for fusion.
"""

import cv2
import numpy as np

class ArucoTargetDetector:
    def __init__(self, target_id=5):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.target_id = target_id

    def detect(self, frame):
        corners, ids, rejected = self.detector.detectMarkers(frame)

        target_info = None

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.target_id:
                    marker_corners = corners[i][0]
                    x_coords = marker_corners[:, 0]
                    y_coords = marker_corners[:, 1]
                    x1, y1 = np.min(x_coords), np.min(y_coords)
                    x2, y2 = np.max(x_coords), np.max(y_coords)
                    w = x2 - x1
                    h = y2 - y1
                    cx = x1 + w / 2.0
                    target_info = {
                        'id': marker_id,
                        'cx': cx,
                        'w': w,
                        'box': [int(x1), int(y1), int(w), int(h)],
                        'corners': corners[i]
                    }
                    break
        return target_info

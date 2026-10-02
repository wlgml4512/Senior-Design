"""Geometry helpers for turning detections into robot-facing measurements.

This file is relevant because it translates image-space marker size and position
into the angle and distance values that the planner uses for following.
"""

import numpy as np

class SpatialEstimator:
    def __init__(self, frame_width=320, hfov_deg=66.0, marker_size_m=0.15):
        self.frame_width = frame_width
        self.c_x = frame_width / 2.0
        self.f_x = self.c_x / np.tan(np.radians(hfov_deg / 2.0))
        self.marker_size_m = marker_size_m
    #center of target - center of camera div by focal length
    def estimate_angle(self, center_x):
        angle_rad = np.arctan((center_x - self.c_x) / self.f_x)
        # Fusion treats positive angular commands as "turn left". Negate the
        # image-space angle so targets on the left produce positive angles.
        return -np.degrees(angle_rad)

    #https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    def estimate_distance(self, bbox_width):
        if bbox_width <= 0:
            return -1.0
        return (self.f_x * self.marker_size_m) / bbox_width

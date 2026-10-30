"""Relative LiDAR centering law; no odometry, ROS, timer or I/O."""
from __future__ import annotations
import math
import numpy as np


class LidarFrenetCore:
    def __init__(self):
        self.L = 0.26
        self.Ld = 1.3
        self.throttle_cmd = 0.4

    def command(self, scan):
        d_left, d_right = self.extract_left_right(scan)
        if d_left is None:
            return None
        e_y = (d_left - d_right) / 2.0
        kappa = 2.0 * e_y / (self.Ld ** 2)
        delta = math.atan(self.L * kappa)
        return self.throttle_cmd, delta

    def extract_left_right(self, scan: LaserScan):
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            ranges = np.array(scan.ranges)

            valid = np.isfinite(ranges)
            angles = angles[valid]
            ranges = ranges[valid]

            left = (angles > np.deg2rad(70)) & (angles < np.deg2rad(110))
            right = (angles > -np.deg2rad(110)) & (angles < -np.deg2rad(70))

            if not np.any(left) or not np.any(right):
                return None, None

            return np.median(ranges[left]), np.median(ranges[right])

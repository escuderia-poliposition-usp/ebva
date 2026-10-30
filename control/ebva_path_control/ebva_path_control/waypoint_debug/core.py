"""Selected waypoint laws and target selection, without ROS or I/O."""
import math
import numpy as np

MODES = ('carrot', 'stanley', 'pure_pursuit')


class WaypointEffortCore:
    def __init__(self, waypoints, control_mode='carrot'):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('At least two finite XY reference points required')
        if control_mode not in MODES:
            raise ValueError('Unknown control_mode')
        self.waypoints = points.copy()
        self.control_mode = control_mode
        self.lookahead_distance = 0.8
        self.wheelbase = 0.33
        self.k_heading = 1.5
        self.k_stanley = 2.0
        self.throttle = 0.2

    def command(self, x, y, yaw):
        dists = np.linalg.norm(self.waypoints - [x, y], axis=1)
        idx_closest = np.argmin(dists)
        idx_look = idx_closest
        for i in range(idx_closest, len(self.waypoints)):
            if np.linalg.norm(self.waypoints[i] - [x, y]) >= self.lookahead_distance:
                idx_look = i
                break
        target = self.waypoints[idx_look]
        dx = target[0] - x
        dy = target[1] - y
        x_v = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_v = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if x_v <= 0.0:
            return None
        if self.control_mode == "carrot":
            steering = self.carrot_control(dx, dy, yaw)
        elif self.control_mode == "stanley":
            steering = self.stanley_control(dx, dy, yaw, y_v)
        else:
            steering = self.pure_pursuit_control(y_v)
        steering = np.clip(steering, -0.41, 0.41)
        return self.throttle, float(steering), target.copy(), float(y_v)

    def carrot_control(self, dx, dy, yaw):
            psi_ref = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(psi_ref - yaw),
                math.cos(psi_ref - yaw)
            )
            return self.k_heading * heading_error

    def stanley_control(self, dx, dy, yaw, y_v):
            psi_ref = math.atan2(dy, dx)
            heading_error = math.atan2(
                math.sin(psi_ref - yaw),
                math.cos(psi_ref - yaw)
            )
            return heading_error + math.atan2(self.k_stanley * y_v, 1.0)

    def pure_pursuit_control(self, y_v):
            curvature = 2.0 * y_v / (self.lookahead_distance ** 2)
            return math.atan(self.wheelbase * curvature)

import numpy as np
import math


class PurePursuitFrenet:
    def __init__(self, waypoints, lookahead):
        """
        waypoints: np.array [[x, y], ...]
        lookahead: float
        """
        self.waypoints = waypoints
        self.lookahead = lookahead

    def find_target_index(self, x, y):
        dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        return np.argmin(dists)

    def compute_control(self, x, y, yaw):
        idx = self.find_target_index(x, y)

        for i in range(idx, len(self.waypoints)):
            dx = self.waypoints[i][0] - x
            dy = self.waypoints[i][1] - y
            if math.hypot(dx, dy) >= self.lookahead:
                target = self.waypoints[i]
                break
        else:
            target = self.waypoints[-1]

        dx = target[0] - x
        dy = target[1] - y

        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if local_x == 0.0:
            return 0.0

        curvature = 2.0 * local_y / (self.lookahead ** 2)
        return curvature


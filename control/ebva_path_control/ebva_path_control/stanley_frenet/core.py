"""Original Stanley/Frenet numerical law; frame/time contracts live outside."""
import math
import numpy as np

def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class StanleyFrenetCore:
    def __init__(self):
        self.k_e = .8
        self.k_s = 1.2
        self.eps = .1
        self.max_steer = .4
        self.speed_cmd = .6
        self.yaw_ref = 0.

    def command(self, lateral_error_m, vx, vy, yaw):
        self.e_y = lateral_error_m
        self.v = math.hypot(vx, vy)
        self.yaw = yaw
        theta_e = wrap_angle(self.yaw - self.yaw_ref)
        e_y = -self.e_y
        delta = theta_e + math.atan(self.k_s * e_y / (self.v + self.eps)) - self.k_e * e_y
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        speed = -self.speed_cmd
        return float(speed), float(delta)

"""Explicit software map for dimensionless effort plus a geometric angle."""
from dataclasses import dataclass
import math
from .core import Actuation

CONTRACTS = frozenset({"waypoint_control_carrot_v1", "waypoint_control_stanley_v1",
                       "waypoint_control_pure_pursuit_v1", "lidar_frenet_pp_v1"})

@dataclass(frozen=True)
class EffortSteeringMapping:
    longitudinal_gain_erpm_per_effort: float
    steering_gain_servo_per_rad: float
    servo_center: float
    max_longitudinal_effort: float
    max_steering_rad: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if any(isinstance(v, bool) or not math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("Finite explicit fixture mapping required")
        if (self.longitudinal_gain_erpm_per_effort == 0 or self.steering_gain_servo_per_rad == 0
                or self.max_longitudinal_effort <= 0 or not 0 < self.max_steering_rad < math.pi/2
                or self.erpm_limit <= 0 or not 0 <= self.servo_min < self.servo_center < self.servo_max <= 1):
            raise ValueError("Invalid effort/angle mapping bounds")

    def convert(self, effort, angle_rad):
        if not all(math.isfinite(v) for v in (effort, angle_rad)):
            raise ValueError("Nonfinite effort/angle")
        if abs(effort) > self.max_longitudinal_effort or abs(angle_rad) > self.max_steering_rad:
            raise ValueError("Effort/angle outside declared source domain")
        erpm = effort*self.longitudinal_gain_erpm_per_effort
        servo = self.servo_center+angle_rad*self.steering_gain_servo_per_rad
        return Actuation(max(-self.erpm_limit, min(self.erpm_limit, erpm)),
                         max(self.servo_min, min(self.servo_max, servo)))

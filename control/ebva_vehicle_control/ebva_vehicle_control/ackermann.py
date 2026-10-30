"""Explicit affine fixture conversion; no ROS or vehicle defaults."""
from dataclasses import dataclass
import math
from .core import Actuation


@dataclass(frozen=True)
class AckermannMapping:
    speed_gain_erpm_per_mps: float
    steering_gain_servo_per_rad: float
    servo_center: float
    max_speed_mps: float
    max_steering_rad: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if any(isinstance(v, bool) or not math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("Mapping must contain finite numbers")
        if (self.speed_gain_erpm_per_mps == 0 or self.steering_gain_servo_per_rad == 0
                or self.max_speed_mps <= 0 or not 0 < self.max_steering_rad < math.pi/2
                or self.erpm_limit <= 0
                or not 0 <= self.servo_min < self.servo_center < self.servo_max <= 1):
            raise ValueError("Invalid explicit Ackermann mapping")

    def convert(self, speed_mps, steering_rad):
        if not all(math.isfinite(v) for v in (speed_mps, steering_rad)):
            raise ValueError("Nonfinite Ackermann command")
        speed = max(-self.max_speed_mps, min(self.max_speed_mps, speed_mps))
        angle = max(-self.max_steering_rad, min(self.max_steering_rad, steering_rad))
        erpm = speed*self.speed_gain_erpm_per_mps
        servo = self.servo_center+angle*self.steering_gain_servo_per_rad
        return Actuation(max(-self.erpm_limit, min(self.erpm_limit, erpm)),
                         max(self.servo_min, min(self.servo_max, servo)))

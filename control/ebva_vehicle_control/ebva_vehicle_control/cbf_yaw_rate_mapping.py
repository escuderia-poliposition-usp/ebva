"""Explicit CBF software fixture; acceleration is never silently a speed target."""
from dataclasses import dataclass
import math
from .core import Actuation


@dataclass(frozen=True)
class CbfYawRateMapping:
    acceleration_gain_erpm_per_mps2: float
    steering_gain_servo_per_rad: float
    model_wheelbase_m: float
    speed_floor_mps: float
    positive_acceleration_cap_mps2: float
    max_acceleration_mps2: float
    max_yaw_rate_rps: float
    max_steering_rad: float
    servo_center: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if any(isinstance(v,bool) or not math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("Finite explicit fixture values required")
        if (self.acceleration_gain_erpm_per_mps2 == 0 or self.steering_gain_servo_per_rad == 0
                or min(self.model_wheelbase_m,self.speed_floor_mps,self.positive_acceleration_cap_mps2,
                       self.max_acceleration_mps2,self.max_yaw_rate_rps,self.erpm_limit) <= 0
                or self.positive_acceleration_cap_mps2 > self.max_acceleration_mps2
                or not 0 < self.max_steering_rad < math.pi/2
                or not 0 <= self.servo_min < self.servo_center < self.servo_max <= 1):
            raise ValueError("Invalid CBF software mapping")

    def convert(self, acceleration_mps2, yaw_rate_rps, longitudinal_velocity_mps):
        if (not all(math.isfinite(v) for v in (acceleration_mps2,yaw_rate_rps,longitudinal_velocity_mps))
                or abs(acceleration_mps2) > self.max_acceleration_mps2+1e-9
                or abs(yaw_rate_rps) > self.max_yaw_rate_rps+1e-9):
            raise ValueError("CBF command outside declared model domain")
        # Preserved filter publication cap/inversion, not measured vehicle calibration.
        acceleration=min(acceleration_mps2,self.positive_acceleration_cap_mps2)
        effective_speed=max(abs(longitudinal_velocity_mps),self.speed_floor_mps)*(
            -1. if longitudinal_velocity_mps < 0 else 1.)
        angle=math.atan(self.model_wheelbase_m*yaw_rate_rps/effective_speed)
        angle=max(-self.max_steering_rad,min(self.max_steering_rad,angle))
        erpm=acceleration*self.acceleration_gain_erpm_per_mps2
        servo=self.servo_center+angle*self.steering_gain_servo_per_rad
        return Actuation(max(-self.erpm_limit,min(self.erpm_limit,erpm)),
                         max(self.servo_min,min(self.servo_max,servo)))

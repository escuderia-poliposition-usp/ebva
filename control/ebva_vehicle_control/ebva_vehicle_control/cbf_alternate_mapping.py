"""Preserved tangent publication projection, with artificial longitudinal map."""
from dataclasses import dataclass
import math
from .core import Actuation

class UnrepresentableSteering(ValueError):
    """A feasible model control cannot be expressed by the selected angle domain."""

@dataclass(frozen=True)
class CbfSteeringTangentMapping:
    acceleration_gain_erpm_per_mps2: float
    steering_gain_servo_per_rad: float
    positive_acceleration_cap_mps2: float
    max_acceleration_mps2: float
    max_steering_tangent: float
    max_steering_rad: float
    servo_center: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if any(isinstance(v,bool) or not math.isfinite(v) for v in self.__dict__.values()):
            raise ValueError("Finite explicit fixture values required")
        if (self.acceleration_gain_erpm_per_mps2==0 or self.steering_gain_servo_per_rad==0
                or min(self.positive_acceleration_cap_mps2,self.max_acceleration_mps2,
                       self.max_steering_tangent,self.erpm_limit)<=0
                or self.positive_acceleration_cap_mps2>self.max_acceleration_mps2
                or not 0<self.max_steering_rad<math.pi/2
                or not 0<=self.servo_min<self.servo_center<self.servo_max<=1):
            raise ValueError("Invalid tangent fixture domain")

    def convert(self,acceleration,steering_tangent,longitudinal_velocity):
        if (not all(math.isfinite(v) for v in (acceleration,steering_tangent,longitudinal_velocity))
                or abs(acceleration)>self.max_acceleration_mps2+1e-9
                or abs(steering_tangent)>self.max_steering_tangent+1e-9):
            raise ValueError("Outside steering-tangent model domain")
        angle=math.atan(steering_tangent)
        # Clipping a feasible tangent changes the QP control and can violate A*u<=b.
        # Reject unrepresentable controls; retain the legacy clip only as history.
        if abs(angle)>self.max_steering_rad+1e-12:
            raise UnrepresentableSteering("Tangent output cannot be represented within the declared steering angle")
        erpm=min(acceleration,self.positive_acceleration_cap_mps2)*self.acceleration_gain_erpm_per_mps2
        servo=self.servo_center+angle*self.steering_gain_servo_per_rad
        return Actuation(max(-self.erpm_limit,min(self.erpm_limit,erpm)),max(self.servo_min,min(self.servo_max,servo)))

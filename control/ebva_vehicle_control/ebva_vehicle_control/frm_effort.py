"""Artificial fixture mapping for the evidenced FRM point-sample contract."""
from dataclasses import dataclass
import math
from .core import Actuation

CONTRACT="frm_point_sample_v1"


@dataclass(frozen=True)
class FrmEffortMapping:
    longitudinal_gain_erpm_per_effort: float
    steering_gain_servo_per_effort: float
    servo_center: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if (any(isinstance(v,bool) or not math.isfinite(v) for v in self.__dict__.values())
                or self.longitudinal_gain_erpm_per_effort==0 or self.steering_gain_servo_per_effort==0
                or self.erpm_limit<=0 or not 0<=self.servo_min<self.servo_center<self.servo_max<=1):
            raise ValueError("Explicit finite nonzero fixture gains and bounds required")

    def convert(self, longitudinal, steering):
        if not all(math.isfinite(v) and -1<=v<=1 for v in (longitudinal,steering)):
            raise ValueError("FRM effort must be finite and in [-1,1]")
        return Actuation(max(-self.erpm_limit,min(self.erpm_limit,longitudinal*self.longitudinal_gain_erpm_per_effort)),
                         max(self.servo_min,min(self.servo_max,self.servo_center+steering*self.steering_gain_servo_per_effort)))

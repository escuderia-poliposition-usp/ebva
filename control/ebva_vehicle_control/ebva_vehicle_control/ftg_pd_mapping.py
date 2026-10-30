"""FTG-specific artificial throttle/PD-effort map, without angle interpretation."""
from dataclasses import dataclass
import math
from .core import Actuation

CONTRACT="ftg_pd_sample_v1"


@dataclass(frozen=True)
class FtgPdMapping:
    throttle_gain_erpm_per_unit: float
    pd_gain_servo_per_unit: float
    servo_center: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if (any(isinstance(v,bool) or not math.isfinite(v) for v in self.__dict__.values())
                or self.throttle_gain_erpm_per_unit==0 or self.pd_gain_servo_per_unit==0
                or self.erpm_limit<=0 or not 0<=self.servo_min<self.servo_center<self.servo_max<=1):
            raise ValueError("Explicit finite fixture gains and limits required")

    def convert(self, throttle, effort):
        if not math.isfinite(throttle) or not 0<=throttle<=.120001 or not math.isfinite(effort):
            raise ValueError("Invalid FTG throttle or nonfinite PD effort")
        # Do not normalize/saturate the PD internal output before its specific map.
        return Actuation(max(-self.erpm_limit,min(self.erpm_limit,throttle*self.throttle_gain_erpm_per_unit)),
            max(self.servo_min,min(self.servo_max,self.servo_center+effort*self.pd_gain_servo_per_unit)))

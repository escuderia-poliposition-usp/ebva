"""Explicit normalized supervisor mapping for synthetic software campaigns."""
from dataclasses import dataclass
import math
from .core import Actuation

METHODS = ("pi_straight", "p_curve", "p_intersection", "wall_recovery", "straight_rule")
CONTRACTS = frozenset("supervisor_" + method + "_v1" for method in METHODS)


@dataclass(frozen=True)
class SupervisorEffortMapping:
    longitudinal_gain_erpm_per_effort: float
    steering_gain_servo_per_effort: float
    servo_center: float
    erpm_limit: float
    servo_min: float
    servo_max: float

    def __post_init__(self):
        if (any(type(v) not in (int, float) or not math.isfinite(v)
                for v in self.__dict__.values())
                or self.longitudinal_gain_erpm_per_effort == 0
                or self.steering_gain_servo_per_effort == 0 or self.erpm_limit <= 0
                or not 0 <= self.servo_min < self.servo_center < self.servo_max <= 1):
            raise ValueError("Explicit finite nonzero supervisor fixture gains and bounds required")

    def convert(self, longitudinal, steering):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (longitudinal, steering)):
            raise ValueError("Supervisor efforts must be finite and in [-1,1]")
        erpm = max(-self.erpm_limit, min(self.erpm_limit,
                   longitudinal * self.longitudinal_gain_erpm_per_effort))
        servo = max(self.servo_min, min(self.servo_max,
                    self.servo_center + steering * self.steering_gain_servo_per_effort))
        return Actuation(erpm, servo)

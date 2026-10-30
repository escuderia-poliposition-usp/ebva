"""Pure supervisor lifecycle and original controller selection.

The numeric kernels retain their original per-cycle laws. Curvature is a model
feature with its producer scale, never silently converted to inverse metres.
"""
from collections import deque
from dataclasses import dataclass
import math

from .controllers.pi_straight import PIStraightController
from .controllers.p_curve import PCurveController
from .controllers.p_intersection import PIntersectionController
from .controllers.wall_controller import WallController

METHODS = ("supervised", "pi_straight", "p_curve", "p_intersection",
           "wall_recovery", "straight_rule")
TRACKS = ("straight", "left_curve", "right_curve", "intersection", "walls")
POSITIONS = ("left", "middle", "right")
CONTRACTS = tuple("supervisor_" + mode + "_v1" for mode in METHODS if mode != "supervised")
SELECTED = {"straight": "pi_straight", "left_curve": "p_curve",
            "right_curve": "p_curve", "intersection": "p_intersection",
            "walls": "wall_recovery"}


@dataclass(frozen=True)
class Classification:
    stamp_ns: int
    frame_id: str
    valid: bool
    track_type: str
    position: str
    track_probability: float
    track_entropy: float
    position_probability: float
    position_entropy: float
    curvature_feature: float

    def validate(self, expected_frame):
        if type(self.stamp_ns) is not int or self.stamp_ns <= 0:
            raise ValueError("invalid_stamp")
        if self.frame_id != expected_frame:
            raise ValueError("frame_mismatch")
        if self.valid is not True:
            raise ValueError("classification_invalid")
        if self.track_type not in TRACKS or self.position not in POSITIONS:
            raise ValueError("unknown_classification")
        values = (self.track_probability, self.track_entropy,
                  self.position_probability, self.position_entropy,
                  self.curvature_feature)
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError("nonfinite_classification")
        if not (0 <= self.track_probability <= 1 and 0 <= self.position_probability <= 1):
            raise ValueError("invalid_probability")
        if self.track_entropy < 0 or self.position_entropy < 0:
            raise ValueError("invalid_entropy")

    def state(self):
        return {"track_type": self.track_type, "position": self.position,
                "track_prob": self.track_probability, "track_entropy": self.track_entropy,
                "pos_prob": self.position_probability, "pos_entropy": self.position_entropy,
                "curvature": self.curvature_feature}


@dataclass(frozen=True)
class Settings:
    method: str = "supervised"
    expected_frame: str = "classification_model"
    source_timeout_s: float = .2
    control_rate_hz: float = 20.
    kp_reta: float = 1.
    ki_reta: float = .1
    kp_curva: float = 1.5
    curvature_gain: float = .3
    kp_intersecao: float = .8

    def __post_init__(self):
        if self.method not in METHODS:
            raise ValueError("Unknown supervisor method")
        if not isinstance(self.expected_frame, str) or not self.expected_frame:
            raise ValueError("Explicit classifier model frame required")
        for value in (self.source_timeout_s, self.control_rate_hz):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("Finite positive rate and timeout required")
        if self.control_rate_hz != 20.:
            raise ValueError("Original per-cycle laws require 20 Hz; retuning is separate")
        for name in ("kp_reta", "ki_reta", "kp_curva", "curvature_gain", "kp_intersecao"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError("Finite controller gain required: " + name)


@dataclass(frozen=True)
class Command:
    stamp_ns: int
    frame_id: str
    contract: str
    longitudinal: float
    steering: float
    selected_method: str


class Supervisor:
    def __init__(self, settings=Settings()):
        self.settings = settings
        self.last_stamp_ns = None
        self.reason = "waiting_for_classification"
        self.selected_method = None
        self._reset()

    def _reset(self):
        self.classification = None
        self.received = None
        self.history = deque(maxlen=40)
        self.previous_method = None
        self.straight = PIStraightController(self.settings.kp_reta, self.settings.ki_reta)
        self.curve = PCurveController(self.settings.kp_curva, self.settings.curvature_gain)
        self.intersection = PIntersectionController(self.settings.kp_intersecao)
        self.wall = WallController()

    def invalidate(self, reason):
        self._reset()
        self.reason = reason
        self.selected_method = None

    def configure(self, settings):
        changed_identity = (settings.method != self.settings.method
                            or settings.expected_frame != self.settings.expected_frame)
        self.settings = settings
        if changed_identity:
            self.invalidate("configuration_changed")
        else:
            self.straight.kp, self.straight.ki = settings.kp_reta, settings.ki_reta
            self.curve.kp, self.curve.curvature_gain = settings.kp_curva, settings.curvature_gain
            self.intersection.kp = settings.kp_intersecao

    def observe(self, value, now_ns, received_monotonic):
        try:
            if type(value.stamp_ns) is not int or value.stamp_ns <= 0:
                raise ValueError("invalid_stamp")
            if value.frame_id != self.settings.expected_frame:
                raise ValueError("frame_mismatch")
            age = (now_ns - value.stamp_ns) / 1e9
            if not math.isfinite(age) or not 0 <= age < self.settings.source_timeout_s:
                raise ValueError("stale_or_future_classification")
            if self.last_stamp_ns is not None and value.stamp_ns <= self.last_stamp_ns:
                raise ValueError("duplicate_or_reordered_classification")
            # A fresh observation remains seen even if its payload is invalid;
            # changing validity on the same observation cannot rearm a source.
            # Future/stale/wrong-frame headers never advance this watermark.
            self.last_stamp_ns = value.stamp_ns
            value.validate(self.settings.expected_frame)
            if not math.isfinite(received_monotonic):
                raise ValueError("invalid_receipt_clock")
        except ValueError as error:
            self.invalidate(str(error))
            return False
        self.last_stamp_ns = value.stamp_ns
        self.classification = value
        self.received = received_monotonic
        self.reason = "classification_accepted"
        return True

    def command(self, now_ns, now_monotonic):
        value = self.classification
        if value is None:
            return None
        age = (now_ns - value.stamp_ns) / 1e9
        receipt_age = now_monotonic - self.received
        if not (math.isfinite(age) and 0 <= age < self.settings.source_timeout_s
                and math.isfinite(receipt_age) and 0 <= receipt_age < self.settings.source_timeout_s):
            self.invalidate("classification_expired")
            return None
        method = (SELECTED[value.track_type] if self.settings.method == "supervised"
                  else self.settings.method)
        state = value.state()
        state["command_history"] = list(self.history)
        if self.previous_method == "wall_recovery" and method != "wall_recovery":
            self.wall.reset_recovery()
        self.previous_method = method
        if method == "straight_rule":
            steering, throttle = 0., (1. if value.track_type == "straight" else 0.)
        else:
            kernel = {"pi_straight": self.straight, "p_curve": self.curve,
                      "p_intersection": self.intersection, "wall_recovery": self.wall}[method]
            steering, throttle = kernel.control(state)
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (throttle, steering)):
            self.invalidate("invalid_controller_output")
            return None
        if method != "wall_recovery":
            self.history.append({"steer": steering, "throttle": throttle})
        self.selected_method = method
        self.reason = "controller_command"
        return Command(value.stamp_ns, value.frame_id, "supervisor_" + method + "_v1",
                       float(throttle), float(steering), method)

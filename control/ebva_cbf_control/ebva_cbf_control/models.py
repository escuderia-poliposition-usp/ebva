"""Pure model selection and admissibility gate; no ROS or actuator conversion."""
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Optional, Tuple
import math
import numpy as np
from .qp import qp_hildreth
from .model_kernels.yaw_rate import cbf_filter as yaw_rate
from .model_kernels.yaw_rate_margin import cbf_filter as yaw_rate_margin
from .model_kernels.steering_tangent import cbf_filter as steering_tangent

MODELS = {"yaw_rate": yaw_rate, "yaw_rate_margin": yaw_rate_margin,
          "steering_tangent": steering_tangent}


@dataclass(frozen=True)
class ModelDecision:
    model: str
    status: str
    command: Optional[Tuple[float, float]] = None
    raw_control: Optional[Tuple[float, float]] = None
    barriers: Tuple[float, ...] = ()
    max_constraint_violation: Optional[float] = None
    box_contradictions: Tuple[int, ...] = ()

    @property
    def accepted(self):
        return self.command is not None


class CbfKernel:
    """Run one preserved kernel, then reject results failing its inequalities.

    Yaw models use [acceleration m/s^2, yaw rate rad/s]. Tangent uses
    [acceleration m/s^2, tan(steering angle)] internally. There is no physical
    mapping, solver fallback, convergence assertion or manual RC input here.
    """
    def __init__(self, model, wheelbase=.33, tolerance=1e-6):
        if model not in MODELS:
            raise ValueError("Unknown CBF model: " + str(model))
        if not math.isfinite(wheelbase) or wheelbase <= 0:
            raise ValueError("Positive finite wheelbase required")
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("Positive finite constraint tolerance required")
        self.model, self.wheelbase, self.tolerance = model, wheelbase, tolerance

    def evaluate(self, speed, ranges, angles, nominal):
        try:
            speed = float(speed)
            ranges, angles, nominal = (np.asarray(x, dtype=float)
                                       for x in (ranges, angles, nominal))
        except (TypeError, ValueError, OverflowError):
            return ModelDecision(self.model, "invalid_input")
        if (not math.isfinite(speed) or ranges.ndim != 1 or angles.ndim != 1
                or len(ranges) < 2 or ranges.shape != angles.shape
                or nominal.shape != (2,) or not np.isfinite(nominal).all()
                or not np.isfinite(ranges).all() or not np.isfinite(angles).all()
                or np.any(ranges <= 0) or np.any(np.diff(angles) <= 0)
                or np.any(np.abs(angles) > np.pi / 2 + 1e-12)):
            return ModelDecision(self.model, "invalid_input")
        matrices = {}
        def solve(H, f, A, b):
            matrices.update(A=A.copy(), b=b.copy())
            if not all(np.isfinite(x).all() for x in (H, f, A, b)):
                raise FloatingPointError("Nonfinite QP")
            return qp_hildreth(H, f, A, b)
        context = SimpleNamespace(L=self.wheelbase, qp_hildreth=solve)
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                control, h = MODELS[self.model](context, speed, ranges, angles, nominal)
                A, b = matrices["A"], matrices["b"]
                violation = float(np.max(A @ control - b))
                # Final four inequalities are the preserved model's box.
                lower = np.array([-b[-3], -b[-1]])
                upper = np.array([b[-4], b[-2]])
                box_minimum = np.sum(np.where(A >= 0, A * lower, A * upper), axis=1)
                contradictions = tuple(int(i) for i in np.flatnonzero(box_minimum > b + self.tolerance))
        except (ArithmeticError, ValueError, np.linalg.LinAlgError):
            return ModelDecision(self.model, "numerical_failure")
        if not np.isfinite(control).all() or not np.isfinite(h).all() or not math.isfinite(violation):
            return ModelDecision(self.model, "nonfinite_result")
        raw = tuple(float(x) for x in control)
        barriers = tuple(float(x) for x in h)
        if contradictions:
            return ModelDecision(self.model, "infeasible_box_bound", None, raw,
                                 barriers, violation, contradictions)
        if violation > self.tolerance:
            return ModelDecision(self.model, "constraint_violation", None, raw, barriers, violation)
        return ModelDecision(self.model, "accepted", raw, raw, barriers, violation)


def legacy_filter_projection(model, control, speed, wheelbase=.33):
    """Characterize legacy publication; not the new command or actuator contract.

    Returns legacy [capped acceleration, steering angle]. Retains asymmetric
    positive acceleration cap and model-specific limits; no servo calibration.
    """
    if model not in MODELS:
        raise ValueError("Unknown CBF model")
    acceleration, lateral = control
    if model == "steering_tangent":
        angle = np.clip(np.arctan(lateral), -np.radians(20), np.radians(20))
    else:
        effective_speed = max(abs(speed), .1) * np.sign(speed if speed != 0 else 1.)
        angle = np.clip(np.arctan(wheelbase * lateral / effective_speed), -np.pi / 6, np.pi / 6)
    return float(min(acceleration, .24)), float(angle)

"""Stamped synthetic CBF composition. No ROS I/O, authority or actuator outputs."""
from dataclasses import dataclass
import math
from typing import Optional, Tuple
import numpy as np
from ebva_control_contracts import MeasurementLease
from .models import CbfKernel, ModelDecision


@dataclass(frozen=True)
class SourceStamp:
    seconds: float
    frame: str


@dataclass(frozen=True)
class ScanSample:
    header: SourceStamp
    ranges: Tuple[float, ...]
    angle_min: float
    angle_increment: float
    angle_max: float
    range_min: float = .05
    range_max: float = 10.


@dataclass(frozen=True)
class SpeedSample:
    # Frame is velocity basis (Odometry.child_frame_id), not pose parent.
    header: SourceStamp
    longitudinal_mps: float


@dataclass(frozen=True)
class YawRateNominal:
    header: SourceStamp
    model: str
    acceleration_mps2: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class SteeringAngleNominal:
    header: SourceStamp
    model: str
    acceleration_mps2: float
    steering_angle_rad: float


@dataclass(frozen=True)
class SoftwareCommand:
    model: str
    control: Tuple[float, float]
    validity_header: SourceStamp
    scan_header: SourceStamp
    speed_header: SourceStamp
    nominal_header: SourceStamp


@dataclass(frozen=True)
class PipelineDecision:
    status: str
    command: Optional[SoftwareCommand] = None
    kernel: Optional[ModelDecision] = None


class CbfSoftwarePipeline:
    """Explicit aligned fixture frame; nominal callbacks are the only trigger.

    No automatic TF is inferred. Each source retains acquisition time and an
    independent monotonic lease. Rejected samples never renew these leases.
    Filtered nominal is an autonomy experiment, never the manual RC route.
    """
    def __init__(self, model, frame="fixture_control_flu", timeout=.2,
                 future_slack=.02, max_skew=.08, wheelbase=.33):
        if (not math.isfinite(max_skew) or max_skew < 0
                or not math.isfinite(future_slack) or future_slack < 0):
            raise ValueError("Finite nonnegative time tolerances required")
        self.kernel = CbfKernel(model, wheelbase)
        self.model, self.frame, self.max_skew = model, frame, max_skew
        self.leases = {key: MeasurementLease(timeout, frame, future_slack)
                       for key in ("scan", "speed", "nominal")}
        self.scan = self.speed = None

    def _observe(self, source, header, ros_now, monotonic_now):
        return isinstance(header, SourceStamp) and self.leases[source].observe(
            header.seconds, ros_now, monotonic_now, header.frame)

    def observe_scan(self, sample, ros_now, monotonic_now):
        if not isinstance(sample, ScanSample):
            return False
        try:
            ranges = np.asarray(sample.ranges, dtype=float)
            scalars = (sample.angle_min, sample.angle_max, sample.angle_increment,
                       sample.range_min, sample.range_max)
            if (ranges.ndim != 1 or ranges.size < 2 or not np.isfinite(ranges).all()
                    or not all(math.isfinite(x) for x in scalars)
                    or sample.angle_increment <= 0 or sample.range_min <= 0
                    or sample.range_max <= sample.range_min
                    or np.any(ranges < sample.range_min) or np.any(ranges > sample.range_max)
                    or abs(sample.angle_min + (len(ranges)-1)*sample.angle_increment
                           - sample.angle_max) > 1e-5):
                return False
            angles = sample.angle_min + np.arange(ranges.size) * sample.angle_increment
            mask = np.abs(angles) <= np.pi / 2 + 1e-12
            if np.count_nonzero(mask) < 2:
                return False
        except (TypeError, ValueError, OverflowError):
            return False
        if not self._observe("scan", sample.header, ros_now, monotonic_now):
            return False
        self.scan = sample.header, ranges[mask].copy(), angles[mask].copy()
        return True

    def observe_speed(self, sample, ros_now, monotonic_now):
        if not isinstance(sample, SpeedSample) or not math.isfinite(sample.longitudinal_mps):
            return False
        if not self._observe("speed", sample.header, ros_now, monotonic_now):
            return False
        self.speed = sample
        return True

    def nominal(self, sample, ros_now, monotonic_now):
        if not isinstance(sample, (YawRateNominal, SteeringAngleNominal)) or sample.model != self.model:
            return PipelineDecision("nominal_model_mismatch")
        if self.model == "steering_tangent":
            if not isinstance(sample, SteeringAngleNominal):
                return PipelineDecision("nominal_type_mismatch")
            if not math.isfinite(sample.steering_angle_rad) or abs(sample.steering_angle_rad) > np.pi / 9 + 1e-12:
                return PipelineDecision("invalid_nominal")
            lateral = math.tan(sample.steering_angle_rad)
        else:
            if not isinstance(sample, YawRateNominal):
                return PipelineDecision("nominal_type_mismatch")
            lateral = sample.yaw_rate_rps
        if not all(math.isfinite(v) for v in (sample.acceleration_mps2, lateral)):
            return PipelineDecision("invalid_nominal")
        if not self._observe("nominal", sample.header, ros_now, monotonic_now):
            return PipelineDecision("nominal_stamp_rejected")
        for source in ("scan", "speed", "nominal"):
            if not self.leases[source].fresh(ros_now, monotonic_now):
                return PipelineDecision(source + "_unavailable_or_expired")
        headers = (self.scan[0], self.speed.header, sample.header)
        if max(h.seconds for h in headers) - min(h.seconds for h in headers) > self.max_skew + 1e-12:
            return PipelineDecision("source_skew")
        decision = self.kernel.evaluate(self.speed.longitudinal_mps, self.scan[1], self.scan[2],
                                        [sample.acceleration_mps2, lateral])
        if not decision.accepted:
            return PipelineDecision(decision.status, kernel=decision)
        command = SoftwareCommand(self.model, decision.command, min(headers, key=lambda h: h.seconds),
                                  self.scan[0], self.speed.header, sample.header)
        return PipelineDecision("accepted", command, decision)

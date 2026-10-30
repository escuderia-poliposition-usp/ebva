"""Typed ROS message boundary for the preserved yaw_rate CBF kernel."""
from copy import deepcopy
import math
from ebva_control_msgs.msg import CbfYawRateCommand
from .software import (CbfSoftwarePipeline, SourceStamp, ScanSample, SpeedSample,
                       YawRateNominal, PipelineDecision)


def stamp_ns(stamp):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError("invalid_stamp")
    result = stamp.sec * 1_000_000_000 + stamp.nanosec
    if result <= 0:
        raise ValueError("invalid_stamp")
    return result


class YawRatePort:
    """No TF, actuator mapping, speed integration, manual RC or cached heartbeat.

    Odometry acceptance qualifies only the consumed longitudinal velocity port.
    Pose, the remaining twist components, covariance and accuracy are not used.
    """
    def __init__(self, control_frame, velocity_parent_frame, model_reference_point,
                 scan_geometry, timeout_s=.2, future_tolerance_s=.02, max_skew_s=.08):
        for frame in (control_frame, velocity_parent_frame):
            if not isinstance(frame, str) or not frame or any(c.isspace() for c in frame):
                raise ValueError("Explicit nonempty frames required")
        if control_frame == velocity_parent_frame:
            raise ValueError("Velocity parent and body frames must differ")
        if model_reference_point != "rear_axle_center" or scan_geometry != "coincident_aligned":
            raise ValueError("Explicit rear axle/coincident aligned geometry contract required")
        if not all(math.isfinite(v) for v in (timeout_s, future_tolerance_s, max_skew_s)):
            raise ValueError("Finite time policy required")
        if not all(math.isfinite(v * 1e9) for v in (timeout_s, future_tolerance_s)):
            raise ValueError("Time policy cannot be represented in nanoseconds")
        self.control_frame, self.velocity_parent_frame = control_frame, velocity_parent_frame
        self.timeout_ns = round(timeout_s * 1e9)
        self.future_ns = round(future_tolerance_s * 1e9)
        self.pipeline = CbfSoftwarePipeline("yaw_rate", frame=control_frame, timeout=timeout_s,
            future_slack=future_tolerance_s, max_skew=max_skew_s)
        if self.timeout_ns <= 0:
            raise ValueError("Timeout must represent at least one nanosecond")
        self.scan_header = self.velocity_header = None
        self.velocity_child = None
        self.last_now_ns = None
        self.clock_reset_required = False

    def _time(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            return "clock_unavailable"
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.clock_reset_required = True
        self.last_now_ns = now_ns
        return "clock_reset_required" if self.clock_reset_required else None

    def _header(self, header, expected_frame, now_ns):
        reason = self._time(now_ns)
        if reason:
            return reason
        if header.frame_id != expected_frame:
            return "frame_mismatch"
        try:
            source_ns = stamp_ns(header.stamp)
        except ValueError as error:
            return str(error)
        if now_ns - source_ns >= self.timeout_ns:
            return "measurement_expired"
        if source_ns - now_ns > self.future_ns:
            return "measurement_in_future"
        return None

    def observe_scan(self, message, now_ns, monotonic_now):
        reason = self._header(message.header, self.control_frame, now_ns)
        if reason:
            return reason
        sample = ScanSample(SourceStamp(stamp_ns(message.header.stamp) / 1e9, message.header.frame_id),
            tuple(message.ranges), message.angle_min, message.angle_increment, message.angle_max,
            message.range_min, message.range_max)
        if not self.pipeline.observe_scan(sample, now_ns / 1e9, monotonic_now):
            return "scan_rejected"
        self.scan_header = deepcopy(message.header)
        return None

    def observe_velocity(self, message, now_ns, monotonic_now):
        reason = self._header(message.header, self.velocity_parent_frame, now_ns)
        if reason:
            return reason
        if message.child_frame_id != self.control_frame:
            return "velocity_child_frame_mismatch"
        value = message.twist.twist.linear.x
        if not math.isfinite(value):
            return "nonfinite_longitudinal_velocity"
        # SourceStamp.frame is the basis of velocity; the ROS parent header is
        # preserved separately, never relabelled in the output message.
        sample = SpeedSample(SourceStamp(stamp_ns(message.header.stamp) / 1e9,
                                          message.child_frame_id), value)
        if not self.pipeline.observe_speed(sample, now_ns / 1e9, monotonic_now):
            return "velocity_rejected"
        self.velocity_header = deepcopy(message.header)
        self.velocity_child = message.child_frame_id
        return None

    def nominal(self, message, now_ns, monotonic_now):
        reason = self._header(message.header, self.control_frame, now_ns)
        if reason:
            return None, PipelineDecision(reason)
        sample = YawRateNominal(SourceStamp(stamp_ns(message.header.stamp) / 1e9,
                                           message.header.frame_id), "yaw_rate",
                                message.acceleration_mps2, message.yaw_rate_rps)
        decision = self.pipeline.nominal(sample, now_ns / 1e9, monotonic_now)
        if decision.command is None:
            return None, decision
        output = CbfYawRateCommand()
        output.header = deepcopy(message.header)
        output.scan_header = deepcopy(self.scan_header)
        output.velocity_header = deepcopy(self.velocity_header)
        output.velocity_child_frame_id = self.velocity_child
        output.longitudinal_velocity_mps = float(self.pipeline.speed.longitudinal_mps)
        output.acceleration_mps2, output.yaw_rate_rps = decision.command.control
        return output, decision

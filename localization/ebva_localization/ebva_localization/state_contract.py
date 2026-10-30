"""Validation of a declared state port; no ROS I/O or coordinate conversion."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class PosePolicy:
    """All temporal numbers are composition choices, expressed in seconds."""
    parent_frame: str
    child_frame: str
    max_age_sec: float
    future_tolerance_sec: float
    quaternion_norm_tolerance: float = 1e-6
    covariance_tolerance: float = 1e-10

    def __post_init__(self):
        for value in (self.parent_frame, self.child_frame):
            if not isinstance(value, str) or not value or value.strip() != value or any(c.isspace() for c in value):
                raise ValueError("parent_frame and child_frame must be explicit nonempty frame names")
        if self.parent_frame == self.child_frame:
            raise ValueError("parent_frame and child_frame must identify different frames")
        for name in ("max_age_sec", "future_tolerance_sec", "quaternion_norm_tolerance", "covariance_tolerance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.max_age_sec <= 0:
            raise ValueError("max_age_sec must be explicitly positive")
        if self.quaternion_norm_tolerance >= 1:
            raise ValueError("quaternion_norm_tolerance must be less than one")
        if self.max_age_ns <= 0:
            raise ValueError("max_age_sec must be at least one nanosecond")

    @property
    def max_age_ns(self):
        return round(self.max_age_sec * 1_000_000_000)

    @property
    def future_tolerance_ns(self):
        return round(self.future_tolerance_sec * 1_000_000_000)


def stamp_nanoseconds(stamp):
    """Reject malformed/nil acquisition stamps rather than substituting reception."""
    if not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int):
        raise ValueError("invalid_stamp")
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid_stamp")
    result = stamp.sec * 1_000_000_000 + stamp.nanosec
    if result <= 0:
        raise ValueError("invalid_stamp")
    return result


def covariance_reason(values, tolerance):
    """Structural validity only; zero covariance remains zero with no accuracy claim."""
    try:
        matrix = np.asarray(values, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return "invalid_covariance"
    if matrix.shape != (36,) or not np.isfinite(matrix).all():
        return "invalid_covariance"
    matrix = matrix.reshape(6, 6)
    if np.max(np.abs(matrix - matrix.T)) > tolerance:
        return "asymmetric_covariance"
    # Use the symmetric part only for the numerical PSD check, never in output.
    try:
        eigenvalues = np.linalg.eigvalsh(matrix * 0.5 + matrix.T * 0.5)
    except np.linalg.LinAlgError:
        return "invalid_covariance"
    if not np.isfinite(eigenvalues).all() or eigenvalues[0] < -tolerance:
        return "indefinite_covariance"
    return None


class PoseGate:
    """Accept each strictly newer valid pose once.

    A ROS-clock regression latches rejection until the node/gate is recreated.
    There is no heartbeat or persistent cached pose publication. Consumers must
    still enforce their own freshness and silence policy.
    """
    def __init__(self, policy):
        self.policy = policy
        self.last_stamp_ns = None
        self.last_now_ns = None
        self.clock_reset_required = False

    def accept(self, message, now_ns):
        if not isinstance(now_ns, int) or now_ns <= 0:
            return "clock_unavailable"
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.clock_reset_required = True
        self.last_now_ns = now_ns
        if self.clock_reset_required:
            return "clock_reset_required"
        if message.header.frame_id != self.policy.parent_frame:
            return "parent_frame_mismatch"
        if message.child_frame_id != self.policy.child_frame:
            return "child_frame_mismatch"
        try:
            stamp_ns = stamp_nanoseconds(message.header.stamp)
        except ValueError as error:
            return str(error)
        if now_ns - stamp_ns >= self.policy.max_age_ns:
            return "measurement_expired"
        if stamp_ns - now_ns > self.policy.future_tolerance_ns:
            return "measurement_in_future"
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return "measurement_not_newer"
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            return "nonfinite_pose"
        norm = math.hypot(*values[3:])
        if abs(norm - 1.0) > self.policy.quaternion_norm_tolerance:
            return "nonunit_quaternion"
        reason = covariance_reason(message.pose.covariance, self.policy.covariance_tolerance)
        if reason:
            return reason
        self.last_stamp_ns = stamp_ns
        return None

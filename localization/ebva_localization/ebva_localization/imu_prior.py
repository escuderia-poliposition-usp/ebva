"""Research IMU prior: explicit SI measurements, pure numerical state and gates.

The translation method is the preserved sensor_fusion phase-2b prior: supplied
orientation, startup mean specific force, semi-implicit planar integration,
optional ZUPT and speed cap. This is not a general inertial navigation filter.
"""
from dataclasses import dataclass
import math

import numpy as np

from .state_contract import covariance_reason


def quaternion_rotation(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion_yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


@dataclass(frozen=True)
class ImuPriorPolicy:
    input_frame: str
    parent_frame: str
    child_frame: str
    orientation_reference_frame: str
    input_acceleration_units: str
    input_angular_velocity_units: str
    acceleration_content: str
    calibration_stationary_asserted: bool
    max_age_sec: float
    future_tolerance_sec: float
    max_integration_dt_sec: float
    calib_samples: int
    calibration_accel_tolerance_mps2: float
    calibration_gyro_threshold_rps: float
    calibration_orientation_tolerance_rad: float
    expected_specific_force_norm_mps2: float
    specific_force_norm_tolerance_mps2: float
    pose_covariance: tuple
    twist_covariance: tuple
    covariance_provenance: str
    zupt_enable: bool = True
    zupt_accel_thresh: float = 0.2
    zupt_gyro_thresh: float = 0.05
    max_speed: float = 4.0
    quaternion_norm_tolerance: float = 1e-6

    def __post_init__(self):
        for name in ("input_frame", "parent_frame", "child_frame", "orientation_reference_frame"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value or value.strip() != value or any(c.isspace() for c in value):
                raise ValueError(name + " must be an explicit frame")
        if self.input_frame != self.child_frame:
            raise ValueError("input frame must be the same body/origin; no extrinsic transform is implemented")
        if len({self.parent_frame, self.child_frame, self.orientation_reference_frame}) != 3:
            raise ValueError("startup-local parent, supplied orientation reference and body frames must differ")
        if self.input_acceleration_units != "m/s^2" or self.input_angular_velocity_units != "rad/s":
            raise ValueError("only explicitly declared SI input is supported")
        if self.acceleration_content != "specific_force":
            raise ValueError("specific_force is required; gravity-removed or ambiguous acceleration is unsupported")
        if self.calibration_stationary_asserted is not True:
            raise ValueError("composition/operator must assert startup stationarity")
        if type(self.calib_samples) is not int or self.calib_samples < 2:
            raise ValueError("calib_samples must be an integer >= 2")
        if type(self.zupt_enable) is not bool:
            raise ValueError("zupt_enable must be boolean")
        positive = ("max_age_sec", "max_integration_dt_sec",
                    "calibration_accel_tolerance_mps2", "calibration_gyro_threshold_rps",
                    "calibration_orientation_tolerance_rad", "expected_specific_force_norm_mps2",
                    "specific_force_norm_tolerance_mps2", "max_speed")
        nonnegative = ("future_tolerance_sec", "zupt_accel_thresh", "zupt_gyro_thresh",
                       "quaternion_norm_tolerance")
        for name in (*positive, *nonnegative):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0 or (name in positive and value == 0):
                raise ValueError(name + " must be finite with the declared sign")
        if round(self.max_age_sec*1e9) <= 0 or round(self.max_integration_dt_sec*1e9) <= 0:
            raise ValueError("time intervals must represent at least one nanosecond")
        if self.future_tolerance_sec >= self.max_age_sec:
            raise ValueError("future_tolerance_sec must be less than max_age_sec")
        if self.quaternion_norm_tolerance >= 1:
            raise ValueError("quaternion_norm_tolerance must be less than one")
        if self.covariance_provenance not in ("fixture", "research_assumption"):
            raise ValueError("covariance provenance must explicitly identify an unmeasured assumption")
        for name in ("pose_covariance", "twist_covariance"):
            values = tuple(getattr(self, name))
            if covariance_reason(values, 1e-10) or any(values[i*6+i] <= 0 for i in range(6)):
                raise ValueError(name + " must be explicit PSD covariance with positive diagonal")
            object.__setattr__(self, name, values)


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    frame: str
    quaternion: tuple
    acceleration: tuple
    angular_velocity: tuple
    orientation_available: bool = True
    acceleration_available: bool = True
    angular_velocity_available: bool = True


class ImuPrior:
    def __init__(self, policy):
        self.policy = policy
        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.yaw = 0.0
        self.yaw0 = None
        self.grav_bias = np.zeros(3)
        self.calib_accum = np.zeros(3)
        self.calib_count = 0
        self.calibration_first_force = None
        self.calibration_first_q = None
        self.calibrated = False
        self.last_stamp_ns = None
        self.integration_stamp_ns = None
        self.last_now_ns = None
        self.clock_reset_required = False
        self.last_reason = "waiting_for_measurement"

    def _reset_calibration(self):
        self.calib_accum[:] = 0
        self.calib_count = 0
        self.calibration_first_force = None
        self.calibration_first_q = None

    def _reject(self, reason):
        self.last_reason = reason
        self.integration_stamp_ns = None
        self.vel[:] = 0
        if not self.calibrated:
            self._reset_calibration()
        return None, reason

    def process(self, sample, now_ns):
        p = self.policy
        if type(now_ns) is not int or now_ns <= 0:
            return self._reject("clock_unavailable")
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.clock_reset_required = True
        self.last_now_ns = now_ns
        if self.clock_reset_required:
            return self._reject("clock_reset_required")
        if sample.frame != p.input_frame:
            return self._reject("input_frame_mismatch")
        if type(sample.stamp_ns) is not int or sample.stamp_ns <= 0:
            return self._reject("invalid_stamp")
        if self.last_stamp_ns is not None and sample.stamp_ns <= self.last_stamp_ns:
            # Even expired reordered data must not corrupt a valid trajectory.
            # It cannot renew the integration cursor or create output.
            self.last_reason = "measurement_not_newer"
            return None, self.last_reason
        if now_ns-sample.stamp_ns >= round(p.max_age_sec*1e9):
            return self._reject("measurement_expired")
        if sample.stamp_ns-now_ns > round(p.future_tolerance_sec*1e9):
            return self._reject("measurement_in_future")
        if not all((sample.orientation_available, sample.acceleration_available, sample.angular_velocity_available)):
            return self._reject("required_imu_component_unavailable")
        try:
            q = np.asarray(sample.quaternion, dtype=float)
            a = np.asarray(sample.acceleration, dtype=float)
            gyro = np.asarray(sample.angular_velocity, dtype=float)
            if q.shape != (4,) or a.shape != (3,) or gyro.shape != (3,):
                return self._reject("invalid_imu_shape")
            if not all(np.isfinite(v).all() for v in (q, a, gyro)):
                return self._reject("nonfinite_imu")
        except (TypeError, ValueError, OverflowError):
            return self._reject("invalid_imu_shape")
        if abs(math.hypot(*q)-1) > p.quaternion_norm_tolerance:
            return self._reject("nonunit_quaternion")
        # No state commit occurs until derived arithmetic is also finite.
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            rotation = quaternion_rotation(q)
            a_world = rotation @ a
        if not np.isfinite(a_world).all():
            return self._reject("nonfinite_derived_state")

        previous = self.last_stamp_ns
        gap = previous is not None and sample.stamp_ns-previous > round(p.max_integration_dt_sec*1e9)
        if not self.calibrated:
            if gap:
                self._reset_calibration()
            if np.linalg.norm(gyro) >= p.calibration_gyro_threshold_rps:
                return self._reject("calibration_angular_motion")
            if abs(np.linalg.norm(a)-p.expected_specific_force_norm_mps2) > p.specific_force_norm_tolerance_mps2:
                return self._reject("calibration_specific_force_mismatch")
            if self.calib_count:
                force_change = np.linalg.norm(a_world-self.calibration_first_force)
                angle_change = 2*math.acos(min(1.0, abs(float(np.dot(q, self.calibration_first_q)))))
                if force_change > p.calibration_accel_tolerance_mps2 or angle_change > p.calibration_orientation_tolerance_rad:
                    return self._reject("calibration_unstable")
            else:
                self.calibration_first_force = a_world.copy()
                self.calibration_first_q = q.copy()
            with np.errstate(over="ignore", invalid="ignore"):
                accum = self.calib_accum+a_world
            if not np.isfinite(accum).all():
                return self._reject("nonfinite_derived_state")
            self.calib_accum = accum
            self.last_stamp_ns = sample.stamp_ns
            self.calib_count += 1
            if self.calib_count < p.calib_samples:
                self.last_reason = "calibrating"
                return None, self.last_reason
            self.grav_bias = self.calib_accum/self.calib_count
            self.yaw0 = quaternion_yaw(q)
            self.calibrated = True
            self.integration_stamp_ns = sample.stamp_ns
            self.last_reason = "calibration_complete"
            return None, self.last_reason

        if gap or self.integration_stamp_ns is None:
            self.vel[:] = 0
            self.integration_stamp_ns = sample.stamp_ns
            self.last_stamp_ns = sample.stamp_ns
            self.last_reason = "integration_rebased_after_gap"
            return None, self.last_reason

        dt = (sample.stamp_ns-self.integration_stamp_ns)*1e-9
        # The strict newer cursor above ensures dt > 0.
        yaw = math.atan2(math.sin(quaternion_yaw(q)-self.yaw0),
                         math.cos(quaternion_yaw(q)-self.yaw0))
        a_lin = a_world-self.grav_bias
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        a_xy = np.array([c0*a_lin[0]+s0*a_lin[1], -s0*a_lin[0]+c0*a_lin[1]])
        vel = self.vel.copy()
        pos = self.pos.copy()
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            if p.zupt_enable and np.linalg.norm(a_xy) < p.zupt_accel_thresh and abs(gyro[2]) < p.zupt_gyro_thresh:
                vel[:] = 0
            else:
                vel += a_xy*dt
                speed = math.hypot(*vel)
                if speed > p.max_speed:
                    vel *= p.max_speed/speed
                pos += vel*dt
            c, s = math.cos(yaw), math.sin(yaw)
            body_velocity = np.array([c*vel[0]+s*vel[1], -s*vel[0]+c*vel[1]])
        if not all(np.isfinite(v).all() for v in (a_xy, vel, pos, body_velocity)):
            return self._reject("nonfinite_derived_state")
        self.vel, self.pos, self.yaw = vel, pos, yaw
        self.last_stamp_ns = sample.stamp_ns
        self.integration_stamp_ns = sample.stamp_ns
        self.last_reason = "measurement_integrated"
        return {"stamp_ns": sample.stamp_ns, "position": tuple(pos),
                "yaw": yaw, "body_velocity": tuple(body_velocity)}, self.last_reason

    def status(self, reason):
        return {
            "reason": reason, "calibrated": self.calibrated,
            "calibration_samples": self.calib_count,
            "clock_reset_required": self.clock_reset_required,
            "quality": "research_prior",
            "covariance_provenance": self.policy.covariance_provenance,
            "uncertainty_estimated": False, "covariance_propagated": False,
            "stationarity_verified": False,
            "stationarity_basis": "operator_assertion_and_observed_stability",
            "provides": ["relative_planar_pose", "planar_linear_velocity"],
            "not_estimated": ["angular_velocity", "uncertainty"],
            "publish_tf": False,
            "input_frame": self.policy.input_frame,
            "parent_frame": self.policy.parent_frame,
            "child_frame": self.policy.child_frame,
            "orientation_reference_frame": self.policy.orientation_reference_frame,
            "acceleration_content": self.policy.acceleration_content,
        }

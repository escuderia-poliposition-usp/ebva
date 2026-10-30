"""Two original measurements bound the fixed-zero-heading clearance model."""
import copy
import math
from ebva_control_contracts import MeasurementLease
from .core import StanleyFrenetCore

REFERENCE_CONTRACT = 'stanley_frenet_clearance_fixed_zero_v1'
STANLEY_CONTRACT_PARAMS = ('reference_frame_id', 'body_frame_id', 'model_point',
                           'reference_id', 'reference_contract')


class StanleyFrenetContract:
    def __init__(self, reference_frame_id, body_frame_id, model_point,
                 reference_id, reference_contract, observation_timeout=.2,
                 future_slack=.02, maximum_skew=.08):
        if (not reference_frame_id or not body_frame_id
                or reference_frame_id == body_frame_id):
            raise ValueError('Distinct explicit reference/body frames required')
        if model_point not in ('rear_axle', 'software_fixture_rear_axle'):
            raise ValueError('Explicit rear axle model point required')
        if not reference_id or reference_contract != REFERENCE_CONTRACT:
            raise ValueError('Explicit clearance/fixed-zero reference identity required')
        if (not math.isfinite(observation_timeout) or observation_timeout <= 0
                or not math.isfinite(future_slack)
                or not 0 <= future_slack < observation_timeout
                or not math.isfinite(maximum_skew)
                or not 0 <= maximum_skew < observation_timeout):
            raise ValueError('Finite timeout with smaller nonnegative future/skew bounds required')
        self.reference_frame_id = reference_frame_id
        self.body_frame_id, self.model_point = body_frame_id, model_point
        self.reference_id, self.reference_contract = reference_id, reference_contract
        self.maximum_skew = maximum_skew
        self.odom_lease = MeasurementLease(observation_timeout, reference_frame_id, future_slack)
        self.error_lease = MeasurementLease(observation_timeout, reference_frame_id, future_slack)
        self.core = StanleyFrenetCore()
        self.odometry = self.lateral_error_m = None
        self.last_output_stamp = None
        self.last_rejection = ''

    def observe_odometry(self, *, stamp, ros_now, monotonic_now, frame_id,
                         child_frame_id, quaternion, vx, vy):
        if child_frame_id != self.body_frame_id:
            self.last_rejection = 'body_frame_mismatch'
            return False
        if (len(quaternion) != 4
                or not all(math.isfinite(v) for v in (*quaternion, vx, vy))):
            self.last_rejection = 'nonfinite_odometry'
            return False
        norm = sum(v*v for v in quaternion)
        if not math.isfinite(norm) or abs(norm-1.) > 1.e-3:
            self.last_rejection = 'nonunit_quaternion'
            return False
        if not math.isfinite(math.hypot(vx, vy)):
            self.last_rejection = 'unrepresentable_velocity_norm'
            return False
        candidate = copy.copy(self.odom_lease)
        if not candidate.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'odometry_time_or_frame'
            return False
        qx, qy, qz, qw = quaternion
        yaw = math.atan2(2.*(qw*qz+qx*qy), 1.-2.*(qy*qy+qz*qz))
        self.odom_lease = candidate
        self.odometry = (vx, vy, yaw)
        self.last_rejection = ''
        return True

    def observe_error(self, *, stamp, ros_now, monotonic_now, frame_id,
                      reference_id, body_frame_id, model_point, contract_id,
                      lateral_error_m):
        if (reference_id != self.reference_id or body_frame_id != self.body_frame_id
                or model_point != self.model_point or contract_id != self.reference_contract):
            self.last_rejection = 'lateral_error_reference_contract'
            return False
        if not math.isfinite(lateral_error_m):
            self.last_rejection = 'nonfinite_lateral_error'
            return False
        candidate = copy.copy(self.error_lease)
        if not candidate.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'lateral_error_time_or_frame'
            return False
        self.error_lease = candidate
        self.lateral_error_m = lateral_error_m
        self.last_rejection = ''
        return True

    def command(self, ros_now, monotonic_now):
        if not self.odom_lease.fresh(ros_now, monotonic_now):
            self.last_rejection = 'odometry_missing_or_expired'
            return None
        if not self.error_lease.fresh(ros_now, monotonic_now):
            self.last_rejection = 'lateral_error_missing_or_expired'
            return None
        if abs(self.odom_lease.stamp-self.error_lease.stamp) > self.maximum_skew:
            self.last_rejection = 'measurement_skew'
            return None
        oldest = min(self.odom_lease.stamp, self.error_lease.stamp)
        if self.last_output_stamp is not None and oldest <= self.last_output_stamp:
            self.last_rejection = 'oldest_measurement_not_advanced'
            return None
        speed, steering = self.core.command(self.lateral_error_m, *self.odometry)
        if not all(math.isfinite(v) for v in (speed, steering)):
            self.last_rejection = 'nonfinite_command'
            return None
        self.last_output_stamp = oldest
        self.last_rejection = ''
        return speed, steering

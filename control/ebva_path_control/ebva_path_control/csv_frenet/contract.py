"""Explicit temporal/frame boundary for the CSV/Frenet reference model."""
import math
from ebva_control_contracts import MeasurementLease
from .core import CsvFrenetCore

REFERENCE_CONTRACT = 'closed_progress_si_v1'
MODEL_POINTS = ('rear_axle', 'software_fixture_rear_axle')
PROGRESS_ORIGIN = 'first_accepted_odometry_s0'
PROGRESS_DIRECTION = 'path_order'


class CsvFrenetContract:
    def __init__(self, s_ref, ey_ref, reference_frame_id, body_frame_id,
                 model_point, reference_contract, progress_origin,
                 progress_direction, observation_timeout=.2, future_slack=.02):
        if (not reference_frame_id or not body_frame_id
                or reference_frame_id == body_frame_id):
            raise ValueError('Distinct explicit odometry parent and body frames required')
        if model_point not in MODEL_POINTS:
            raise ValueError('Explicit rear axle model point or software fixture required')
        if reference_contract != REFERENCE_CONTRACT:
            raise ValueError('Explicit closed_progress_si_v1 reference contract required')
        if progress_origin != PROGRESS_ORIGIN or progress_direction != PROGRESS_DIRECTION:
            raise ValueError('Explicit progress origin and direction required')
        if not math.isfinite(future_slack) or future_slack < 0:
            raise ValueError('Future slack must be finite and nonnegative')
        self.core = CsvFrenetCore(s_ref, ey_ref)
        self.body_frame_id = body_frame_id
        self.lease = MeasurementLease(observation_timeout, reference_frame_id, future_slack)
        self.last_rejection = ''

    def accept(self, *, stamp, ros_now, monotonic_now, frame_id, child_frame_id,
               x, y, quaternion, vx, vy):
        if child_frame_id != self.body_frame_id:
            self.last_rejection = 'body_frame'
            return None
        values = (x, y, vx, vy, *quaternion)
        if len(quaternion) != 4 or not all(math.isfinite(v) for v in values):
            self.last_rejection = 'nonfinite_or_quaternion'
            return None
        qx, qy, qz, qw = quaternion
        norm_squared = sum(q*q for q in quaternion)
        if not math.isfinite(norm_squared) or abs(norm_squared-1.) > 1.e-3:
            self.last_rejection = 'nonunit_quaternion'
            return None
        speed = math.hypot(vx, vy)
        if not math.isfinite(speed):
            self.last_rejection = 'unrepresentable_speed_norm'
            return None
        if self.core.prev_x is not None:
            ds = math.hypot(x-self.core.prev_x, y-self.core.prev_y)
            if not math.isfinite(ds) or ds > self.core.s_max:
                self.last_rejection = 'progress_jump'
                return None
        if not self.lease.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'measurement_time_or_frame'
            return None
        yaw = math.atan2(2.*(qw*qz+qx*qy), 1.-2.*(qy*qy+qz*qz))
        self.core.update_odometry(x, y, yaw, speed)
        self.last_rejection = ''
        return self.core.command()

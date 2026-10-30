from ebva_path_control.resource_paths import resolve_reference
"""Pose/reference boundary for the waypoint Pure Pursuit model; no ROS I/O."""
import copy
import math
import numpy as np
from ebva_control_contracts import MeasurementLease
from .waypoint_follow import PurePursuitPlanner, quaternion_to_yaw
from types import SimpleNamespace

PATH_CONTRACT_PARAMS = ('path_frame_id', 'body_frame_id', 'model_point',
                        'path_contract', 'path_origin', 'path_direction', 'pose_contract')


def load_waypoint_reference(path):
    path = resolve_reference(path)
    return PurePursuitPlanner(np.loadtxt(path, delimiter=',', ndmin=2)).waypoints


class WaypointContract:
    def __init__(self, waypoints, path_frame_id, body_frame_id, model_point,
                 path_contract, path_origin, path_direction, pose_contract,
                 observation_timeout=.2, future_slack=.02):
        if not path_frame_id or not body_frame_id or path_frame_id == body_frame_id:
            raise ValueError('Distinct explicit reference and body frames required')
        if model_point not in ('rear_axle', 'software_fixture_rear_axle'):
            raise ValueError('Explicit model point required')
        if (path_contract != 'ordered_closed_xy_si_v1'
                or path_origin != 'shared_reference_frame'
                or path_direction != 'csv_order'
                or pose_contract != 'cartesian_body_pose_si_v1'):
            raise ValueError('Explicit XY SI/body/origin/order contracts required')
        if not math.isfinite(future_slack) or future_slack < 0:
            raise ValueError('Finite nonnegative future slack required')
        self.core = PurePursuitPlanner(waypoints)
        self.body_frame_id = body_frame_id
        self.lease = MeasurementLease(observation_timeout, path_frame_id, future_slack)
        self.last_rejection = ''

    def accept(self, *, stamp, ros_now, monotonic_now, frame_id, x, y, quaternion):
        if len(quaternion) != 4 or not all(math.isfinite(v) for v in (x, y, *quaternion)):
            self.last_rejection = 'nonfinite_pose'
            return None
        norm = sum(value*value for value in quaternion)
        if not math.isfinite(norm) or abs(norm-1.) > 1.e-3:
            self.last_rejection = 'nonunit_quaternion'
            return None
        candidate = copy.copy(self.lease)
        if not candidate.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'measurement_time_or_frame'
            return None
        q = SimpleNamespace(**dict(zip(('x','y','z','w'), quaternion)))
        try:
            result = self.core.command(x, y, quaternion_to_yaw(q))
        except (ArithmeticError, ValueError):
            self.last_rejection = 'numerical_geometry'
            return None
        self.lease = candidate
        self.last_rejection = ''
        return result

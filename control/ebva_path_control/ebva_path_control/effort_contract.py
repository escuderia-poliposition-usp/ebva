from ebva_path_control.resource_paths import resolve_reference
"""Atomic longitudinal effort / geometric steering boundaries, without ROS."""
import copy
import csv
import math
import numpy as np
from ebva_control_contracts import MeasurementLease
from .waypoint_debug.core import WaypointEffortCore
from .lidar_frenet.core import LidarFrenetCore

WAYPOINT_EFFORT_PARAMS = (
    'reference_frame_id', 'body_frame_id', 'model_point', 'reference_contract',
    'reference_origin', 'reference_direction', 'pose_contract')
LIDAR_EFFORT_PARAMS = ('scan_frame_id', 'scan_contract')


def load_effort_waypoints(path):
    path = resolve_reference(path)
    with open(path, newline='') as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not {'x_center', 'y_center'} <= set(reader.fieldnames):
            raise ValueError('Reference requires named x_center,y_center columns')
        try:
            points = np.array([[float(row['x_center']), float(row['y_center'])] for row in reader])
        except (TypeError, KeyError, ValueError) as error:
            raise ValueError('Finite XY reference required') from error
    return WaypointEffortCore(points).waypoints


def _lease(timeout, frame, future):
    if not math.isfinite(future) or future < 0:
        raise ValueError('Finite nonnegative future_slack required')
    return MeasurementLease(timeout, frame, future)


class WaypointEffortContract:
    def __init__(self, waypoints, reference_frame_id, body_frame_id, model_point,
                 reference_contract, reference_origin, reference_direction,
                 pose_contract, control_mode='carrot', observation_timeout=.2,
                 future_slack=.02):
        if not reference_frame_id or not body_frame_id or reference_frame_id == body_frame_id:
            raise ValueError('Distinct explicit reference/body frames required')
        if model_point not in ('rear_axle', 'software_fixture_rear_axle'):
            raise ValueError('Explicit rear axle model point required')
        if (reference_contract != 'ordered_xy_si_v1'
                or reference_origin != 'shared_reference_frame'
                or reference_direction != 'csv_order'
                or pose_contract != 'cartesian_body_pose_si_v1'):
            raise ValueError('Explicit ordered XY/body/origin/direction contracts required')
        self.core = WaypointEffortCore(waypoints, control_mode)
        self.body_frame_id = body_frame_id
        self.contract = 'waypoint_control_'+control_mode+'_v1'
        self.lease = _lease(observation_timeout, reference_frame_id, future_slack)
        self.last_rejection = ''

    def accept(self, *, stamp, ros_now, monotonic_now, frame_id, child_frame_id,
               x, y, quaternion):
        if child_frame_id != self.body_frame_id:
            self.last_rejection = 'body_frame_mismatch'
            return None
        if len(quaternion) != 4 or not all(math.isfinite(v) for v in (x, y, *quaternion)):
            self.last_rejection = 'nonfinite_pose'
            return None
        if abs(sum(v*v for v in quaternion)-1.) > 1.e-3:
            self.last_rejection = 'nonunit_quaternion'
            return None
        candidate = copy.copy(self.lease)
        if not candidate.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'measurement_time_or_frame'
            return None
        qx, qy, qz, qw = quaternion
        yaw = math.atan2(2.*(qw*qz+qx*qy), 1.-2.*(qy*qy+qz*qz))
        result = self.core.command(x, y, yaw)
        if result is None:
            self.last_rejection = 'target_not_ahead'
            return None
        if not all(math.isfinite(v) for v in result[:2]):
            self.last_rejection = 'nonfinite_command'
            return None
        self.lease = candidate
        self.last_rejection = ''
        return result


class LidarEffortContract:
    contract = 'lidar_frenet_pp_v1'

    def __init__(self, scan_frame_id, scan_contract, observation_timeout=.2, future_slack=.02):
        if scan_contract != 'planar_body_flu_ranges_m_v1':
            raise ValueError('Explicit planar FLU scan/ranges SI contract required')
        self.core = LidarFrenetCore()
        self.lease = _lease(observation_timeout, scan_frame_id, future_slack)
        self.last_rejection = ''

    def accept(self, scan, *, stamp, ros_now, monotonic_now, frame_id):
        candidate = copy.copy(self.lease)
        if not candidate.observe(stamp, ros_now, monotonic_now, frame_id):
            self.last_rejection = 'measurement_time_or_frame'
            return None
        n = len(scan.ranges)
        geometry = (scan.angle_min, scan.angle_max, scan.angle_increment,
                    scan.range_min, scan.range_max)
        if (n < 2 or not all(math.isfinite(v) for v in geometry)
                or scan.angle_increment <= 0 or not 0 <= scan.range_min < scan.range_max
                or abs(scan.angle_max-(scan.angle_min+(n-1)*scan.angle_increment)) > 1.e-6):
            self.last_rejection = 'invalid_scan_geometry'
            return None
        angles = scan.angle_min+np.arange(n)*scan.angle_increment
        consumed = (((angles > math.radians(70)) & (angles < math.radians(110)))
                    | ((angles > -math.radians(110)) & (angles < -math.radians(70))))
        ranges = np.asarray(scan.ranges)
        finite = np.isfinite(ranges)
        if np.any(consumed & finite & ((ranges < scan.range_min) | (ranges > scan.range_max))):
            self.last_rejection = 'consumed_range_out_of_bounds'
            return None
        result = self.core.command(scan)
        if result is None:
            self.last_rejection = 'missing_side_returns'
            return None
        if not all(math.isfinite(v) for v in result):
            self.last_rejection = 'nonfinite_command'
            return None
        self.lease = candidate
        self.last_rejection = ''
        return result

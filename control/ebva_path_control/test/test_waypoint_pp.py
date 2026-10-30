import hashlib
import json
import math
from pathlib import Path
import numpy as np
import pytest
from ebva_path_control.waypoint_pp import waypoint_follow as model
from ebva_path_control.waypoint_pp.contract import WaypointContract

ORACLE = json.loads((Path(__file__).parent/'data/waypoint-pp-regression.json').read_text())


def contract(**changes):
    params = dict(waypoints=ORACLE['fixture'], path_frame_id='fixture_odom',
                  body_frame_id='fixture_rear_axle_flu', model_point='software_fixture_rear_axle',
                  path_contract='ordered_closed_xy_si_v1', path_origin='shared_reference_frame',
                  path_direction='csv_order', pose_contract='cartesian_body_pose_si_v1')
    params.update(changes)
    return WaypointContract(**params)


def sample(**changes):
    values = dict(stamp=10., ros_now=10.01, monotonic_now=100., frame_id='fixture_odom',
                  x=.5, y=-.2, quaternion=(0.,0.,0.,1.))
    values.update(changes)
    return values






@pytest.mark.parametrize('case', ORACLE['cases'], ids=lambda case: case['label'])
def test_original_full_callback_targets_preserved(case):
    if case['reference'] == 'preserved_center':
        path = Path(__file__).parents[1]/'data/waypoint_pp/center.csv'
        assert hashlib.sha256(path.read_bytes()).hexdigest() == ORACLE['preserved_center_sha256']
        points = np.loadtxt(path, delimiter=',')
    else:
        points = ORACLE['fixture']
    core = model.PurePursuitPlanner(points)
    speed, delta = core.command(case['x'], case['y'], case['yaw'])
    assert (speed, delta) == pytest.approx((case['speed_target'], case['steering_angle']), abs=1.e-10)
    assert speed == pytest.approx(case['legacy_throttle']*20., abs=1.e-10)
    assert delta == pytest.approx(-case['legacy_steering'], abs=1.e-10)


def test_left_right_and_rotated_flu_sign_is_geometry_not_electrical_sign():
    core = model.PurePursuitPlanner(ORACLE['fixture'])
    assert core.command(.5, -.2, 0.)[1] > 0.
    assert core.command(.5, .2, 0.)[1] < 0.
    assert core.command(3.2, .5, math.pi/2)[1] > 0.
    assert core.command(2.8, .5, math.pi/2)[1] < 0.
    assert model.get_speed_by_curvature(0.)*.5 == 6.
    assert model.get_speed_by_curvature(2.)*.5 < 1.


@pytest.mark.parametrize('points', [[], [[0.,0.],[1.,0.],[1.,1.]],
    [[0.,0.],[1.,0.],[0.,0.]], [[0.,0.],[0.,0.],[1.,1.],[0.,0.]],
    [[0.,0.],[1.,float('nan')],[1.,1.],[0.,0.]],
    [[0.,0.],[1.,0.],[1.,1.],[0.,0.],[0.,0.]],
    [[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,0.,0.]],
    [[0.,0.],[1.e308,0.],[1.e308,1.e308],[0.,0.]]])
def test_invalid_reference_is_rejected_without_repairing_or_reordering(points):
    with pytest.raises(ValueError):
        model.PurePursuitPlanner(points)


def test_closed_duplicate_endpoint_allowed_but_duplicate_adjacent_defect_rejected():
    assert ORACLE['confirmed_defects']['duplicate_segment_original_nearest_point_nonfinite']
    core = model.PurePursuitPlanner(ORACLE['fixture'])
    assert np.array_equal(core.waypoints[0], core.waypoints[-1])
    assert not core.waypoints.flags.writeable


@pytest.mark.parametrize('change', [dict(path_frame_id=''), dict(body_frame_id=''),
    dict(body_frame_id='fixture_odom'), dict(model_point='base_link'),
    dict(path_contract=''), dict(path_origin='first_pose'), dict(path_direction='reverse'),
    dict(pose_contract=''), dict(future_slack=float('nan'))])
def test_composition_contracts_are_explicit(change):
    with pytest.raises(ValueError):
        contract(**change)


@pytest.mark.parametrize('change', [dict(stamp=0.), dict(stamp=9.2), dict(stamp=10.04),
    dict(frame_id='map'), dict(x=float('nan')), dict(quaternion=(0.,0.,0.,0.)),
    dict(quaternion=(0.,0.,float('nan'),1.)), dict(x=1.e308, y=1.e308)])
def test_invalid_source_does_not_commit_lease(change):
    policy = contract()
    assert policy.accept(**sample(**change)) is None
    assert policy.lease.stamp is None
    assert policy.last_rejection


def test_duplicate_reorder_paused_clock_gap_and_valid_recovery():
    policy = contract()
    assert policy.accept(**sample()) is not None
    deadline = policy.lease.deadline
    assert policy.accept(**sample(monotonic_now=100.19)) is None
    assert policy.accept(**sample(stamp=9.99, monotonic_now=100.19)) is None
    assert policy.lease.deadline == deadline
    assert not policy.lease.fresh(10.01,100.21)
    result = policy.accept(**sample(stamp=10.5, ros_now=10.51, monotonic_now=100.5, y=.2))
    assert result is not None and result[1] < 0.
    assert policy.lease.stamp == 10.5

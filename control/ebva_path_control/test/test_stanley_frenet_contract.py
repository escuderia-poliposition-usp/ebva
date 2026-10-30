"""Original-model equivalence and two-measurement validity, independent of ROS."""
import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from ebva_path_control.stanley_frenet.core import StanleyFrenetCore, wrap_angle
from ebva_path_control.stanley_frenet.contract import StanleyFrenetContract, REFERENCE_CONTRACT

DATA = json.loads((Path(__file__).parent/'data/stanley-frenet-regression.json').read_text())
PARAMS = dict(reference_frame_id='fixture_odom', body_frame_id='fixture_rear_axle',
              model_point='software_fixture_rear_axle', reference_id='fixture_straight_corridor',
              reference_contract=REFERENCE_CONTRACT)


def contract(**kwargs):
    return StanleyFrenetContract(**dict(PARAMS, **kwargs))


def odom(**kwargs):
    value = dict(stamp=10., ros_now=10.01, monotonic_now=100., frame_id='fixture_odom',
                 child_frame_id='fixture_rear_axle', quaternion=(0.,0.,0.,1.), vx=.6, vy=0.)
    value.update(kwargs)
    return value


def error(**kwargs):
    value = dict(stamp=10., ros_now=10.01, monotonic_now=100., frame_id='fixture_odom',
                 reference_id=PARAMS['reference_id'], body_frame_id=PARAMS['body_frame_id'],
                 model_point=PARAMS['model_point'], contract_id=REFERENCE_CONTRACT,
                 lateral_error_m=.3)
    value.update(kwargs)
    return value


@pytest.mark.parametrize('case', DATA['cases'])
def test_fifteen_pairs_equal_original_callback(case):
    result = StanleyFrenetCore().command(case['lateral_error_m'], case['vx'], case['vy'], case['yaw'])
    assert result == pytest.approx((case['speed'], case['steering_angle']), abs=1.e-14)
    assert result[0] == -.6 and abs(result[1]) <= .4




def test_lateral_error_without_odometry_never_produces_a_command():
    c = contract()
    assert c.observe_error(**error())
    for _ in range(12):
        assert c.command(10.01, 100.) is None
        assert c.last_rejection == 'odometry_missing_or_expired'


def test_reverse_feedback_sign_is_deliberately_not_a_different_model():
    c=StanleyFrenetCore()
    assert c.command(.3,.6,0.,0.) == c.command(.3,-.6,0.,0.)
    # A negative target alone does not prove reverse tracking is stable/correct.


@pytest.mark.parametrize('changes,reason', [
    ({'child_frame_id':'other'},'body_frame_mismatch'),
    ({'frame_id':'other'},'odometry_time_or_frame'),
    ({'quaternion':(0.,0.,0.,0.)},'nonunit_quaternion'),
    ({'quaternion':(0.,0.,0.,float('nan'))},'nonfinite_odometry'),
    ({'quaternion':(0.,0.,0.,1.e308)},'nonunit_quaternion'),
    ({'vx':float('inf')},'nonfinite_odometry'),
    ({'vx':1.7e308,'vy':1.7e308},'unrepresentable_velocity_norm'),
    ({'stamp':0.},'odometry_time_or_frame'),
    ({'stamp':9.},'odometry_time_or_frame'),
    ({'stamp':10.1},'odometry_time_or_frame'),
    ({'monotonic_now':float('nan')},'odometry_time_or_frame'),
])
def test_odometry_rejections_do_not_advance_lease(changes,reason):
    c=contract();assert not c.observe_odometry(**odom(**changes))
    assert c.last_rejection==reason and c.odom_lease.stamp is None


@pytest.mark.parametrize('changes,reason', [
    ({'body_frame_id':'other'},'lateral_error_reference_contract'),
    ({'model_point':'center_of_mass'},'lateral_error_reference_contract'),
    ({'reference_id':'other'},'lateral_error_reference_contract'),
    ({'contract_id':'cartesian_error'},'lateral_error_reference_contract'),
    ({'frame_id':'other'},'lateral_error_time_or_frame'),
    ({'lateral_error_m':float('nan')},'nonfinite_lateral_error'),
    ({'lateral_error_m':float('inf')},'nonfinite_lateral_error'),
    ({'stamp':9.},'lateral_error_time_or_frame'),
    ({'stamp':10.1},'lateral_error_time_or_frame'),
])
def test_lateral_error_rejections_do_not_advance_lease(changes,reason):
    c=contract();assert not c.observe_error(**error(**changes))
    assert c.last_rejection==reason and c.error_lease.stamp is None


@pytest.mark.parametrize('changes', [
    {'reference_frame_id':''},{'body_frame_id':'fixture_odom'},
    {'model_point':'center_of_mass'},{'reference_id':''},{'reference_contract':'unknown'},
    {'observation_timeout':0.},{'observation_timeout':float('inf')},
    {'future_slack':-.1},{'future_slack':.2},{'future_slack':float('nan')},
    {'maximum_skew':-.1},{'maximum_skew':.2},{'maximum_skew':float('inf')},
])
def test_explicit_finite_model_contract_required(changes):
    with pytest.raises(ValueError):contract(**changes)


def test_oldest_stamp_must_advance_and_neither_measurement_can_refresh_other():
    c=contract();assert c.observe_odometry(**odom());assert c.observe_error(**error())
    assert c.command(10.01,100.) is not None
    original=c.error_lease.deadline
    assert c.observe_odometry(**odom(stamp=10.05,ros_now=10.06,monotonic_now=100.05))
    assert c.command(10.06,100.05) is None
    assert c.last_rejection=='oldest_measurement_not_advanced'
    assert c.error_lease.deadline==original
    assert c.observe_error(**error(stamp=10.05,ros_now=10.06,monotonic_now=100.05))
    assert c.command(10.06,100.05) is not None
    assert c.last_output_stamp==10.05
    assert not c.observe_error(**error(stamp=10.05,ros_now=10.1,monotonic_now=100.1))
    assert c.command(10.06,100.3) is None  # Frozen ROS clock still expires locally.
    assert c.observe_odometry(**odom(stamp=10.5,ros_now=10.51,monotonic_now=100.5))
    assert c.observe_error(**error(stamp=10.5,ros_now=10.51,monotonic_now=100.5,lateral_error_m=-.3))
    assert c.command(10.51,100.5)[1]>0


@pytest.mark.parametrize('missing',['odometry','lateral_error'])
def test_independent_silence_expiration_and_recovery(missing):
    c=contract();assert c.observe_odometry(**odom());assert c.observe_error(**error())
    assert c.command(10.01,100.) is not None
    fresh=dict(stamp=10.21,ros_now=10.22,monotonic_now=100.21)
    if missing=='odometry':assert c.observe_error(**error(**fresh))
    else:assert c.observe_odometry(**odom(**fresh))
    assert c.command(10.22,100.21) is None
    assert c.last_rejection==missing+'_missing_or_expired'
    if missing=='odometry':assert c.observe_odometry(**odom(**fresh))
    else:assert c.observe_error(**error(**fresh))
    assert c.command(10.22,100.21) is not None


def test_skew_is_not_hidden_by_fresh_arrival_and_bounded_future_is_allowed():
    c=contract();assert c.observe_odometry(**odom());assert c.observe_error(**error(stamp=10.1,ros_now=10.11))
    assert c.command(10.11,100.) is None and c.last_rejection=='measurement_skew'
    assert c.observe_odometry(**odom(stamp=10.1,ros_now=10.09,monotonic_now=100.1))
    assert c.command(10.11,100.1) is not None


def test_ros_clock_regression_never_relabels_or_refreshes_old_measurement():
    c=contract();assert c.observe_odometry(**odom());assert c.observe_error(**error())
    assert c.command(10.01,100.) is not None
    deadline=c.odom_lease.deadline
    assert not c.observe_odometry(**odom(stamp=8.,ros_now=8.01,monotonic_now=101.))
    assert c.odom_lease.deadline==deadline
    assert c.command(8.01,101.) is None

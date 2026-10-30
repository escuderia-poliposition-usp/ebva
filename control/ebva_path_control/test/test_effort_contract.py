"""Pre-extraction oracles, pure guards and evidence-driven input removal."""
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from ebva_path_control.effort_contract import (
    WaypointEffortContract, LidarEffortContract, load_effort_waypoints)
from ebva_path_control.waypoint_debug.core import WaypointEffortCore
from ebva_path_control.lidar_frenet.core import LidarFrenetCore

DATA = json.loads((Path(__file__).parent/'data/effort-steering-regression.json').read_text())
PARAMS = dict(reference_frame_id='fixture_odom', body_frame_id='fixture_rear_axle',
              model_point='software_fixture_rear_axle', reference_contract='ordered_xy_si_v1',
              reference_origin='shared_reference_frame', reference_direction='csv_order',
              pose_contract='cartesian_body_pose_si_v1')


def waypoint(**kwargs):
    return WaypointEffortContract(DATA['waypoints'], **dict(PARAMS, **kwargs))


def pose(**kwargs):
    return dict(stamp=10., ros_now=10.01, monotonic_now=100., frame_id='fixture_odom',
                child_frame_id='fixture_rear_axle', x=0., y=-.2, quaternion=(0.,0.,0.,1.), **kwargs)


def scan(left=3., right=1.):
    angles = -math.pi+np.arange(361)*math.pi/180
    ranges = np.full(361, 4.)
    ranges[(angles>math.radians(70)) & (angles<math.radians(110))] = left
    ranges[(angles>-math.radians(110)) & (angles<-math.radians(70))] = right
    return SimpleNamespace(angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/180,
                           range_min=.1, range_max=20., ranges=ranges.tolist())


def lidar():
    return LidarEffortContract('fixture_lidar', 'planar_body_flu_ranges_m_v1')


def receive(contract, message, **kwargs):
    args = dict(stamp=10., ros_now=10.01, monotonic_now=100., frame_id='fixture_lidar')
    args.update(kwargs)
    return contract.accept(message, **args)


@pytest.mark.parametrize('case', DATA['waypoint_cases'])
def test_all_three_waypoint_modes_equal_original_callback(case):
    core = WaypointEffortCore(DATA['waypoints'], case['mode'])
    result = core.command(case['x'], case['y'], case['yaw'])
    if case['output'] is None:
        assert result is None
    else:
        assert result[:2] == pytest.approx((case['output']['longitudinal'], case['output']['steering']), abs=1.e-14)
        assert abs(result[1]) <= .41 and result[0] == .2


@pytest.mark.parametrize('case', DATA['lidar_cases'])
def test_lidar_matches_original_and_requires_no_odometry(case):
    result = LidarFrenetCore().command(scan(case['left'], case['right']))
    assert result == pytest.approx((case['output']['longitudinal'],case['output']['steering']),abs=1.e-14)




@pytest.mark.parametrize('changes,reason', [
    ({'child_frame_id':'other'},'body_frame_mismatch'),
    ({'frame_id':'other'},'measurement_time_or_frame'),
    ({'x':float('nan')},'nonfinite_pose'),
    ({'quaternion':(0.,0.,0.,0.)},'nonunit_quaternion'),
    ({'quaternion':(0.,0.,0.,float('inf'))},'nonfinite_pose'),
    ({'stamp':9.},'measurement_time_or_frame'),
    ({'stamp':10.1},'measurement_time_or_frame'),
    ({'stamp':0.},'measurement_time_or_frame'),
    ({'ros_now':float('nan')},'measurement_time_or_frame'),
    ({'x':5.,'y':0.},'target_not_ahead'),
])
def test_waypoint_rejects_without_advancing_lease(changes,reason):
    contract=waypoint();args=pose();args.update(changes)
    assert contract.accept(**args) is None
    assert contract.last_rejection==reason
    assert contract.lease.stamp is None


def test_waypoint_duplicate_reorder_clock_freeze_no_renewal_and_recovery():
    contract=waypoint()
    assert contract.accept(**pose()) is not None
    deadline=contract.lease.deadline
    for stamp in (10.,9.99):
        args=pose();args.update(stamp=stamp,monotonic_now=100.1)
        assert contract.accept(**args) is None and contract.lease.deadline==deadline
    assert not contract.lease.fresh(10.01,100.21)
    args=pose();args.update(stamp=10.5,ros_now=10.51,monotonic_now=100.5,y=.6)
    result=contract.accept(**args)
    assert result is not None and result[1] < 0


@pytest.mark.parametrize('field,value', [
    ('reference_contract','closed_xy'),('reference_direction','reverse'),
    ('reference_origin','unknown'),('pose_contract','unknown'),
    ('model_point','center_of_mass'),('body_frame_id','fixture_odom'),
    ('future_slack',float('nan')),('future_slack',-.1),
    ('observation_timeout',0.),('control_mode','other'),
])
def test_waypoint_requires_explicit_interpretation(field,value):
    with pytest.raises(ValueError):waypoint(**{field:value})


@pytest.mark.parametrize('body',[
    'x,y\n0,0\n1,0\n','x_center,y_center\n0,0\n',
    'x_center,y_center\n0,0\n1,nan\n','x_center,y_center\n0,0\nfoo,1\n'])
def test_bad_waypoint_reference_rejected(tmp_path,body):
    p=tmp_path/'ref.csv';p.write_text(body)
    with pytest.raises(ValueError):load_effort_waypoints(p)


def test_waypoint_named_resource_order_and_duplicate_points_are_preserved(tmp_path):
    p=tmp_path/'ref.csv';p.write_text('y_center,x_center,metadata\n0,0,a\n0,0,b\n1,2,c\n')
    np.testing.assert_array_equal(load_effort_waypoints(p),[[0,0],[0,0],[2,1]])


@pytest.mark.parametrize('mutation,reason',[
    (lambda s:setattr(s,'angle_increment',0.),'invalid_scan_geometry'),
    (lambda s:setattr(s,'angle_max',2.),'invalid_scan_geometry'),
    (lambda s:setattr(s,'range_max',float('inf')),'invalid_scan_geometry'),
    (lambda s:setattr(s,'ranges',[]),'invalid_scan_geometry'),
    (lambda s:s.ranges.__setitem__(270,-1.),'consumed_range_out_of_bounds'),
    (lambda s:s.ranges.__setitem__(270,21.),'consumed_range_out_of_bounds'),
    (lambda s:setattr(s,'ranges',[float('nan')]*361),'missing_side_returns'),
])
def test_lidar_rejects_bad_consumed_data_without_renewal(mutation,reason):
    c=lidar();s=scan();mutation(s)
    assert receive(c,s) is None and c.last_rejection==reason and c.lease.stamp is None


def test_lidar_original_nonfinite_filter_and_unconsumed_data_preserved():
    original=receive(lidar(),scan())
    s=scan();s.ranges[270]=float('nan');s.ranges[269]=float('inf');s.ranges[0]=-100.
    assert receive(lidar(),s)==pytest.approx(original)


def test_lidar_missing_one_side_does_not_invent_centering_command():
    s=scan()
    angles=s.angle_min+np.arange(len(s.ranges))*s.angle_increment
    for i,angle in enumerate(angles):
        if math.radians(70)<angle<math.radians(110):s.ranges[i]=float('inf')
    assert receive(lidar(),s) is None


@pytest.mark.parametrize('changes',[
    {'frame_id':'other'},{'stamp':9.},{'stamp':10.1},{'stamp':float('inf')},
    {'monotonic_now':float('nan')}])
def test_lidar_time_frame_rejections(changes):
    c=lidar()
    assert receive(c,scan(),**changes) is None and c.lease.stamp is None


def test_lidar_no_duplicate_renewal_and_fresh_recovery():
    c=lidar();assert receive(c,scan())[0]==.4
    deadline=c.lease.deadline
    assert receive(c,scan(),monotonic_now=100.1) is None and c.lease.deadline==deadline
    assert not c.lease.fresh(10.01,100.21)
    assert receive(c,scan(1.,3.),stamp=10.5,ros_now=10.51,monotonic_now=100.5)[1]<0


@pytest.mark.parametrize('params',[{'scan_frame_id':''},{'scan_contract':'unknown'},
    {'future_slack':float('inf')},{'observation_timeout':-.1}])
def test_lidar_requires_explicit_contract(params):
    base=dict(scan_frame_id='fixture_lidar',scan_contract='planar_body_flu_ranges_m_v1')
    base.update(params)
    with pytest.raises(ValueError):LidarEffortContract(**base)

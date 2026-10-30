import math
from types import SimpleNamespace
import numpy as np
import pytest
from ebva_path_control.geometry import quaternion_to_yaw
from ebva_path_control.references import load_frenet_reference
from ebva_path_control.lidar_frenet.pure_pursuit_frenet import PurePursuitFrenet
from ebva_path_control.waypoint_debug.pp_debug_controller_node import PPDebugController


@pytest.mark.parametrize("angle", [-math.pi, -1.2, 0, 0.7, math.pi])
def test_shared_quaternion_geometry(angle):
    q = SimpleNamespace(x=0., y=0., z=math.sin(angle/2), w=math.cos(angle/2))
    assert quaternion_to_yaw(q) == pytest.approx(angle)


def test_curvature_changes_with_side_and_rotation():
    straight = PurePursuitFrenet(np.array([[0.,0.],[2.,0.],[4.,0.]]), 2.)
    left = PurePursuitFrenet(np.array([[0.,0.],[2.,1.],[4.,1.]]), 2.)
    right = PurePursuitFrenet(np.array([[0.,0.],[2.,-1.],[4.,-1.]]), 2.)
    assert straight.compute_control(0,0,0) == 0
    assert left.compute_control(0,0,0) == pytest.approx(.5)
    assert right.compute_control(0,0,0) == pytest.approx(-.5)
    rotated = PurePursuitFrenet(np.array([[0.,0.],[-1.,2.],[-1.,4.]]), 2.)
    assert rotated.compute_control(0,0,math.pi/2) == pytest.approx(.5)


def test_three_waypoint_modes_keep_distinct_laws():
    controller = SimpleNamespace(k_heading=1., k_stanley=1., wheelbase=.33, lookahead_distance=2.)
    assert PPDebugController.carrot_control(controller, 2, 0, 0) == 0
    assert PPDebugController.carrot_control(controller, 0, 2, 0) == pytest.approx(math.pi/2)
    assert PPDebugController.pure_pursuit_control(controller, 1) == pytest.approx(math.atan(.165))
    assert PPDebugController.stanley_control(controller, 2, 0, 0, 1) == pytest.approx(math.pi/4)


def test_reference_uses_names_and_rejects_original_schema(tmp_path):
    csv = tmp_path/'ref.csv'
    csv.write_text('time,e_y,y_center_Ld\n0,.1,.2\n1,.2,.3\n')
    with pytest.raises(ValueError, match='requires named s'):
        load_frenet_reference(csv)
    csv.write_text('e_y,s,time\n.1,0,8\n.2,3,9\n')
    s, ey = load_frenet_reference(csv)
    np.testing.assert_allclose(s,[0,3]); np.testing.assert_allclose(ey,[.1,.2])


@pytest.mark.parametrize('body', ['s,e_y\n0,0\n0,.1\n','s,e_y\n0,0\n1,nan\n','s,e_y\n0,0\n'])
def test_reference_rejects_undefined_interpolation(tmp_path, body):
    csv=tmp_path/'ref.csv'; csv.write_text(body)
    with pytest.raises(ValueError): load_frenet_reference(csv)

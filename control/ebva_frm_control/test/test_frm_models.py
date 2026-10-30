from types import SimpleNamespace
import pytest
from ebva_frm_control.point_core import FrmPointCore
from ebva_frm_control.scan_average import ControleNode as Averaged


def fake_controller(cls):
    result=SimpleNamespace(e1_1=0.,e2_1=0.,int_e1=0.,int_e2=0.,distance_front_1=0.)
    result.get_logger=lambda:SimpleNamespace(info=lambda *args:None)
    steering=[];throttle=[]
    result.steer_publisher=SimpleNamespace(publish=lambda msg:steering.append(msg.data))
    result.throttle_publisher=SimpleNamespace(publish=lambda msg:throttle.append(msg.data))
    if cls is Averaged:
        result.media_lidar=lambda *args:Averaged.media_lidar(result,*args)
    return result,steering,throttle


def test_symmetric_average_has_no_lateral_command_and_keeps_state():
    cls=Averaged
    node,steering,throttle=fake_controller(cls)
    scan=SimpleNamespace(ranges=[10.]*1080)
    cls.lidar_callback(node,scan)
    assert steering==[0.]
    assert throttle==pytest.approx([.2])
    cls.lidar_callback(node,scan)
    assert throttle[-1]==pytest.approx(.2)
    assert node.distance_front_1==10.


def test_averaging_caps_outliers():
    assert Averaged.media_lidar(None,5,10,[20.]*10)==10.


def test_point_core_symmetric_scan_retains_original_longitudinal_recurrence():
    core=FrmPointCore()
    assert core.step([10.]*1080)==pytest.approx((0.,.4))
    assert core.step([10.]*1080)==pytest.approx((0.,.3))
    assert core.distance_front_1==10.

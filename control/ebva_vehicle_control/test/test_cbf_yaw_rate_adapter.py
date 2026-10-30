"""Directed model projection, explicit artificial conversion and source lease regressions."""
from types import SimpleNamespace
from unittest.mock import patch
import math
import pytest
from rclpy.time import Time
from std_msgs.msg import Header
from ebva_control_msgs.msg import CbfYawRateCommand
from ebva_control_contracts import MeasurementLease
from ebva_vehicle_control.cbf_yaw_rate_mapping import CbfYawRateMapping
from ebva_vehicle_control.cbf_yaw_rate_adapter import CbfYawRateAdapter,canonical_seconds


def mapping():
    return CbfYawRateMapping(100.,-.2,.33,.1,.24,3.,1./6.,math.pi/6,.5,600.,.4,.6)


@pytest.mark.parametrize("speed,omega",[(1.,.1),(1.,-.1),(-1.,.1),(0.,.1),(1e-10,-.1)])
def test_projection_retains_signed_speed_floor_and_original_kinematic_inversion(speed,omega):
    out=mapping().convert(.1,omega,speed)
    effective=max(abs(speed),.1)*(-1 if speed<0 else 1)
    angle=max(-math.pi/6,min(math.pi/6,math.atan(.33*omega/effective)))
    assert out.erpm==10.
    assert out.servo==pytest.approx(.5-.2*angle)


def test_positive_acceleration_cap_is_not_a_speed_integrator():
    assert mapping().convert(1.,0.,1.).erpm==24.
    assert mapping().convert(-1.,0.,1.).erpm==-100.
    assert mapping().convert(.1,0.,1.)==mapping().convert(.1,0.,1.)


@pytest.mark.parametrize("a,w,v",[(float("nan"),0.,1.),(0.,float("inf"),1.),(0.,0.,float("nan")),(4.,0.,1.),(0.,.2,1.)])
def test_nonfinite_or_out_of_model_domain_is_rejected(a,w,v):
    with pytest.raises(ValueError):mapping().convert(a,w,v)


def header(t,frame):
    h=Header();ns=round(t*1e9);h.stamp.sec,h.stamp.nanosec=divmod(ns,1_000_000_000);h.frame_id=frame;return h


def fake():
    return SimpleNamespace(expected_frame="body",skew=.08,
        source_leases={"scan":MeasurementLease(.2,"body"),"velocity":MeasurementLease(.2,"odom")},
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.)))


def message():
    m=CbfYawRateCommand();m.header=header(10.,"body");m.scan_header=header(9.99,"body")
    m.velocity_header=header(9.98,"odom");m.velocity_child_frame_id="body";return m


def test_oldest_original_parent_header_bounds_output_and_paused_clock_cannot_renew_sources():
    node=fake();m=message()
    assert CbfYawRateAdapter.accept_contract(node,m)
    with patch('ebva_vehicle_control.cbf_yaw_rate_adapter.time.monotonic',return_value=100.):
        assert CbfYawRateAdapter.sources_valid(node,m)
    assert CbfYawRateAdapter.output_header(node,m)==m.velocity_header
    m.header=header(10.01,"body")
    with patch('ebva_vehicle_control.cbf_yaw_rate_adapter.time.monotonic',return_value=100.19):
        assert not CbfYawRateAdapter.sources_valid(node,m)


@pytest.mark.parametrize("field,stamp,frame",[("scan_header",9.,"body"),("scan_header",10.,"wrong"),("velocity_header",10.,"wrong"),("velocity_header",10.1,"odom")])
def test_invalid_secondary_source_never_qualifies(field,stamp,frame):
    m=message();setattr(m,field,header(stamp,frame))
    assert not CbfYawRateAdapter.sources_valid(fake(),m)


def test_noncanonical_time_and_wrong_twist_body_are_rejected_before_conversion():
    m=message();m.scan_header.stamp.nanosec=1_000_000_000
    assert not CbfYawRateAdapter.accept_contract(fake(),m)
    m=message();m.velocity_child_frame_id="camera"
    assert not CbfYawRateAdapter.accept_contract(fake(),m)

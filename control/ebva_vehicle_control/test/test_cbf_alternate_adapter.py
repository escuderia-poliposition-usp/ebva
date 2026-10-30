"""Distinct model domains and identical original source lease semantics."""
import math
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from rclpy.time import Time
from std_msgs.msg import Header
from ebva_control_msgs.msg import CbfYawRateCommand,CbfYawRateMarginCommand,CbfSteeringTangentCommand
from ebva_control_contracts import MeasurementLease
from ebva_vehicle_control.cbf_alternate_adapter import CbfYawRateMarginAdapter,CbfSteeringTangentAdapter
from ebva_vehicle_control.cbf_alternate_mapping import CbfSteeringTangentMapping

def mapping():return CbfSteeringTangentMapping(100.,-.2,.24,3.,.4,math.pi/9,.5,600.,.4,.6)

@pytest.mark.parametrize("tangent",[0.,.1,-.1,math.tan(math.pi/9),-math.tan(math.pi/9)])
def test_tangent_uses_atan_inside_declared_twenty_degree_domain(tangent):
    angle=max(-math.pi/9,min(math.pi/9,math.atan(tangent)))
    assert mapping().convert(.1,tangent,1.).servo==pytest.approx(.5-.2*angle)
    assert mapping().convert(.1,tangent,-1.)==mapping().convert(.1,tangent,0.)

def test_acceleration_is_capped_not_integrated_as_velocity():
    assert mapping().convert(.8,0.,1.).erpm==24.
    assert mapping().convert(-1.,0.,1.).erpm==-100.

@pytest.mark.parametrize("a,t,v",[(float('nan'),0.,1.),(0.,float('inf'),1.),(4.,0.,1.),(0.,.41,1.),(0.,0.,float('nan'))])
def test_invalid_domain_not_reinterpreted(a,t,v):
    with pytest.raises(ValueError):mapping().convert(a,t,v)

def test_types_cannot_silently_select_another_model():
    assert len({CbfYawRateMarginAdapter.input_type,CbfSteeringTangentAdapter.input_type,CbfYawRateCommand})==3
    assert CbfSteeringTangentAdapter.input_topic!=CbfYawRateMarginAdapter.input_topic

def header(t,frame):
    h=Header();h.stamp.sec,h.stamp.nanosec=divmod(round(t*1e9),10**9);h.frame_id=frame;return h

@pytest.mark.parametrize("cls,msgtype",[(CbfYawRateMarginAdapter,CbfYawRateMarginCommand),(CbfSteeringTangentAdapter,CbfSteeringTangentCommand)])
def test_each_model_keeps_oldest_parent_header_and_independent_monotonic_expiry(cls,msgtype):
    fake=SimpleNamespace(expected_frame='body',skew=.08,source_leases={'scan':MeasurementLease(.2,'body'),'velocity':MeasurementLease(.2,'odom')},get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.)))
    msg=msgtype();msg.header=header(10.,'body');msg.scan_header=header(9.99,'body');msg.velocity_header=header(9.98,'odom');msg.velocity_child_frame_id='body'
    assert cls.accept_contract(fake,msg)
    with patch('ebva_vehicle_control.cbf_yaw_rate_adapter.time.monotonic',return_value=100.):assert cls.sources_valid(fake,msg)
    assert cls.output_header(fake,msg)==msg.velocity_header
    msg.header=header(10.01,'body')
    with patch('ebva_vehicle_control.cbf_yaw_rate_adapter.time.monotonic',return_value=100.19):assert not cls.sources_valid(fake,msg)

@pytest.mark.parametrize("value",[.4,-.4])
def test_admissible_model_tangent_is_rejected_when_angle_cannot_represent_it(value):
    with pytest.raises(ValueError,match="cannot be represented"):
        mapping().convert(-1.045994497,value,.2)

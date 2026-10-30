from types import SimpleNamespace
from unittest.mock import patch
import pytest
from rclpy.time import Time
from std_msgs.msg import Header
from ebva_control_msgs.msg import FtgPdCommand
from ebva_control_contracts import MeasurementLease
from ebva_vehicle_control.ftg_pd_adapter import FtgPdAdapter


def header(t):
    h=Header();ns=round(t*1e9);h.stamp.sec,h.stamp.nanosec=divmod(ns,1_000_000_000);h.frame_id="fixture";return h


def fake():
    return SimpleNamespace(expected_frame="fixture",skew=.08,
        feedback_lease=MeasurementLease(.2,"fixture"),get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.)))


def test_oldest_header_and_feedback_lifetime_survive_new_scans_with_paused_clock():
    n=fake();m=FtgPdCommand();m.header=header(10.);m.feedback_header=header(9.98)
    with patch('ebva_vehicle_control.ftg_pd_adapter.time.monotonic',return_value=100.):
        assert FtgPdAdapter.sources_valid(n,m)
    assert FtgPdAdapter.output_header(n,m)==m.feedback_header
    m.header=header(10.01)
    with patch('ebva_vehicle_control.ftg_pd_adapter.time.monotonic',return_value=100.19):
        assert not FtgPdAdapter.sources_valid(n,m)  # Remaining .18s, not a renewed .2s.


@pytest.mark.parametrize("stamp,frame",[(9.,"fixture"),(9.9,"fixture"),(10.1,"fixture"),(10.,"wrong"),(0.,"fixture")])
def test_bad_secondary_source_cannot_qualify_pair(stamp,frame):
    n=fake();m=FtgPdCommand();m.header=header(10.);m.feedback_header=header(stamp);m.feedback_header.frame_id=frame
    assert not FtgPdAdapter.sources_valid(n,m)

from types import SimpleNamespace
import pytest
from ebva_lane_keeping.confidence_fsm import ConfidenceFSM,LaneState
from ebva_lane_keeping.lane_boundaries_adapter import LaneBoundariesAdapter
from ebva_lane_keeping.lane_keeping_core import LaneKeepingCore,V1_COMPAT


def line(y):
    return [SimpleNamespace(x=x,y=y) for x in [.1,.3,.5,.7,1.]]


def test_geometry_center_offset_single_line_and_invalid():
    adapter=LaneBoundariesAdapter()
    detected,lateral,heading,curvature,mode=adapter.compute_ex(line(-.25),line(.25))
    assert detected and mode=='both'
    assert lateral==pytest.approx(0,abs=1e-12)
    assert heading==pytest.approx(0,abs=1e-12) and curvature==pytest.approx(0,abs=1e-12)
    assert adapter.compute_ex(line(-.15),line(.35))[1]==pytest.approx(.1)
    assert adapter.compute_ex(line(-.25),[])[4]=='left'
    assert not adapter.compute_ex([],[])[0]


def test_tracking_loss_latch_and_reset():
    core=LaneKeepingCore()
    assert core.step([],[],.1)['speed']==0
    outputs=[core.step(line(-.25),line(.25),.1) for _ in range(5)]
    assert outputs[-1]['state'] is LaneState.TRACKING and outputs[-1]['speed']>0
    assert outputs[-1]['steering']==pytest.approx(0,abs=1e-12)
    lost=core.step([],[],.6)
    assert lost['speed']==0 and lost['latched']
    assert core.step(line(-.25),line(.25),.1)['speed']==0
    core.reset()
    assert not core.latched and core.speed_cmd==0


def test_watchdog_and_stale_measured_speed():
    core=LaneKeepingCore()
    for _ in range(4):core.step(line(-.25),line(.25),.1)
    core.set_measured_speed(3.)
    assert core._speed_for_stanley()==3.
    core.measured_age=1.
    assert core._speed_for_stanley()==core.speed_cmd
    core.force_stop()
    assert core.latched and core.speed_cmd==0


def test_v1_parameters_preserve_two_line_requirement():
    core=LaneKeepingCore(**V1_COMPAT)
    assert not core.step(line(-.25),[],.3)['detected']
    assert core.step(line(-.25),[],.3)['speed']==0

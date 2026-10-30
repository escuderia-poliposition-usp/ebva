"""Lease, frame and sign checks without ROS I/O."""
from types import SimpleNamespace
import pytest
from ebva_control_contracts import MeasurementLease
from ebva_lane_keeping.lane_boundaries_adapter import LaneBoundariesAdapter
from ebva_lane_keeping.stanley_controller import StanleyController


@pytest.mark.parametrize("stamp,now,frame",[
    (0.,1.,"ground"),(.5,1.,"ground"),(1.03,1.,"ground"),(1.,1.,"wrong"),
    (float("nan"),1.,"ground")])
def test_invalid_measurement_cannot_start_lease(stamp,now,frame):
    lease=MeasurementLease(.2,"ground")
    assert not lease.observe(stamp,now,10.,frame)
    assert not lease.fresh(now,10.)


def test_original_age_and_paused_clock_do_not_renew_lease():
    lease=MeasurementLease(.2,"ground")
    assert lease.observe(1.,1.1,10.,"ground")
    assert lease.fresh(1.1,10.05)
    assert not lease.observe(1.,1.1,10.09,"ground")
    assert not lease.observe(.99,1.1,10.09,"ground")
    assert not lease.fresh(1.1,10.11)
    assert not lease.fresh(.9,10.05)
    lease.clear()  # Explicit reset/restart for a new replay epoch.
    assert lease.observe(.5,.5,11.,"ground")


@pytest.mark.parametrize("offset,curve",[(.05,0.),(0.,.1)])
def test_ground_frd_geometry_produces_flu_steering_with_mirror_symmetry(offset,curve):
    values=[]
    for sign in (1.,-1.):
        def side(y):
            return [SimpleNamespace(x=x,y=y+sign*(offset+curve*x*x)) for x in (.1,.3,.5,.7,1.)]
        adapter=LaneBoundariesAdapter(lookahead_x=0.)
        valid,lateral,heading,curvature=adapter.compute(side(-.25),side(.25))
        assert valid
        values.append(StanleyController(k_soft=.5).compute_steering(lateral,heading,.4,curvature))
    assert values[0] < 0 < values[1]  # Ground path right/left.
    assert values[0] == pytest.approx(-values[1],abs=1e-10)

import json
import math
from pathlib import Path
import numpy as np
import pytest
from ebva_mapless_control.ftg_pd_core import FtgPdCore, Scan


def fixture(goal=0.,distance=3.):
    angles=np.linspace(-3*math.pi/4,3*math.pi/4,1081)
    values=2.+(distance-2.)*np.exp(-.5*((angles-math.radians(goal))/.3)**2)
    return Scan(values.tolist(),float(angles[0]),float(angles[-1]),float(angles[1]-angles[0]),10.)


def test_full_original_callback_sequence_preserves_both_outputs_and_state():
    d=json.loads((Path(__file__).parent/"data/ftg-pd-golden.json").read_text());core=FtgPdCore()
    for c in d["cases"]:
        assert core.step(fixture(c["goal_degrees"],c["distance"]),c["feedback"])==pytest.approx((c["steering"],c["throttle"]),abs=1e-10)
        assert core.thetaOld==pytest.approx(c["limited_reference"],abs=1e-10)
        assert core.speedOld==pytest.approx(c["unsaturated_speed_state"],abs=1e-10)


def test_left_mask_does_not_wrap_to_last_scan_beam():
    core=FtgPdCore();scan=fixture();scan.ranges=[2.,.1]+[3.]*1079
    core.mask_disparities(0,scan)
    assert scan.ranges[-1]==3.  # Original callback changed this unrelated tail to2.


def test_pd_effort_is_not_reference_angle_or_normalized_command():
    core=FtgPdCore();core.steering_msg.data=.8;core.steer_command(-math.pi/3)
    assert core.steering_output==pytest.approx(-2.2027531067521533)


def test_throttle_ramp_is_per_sample_and_state_precedes_final_saturation():
    core=FtgPdCore();values=[]
    for _ in range(12):
        speed=core.setSafeMaxSpeed(10.,0.);core.publish_control(0.,speed)
        values.append(core.throttle_output)
    assert values[:3]==pytest.approx([.02,.04,.06])
    assert values[-1]==.12 and core.speedOld>.12


def test_input_data_is_not_mutated_by_core_processing():
    core=FtgPdCore();scan=fixture();before=list(scan.ranges);core.step(scan,0.)
    assert scan.ranges==before

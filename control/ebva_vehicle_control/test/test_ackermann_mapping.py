"""SI conversion with invented fixture gains, not vehicle calibration."""
from dataclasses import replace
import pytest
from ebva_vehicle_control.ackermann import AckermannMapping


def fixture():
    return AckermannMapping(100., -.2, .5, 2., .5, 600., .4, .6)


def test_zero_reverse_sign_and_stationary_steering():
    m=fixture()
    assert m.convert(0.,0.).erpm == 0 and m.convert(0.,0.).servo == .5
    assert m.convert(-1.,.2).erpm == -100.
    assert m.convert(1.,.2).servo == pytest.approx(.46)
    assert m.convert(1.,-.2).servo == pytest.approx(.54)
    assert m.convert(0.,.2).erpm == 0.


def test_input_and_output_saturations_are_explicit():
    assert fixture().convert(20.,10.).erpm == 200.
    assert fixture().convert(20.,10.).servo == .4
    assert replace(fixture(),speed_gain_erpm_per_mps=-1000.).convert(2.,-.5).erpm == -600.


@pytest.mark.parametrize("speed,angle",[(float("nan"),0.),(0.,float("inf")),(-float("inf"),0.)])
def test_nonfinite_commands_are_rejected(speed,angle):
    with pytest.raises(ValueError):fixture().convert(speed,angle)


@pytest.mark.parametrize("changes",[
    {"speed_gain_erpm_per_mps":0.},{"steering_gain_servo_per_rad":0.},
    {"servo_center":.7},{"erpm_limit":-1.},{"max_speed_mps":0.},
    {"max_steering_rad":2.},{"servo_min":float("nan")}])
def test_unusable_mapping_is_rejected(changes):
    with pytest.raises(ValueError):replace(fixture(),**changes)

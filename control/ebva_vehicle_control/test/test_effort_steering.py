"""Numerical domain checks distinguish an angle from an arbitrary effort."""
from dataclasses import replace
import pytest
from ebva_vehicle_control.effort_steering import EffortSteeringMapping

def fixture():
    return EffortSteeringMapping(100., -.2, .5, 1., 1.2, 600., .4, .6)

def test_signed_pair_and_geometric_angle_without_speed_inference():
    value=fixture().convert(-.2, .25)
    assert value.erpm == -20. and value.servo == pytest.approx(.45)
    assert fixture().convert(.4, -.25).servo == pytest.approx(.55)

@pytest.mark.parametrize("pair", [(float("nan"), 0.), (0., float("inf")), (1.01, 0.), (0., 1.21)])
def test_invalid_or_outside_source_domain_rejected(pair):
    with pytest.raises(ValueError): fixture().convert(*pair)

def test_electrical_bounds_saturate_after_valid_source():
    mapping=replace(fixture(), longitudinal_gain_erpm_per_effort=1000., steering_gain_servo_per_rad=-2.)
    assert mapping.convert(1., 1.).erpm == 600.
    assert mapping.convert(1., 1.).servo == .4

@pytest.mark.parametrize("changes", [{"servo_center":.8}, {"max_steering_rad":2.},
    {"longitudinal_gain_erpm_per_effort":0.}, {"max_longitudinal_effort":0.}])
def test_invalid_explicit_map_rejected(changes):
    with pytest.raises(ValueError): replace(fixture(), **changes)

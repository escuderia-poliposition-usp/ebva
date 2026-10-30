from dataclasses import replace
import pytest
from ebva_vehicle_control.frm_effort import FrmEffortMapping


def fixture():return FrmEffortMapping(100.,-.1,.5,600.,.4,.6)


def test_explicit_artificial_gain_preserves_zero_and_signed_effort():
    assert fixture().convert(0.,0.).erpm==0.
    assert fixture().convert(-.1,.5).erpm==pytest.approx(-10.)
    assert fixture().convert(-.1,.5).servo==pytest.approx(.45)


def test_output_saturates_at_declared_backend_limits():
    m=replace(fixture(),longitudinal_gain_erpm_per_effort=1000.,steering_gain_servo_per_effort=-2.)
    assert m.convert(1.,1.).erpm==600. and m.convert(1.,1.).servo==.4
    assert m.convert(-1.,-1.).erpm==-600. and m.convert(-1.,-1.).servo==.6


@pytest.mark.parametrize("longitudinal,steering",[(float('nan'),0.),(0.,float('inf')),(1.01,0.),(0.,-1.01)])
def test_invalid_normalized_domain_is_not_silently_reinterpreted(longitudinal,steering):
    with pytest.raises(ValueError):fixture().convert(longitudinal,steering)


@pytest.mark.parametrize("changes",[{"servo_center":.7},{"longitudinal_gain_erpm_per_effort":0.},{"erpm_limit":-1.}])
def test_invalid_or_missing_calibration_bounds_are_rejected(changes):
    with pytest.raises(ValueError):replace(fixture(),**changes)

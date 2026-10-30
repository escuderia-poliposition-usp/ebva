"""Only the supervisor contracts can use its explicit synthetic mapping."""
import math
import pytest
from ebva_vehicle_control.supervisor_effort import CONTRACTS, SupervisorEffortMapping


def mapping():
    return SupervisorEffortMapping(100., -.1, .5, 600., .4, .6)


def test_explicit_supervisor_fixture_mapping():
    command = mapping().convert(.25, .8)
    assert command.erpm == 25. and command.servo == pytest.approx(.42)
    reverse = mapping().convert(-.5, 0.)
    assert reverse.erpm == -50. and reverse.servo == .5
    assert "frm_point_sample_v1" not in CONTRACTS
    assert len(CONTRACTS) == 5


@pytest.mark.parametrize("longitudinal,steering", [(math.nan, 0.), (0., math.inf), (1.1, 0.), (0., -1.1)])
def test_nonfinite_or_out_of_scale_effort_rejected(longitudinal, steering):
    with pytest.raises(ValueError):
        mapping().convert(longitudinal, steering)


def test_invalid_mapping_cannot_be_selected():
    with pytest.raises(ValueError):
        SupervisorEffortMapping(0., -.1, .5, 600., .4, .6)

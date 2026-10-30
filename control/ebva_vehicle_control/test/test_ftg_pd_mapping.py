import pytest
from ebva_vehicle_control.ftg_pd_mapping import FtgPdMapping


def fixture():return FtgPdMapping(1000.,-.1,.5,600.,.4,.6)


def test_distinct_ftg_throttle_and_unbounded_pd_domains_are_preserved():
    assert fixture().convert(0.,0.).erpm==0.
    assert fixture().convert(.12,-2.20275).erpm==pytest.approx(120.)
    assert fixture().convert(.12,-2.20275).servo==.6
    assert fixture().convert(.1,.2).servo==pytest.approx(.48)


@pytest.mark.parametrize("throttle,effort",[(-.1,0.),(.13,0.),(float('nan'),0.),(0.,float('inf'))])
def test_invalid_numeric_domain_is_not_reinterpreted_as_physical_speed(throttle,effort):
    with pytest.raises(ValueError):fixture().convert(throttle,effort)

import json
from pathlib import Path
import pytest
from ebva_frm_control.point_core import FrmPointCore, INDICES


def test_original_callback_golden_sequence_matches_both_values_and_memory():
    fixture=json.loads((Path(__file__).parent/"data/point-sample-golden.json").read_text())
    core=FrmPointCore()
    for case in fixture["cases"]:
        ranges=[1.]*1080
        for index,value in zip(INDICES,case["points_front_left90_right90_left45_right45"]):ranges[index]=value
        assert core.step(ranges)==pytest.approx((case["steering"],case["longitudinal"]),abs=1e-12)


def test_sample_difference_is_not_velocity_or_time_derivative():
    core=FrmPointCore()
    assert core.step([5.]*1080)[1]==pytest.approx(.2)
    assert core.step([5.]*1080)[1]==pytest.approx(.15)
    assert core.step([0.]*1080)[1]==pytest.approx(-.05)
    assert core.step([0.]*1080)[1]==0.


def test_gap_reset_restarts_history_without_changing_the_law():
    core=FrmPointCore();core.step([10.]*1080);core.reset()
    assert core.step([1.]*1080)==pytest.approx((0.,.04))

"""Numeric selection and lifecycle tests without ROS or device imports."""
from dataclasses import replace
import math
import pytest
from ebva_supervisor.core import Classification, Settings, Supervisor, METHODS


def sample(stamp=1_000_000_000, **values):
    return replace(Classification(stamp, "model", True, "straight", "middle",
                                  1., 0., 1., 0., 0.), **values)


def engine(method="supervised", **values):
    return Supervisor(Settings(method=method, expected_frame="model", **values))


def take(core, value=None, now=1_000_000_000, receipt=10.):
    value = value or sample(now)
    assert core.observe(value, now, receipt)
    return core.command(now, receipt)


@pytest.mark.parametrize("method,track,position,steering,longitudinal", [
    ("pi_straight", "straight", "left", 1., 1.),
    ("p_curve", "left_curve", "middle", -.5, .6),
    ("p_intersection", "intersection", "left", .8, .25),
    ("wall_recovery", "walls", "middle", 0., -.5),
    ("straight_rule", "straight", "middle", 0., 1.),
    ("straight_rule", "left_curve", "middle", 0., 0.),
])
def test_fixed_methods_use_original_laws_and_stamps(method, track, position, steering, longitudinal):
    command = take(engine(method), sample(track_type=track, position=position))
    assert command.steering == pytest.approx(steering)
    assert command.longitudinal == pytest.approx(longitudinal)
    assert command.contract == "supervisor_" + method + "_v1"
    assert command.stamp_ns == 1_000_000_000 and command.frame_id == "model"


@pytest.mark.parametrize("track,selected", [("straight", "pi_straight"),
    ("left_curve", "p_curve"), ("right_curve", "p_curve"),
    ("intersection", "p_intersection"), ("walls", "wall_recovery")])
def test_supervised_dispatch(track, selected):
    assert take(engine(), sample(track_type=track)).selected_method == selected


def test_original_pi_per_cycle_accumulation_and_invalid_reset():
    core = engine("pi_straight")
    value = sample(position="left", position_probability=.5, position_entropy=.2)
    first = take(core, value)
    second = core.command(1_050_000_000, 10.05)
    assert first.steering == pytest.approx(.44)
    assert second.steering == pytest.approx(.48)
    assert first.stamp_ns == second.stamp_ns == value.stamp_ns
    assert not core.observe(replace(value, valid=False), 1_050_000_000, 10.05)
    assert core.command(1_050_000_000, 10.05) is None
    assert take(core, replace(value, stamp_ns=1_100_000_000), 1_100_000_000, 10.1).steering == pytest.approx(.44)


def test_original_curve_feature_and_uncertainty_slowdown():
    result = take(engine("p_curve"), sample(track_type="right_curve", curvature_feature=2.))
    assert result.steering == pytest.approx(.6)
    assert result.longitudinal == pytest.approx(.3)
    result = take(engine("p_curve"), sample(track_type="left_curve", track_entropy=.7))
    assert result.steering == pytest.approx(-.5) and result.longitudinal == pytest.approx(.3)


def test_wall_replays_atomic_history_backwards_and_resets_on_exit():
    core = engine()
    first = take(core, sample(track_type="right_curve"))
    second = take(core, sample(1_050_000_000, track_type="intersection", position="left"),
                  1_050_000_000, 10.05)
    wall = take(core, sample(1_100_000_000, track_type="walls"), 1_100_000_000, 10.1)
    assert (wall.steering, wall.longitudinal) == pytest.approx((second.steering, -second.longitudinal))
    wall = core.command(1_150_000_000, 10.15)
    assert (wall.steering, wall.longitudinal) == pytest.approx((first.steering, -first.longitudinal))
    take(core, sample(1_160_000_000), 1_160_000_000, 10.16)
    assert not core.wall.is_recovering


@pytest.mark.parametrize("changes,reason", [
    ({"valid": False}, "classification_invalid"), ({"frame_id": "other"}, "frame_mismatch"),
    ({"track_type": "unknown"}, "unknown_classification"), ({"position": "unknown"}, "unknown_classification"),
    ({"track_probability": 1.1}, "invalid_probability"), ({"position_probability": -.1}, "invalid_probability"),
    ({"track_entropy": -.1}, "invalid_entropy"), ({"position_entropy": -.1}, "invalid_entropy"),
    ({"curvature_feature": math.nan}, "nonfinite_classification"),
    ({"track_probability": math.inf}, "nonfinite_classification"),
    ({"stamp_ns": 0}, "invalid_stamp"),
])
def test_invalid_samples_stop_and_cannot_renew_state(changes, reason):
    core = engine()
    take(core)
    assert not core.observe(replace(sample(1_050_000_000), **changes), 1_050_000_000, 10.05)
    assert core.reason == reason and core.command(1_050_000_000, 10.05) is None


@pytest.mark.parametrize("stamp,now", [(1_000_000_000, 1_200_000_000), (1_000_000_001, 1_000_000_000)])
def test_stale_and_future_rejected(stamp, now):
    core = engine()
    assert not core.observe(sample(stamp), now, 10.)
    assert core.reason == "stale_or_future_classification"


def test_repeated_and_reordered_stamps_do_not_rearm_the_source():
    core = engine()
    take(core)
    for stamp in (1_000_000_000, 999_999_999):
        assert not core.observe(sample(stamp), 1_050_000_000, 10.05)
        assert core.command(1_050_000_000, 10.05) is None
    assert take(core, sample(1_100_000_000), 1_100_000_000, 10.1)


@pytest.mark.parametrize("ros_now,local_now", [(1_200_000_000, 10.1), (1_050_000_000, 10.21),
    (999_999_999, 10.05), (1_050_000_000, 9.9)])
def test_both_ros_age_and_monotonic_receipt_are_required(ros_now, local_now):
    core = engine()
    take(core)
    assert core.command(ros_now, local_now) is None
    assert core.reason == "classification_expired"


def test_method_change_requires_new_classification():
    core = engine()
    take(core)
    core.configure(replace(core.settings, method="straight_rule"))
    assert core.command(1_050_000_000, 10.05) is None
    assert take(core, sample(1_100_000_000), 1_100_000_000, 10.1).selected_method == "straight_rule"


@pytest.mark.parametrize("changes", [{"method": "fuzzy"}, {"source_timeout_s": 0.},
    {"control_rate_hz": 10.}, {"kp_reta": math.nan}, {"expected_frame": ""}])
def test_invalid_configuration_rejected(changes):
    with pytest.raises(ValueError):
        Settings(**changes)


def test_invalid_fresh_observation_requires_a_newer_stamp_to_recover():
    core = engine()
    take(core)
    invalid = sample(1_050_000_000, valid=False)
    assert not core.observe(invalid, 1_050_000_000, 10.05)
    assert not core.observe(replace(invalid, valid=True), 1_060_000_000, 10.06)
    assert core.reason == "duplicate_or_reordered_classification"
    assert take(core, sample(1_100_000_000), 1_100_000_000, 10.1)

import json
import math
from pathlib import Path
import pytest
from ebva_path_control.csv_frenet.core import CsvFrenetCore
from ebva_path_control.csv_frenet.contract import CsvFrenetContract


def contract(**overrides):
    params = dict(s_ref=[0., 1.2, 2.4, 3.6, 4.8, 6.],
                  ey_ref=[0., 0., .2, 0., -.2, 0.],
                  reference_frame_id='fixture_map', body_frame_id='fixture_rear_axle_flu',
                  model_point='software_fixture_rear_axle', reference_contract='closed_progress_si_v1',
                  progress_origin='first_accepted_odometry_s0', progress_direction='path_order')
    params.update(overrides)
    return CsvFrenetContract(**params)


def sample(**overrides):
    values = dict(stamp=10., ros_now=10.01, monotonic_now=100.,
                  frame_id='fixture_map', child_frame_id='fixture_rear_axle_flu',
                  x=0., y=0., quaternion=(0., 0., 0., 1.), vx=.5, vy=0.)
    values.update(overrides)
    return values


def test_original_23_case_replay_preserves_internal_targets_and_progress():
    golden = json.loads((Path(__file__).parent/'data/csv-frenet-golden.json').read_text())
    core = CsvFrenetCore(golden['reference']['s'], golden['reference']['e_y'])
    for case in golden['cases']:
        core.update_odometry(case['x'], case['y'], case['yaw'], math.hypot(case['vx'], case['vy']))
        assert core.s == pytest.approx(case['progress'], abs=1.e-12)
        assert core.command() == pytest.approx((case['speed_target'], case['steering']), abs=1.e-12)
        # Removal of the old test scale is a new boundary decision, not a
        # measured conversion between actuator throttle and physical velocity.
        assert case['speed_target'] == 2.*case['legacy_throttle']
    assert golden['old_stale_timer_reproduction']['age_not_checked']
    assert golden['old_stale_timer_reproduction']['still_published']['throttle'] > 0.


@pytest.mark.parametrize('s,ey', [([1., 2.], [0., 0.]), ([0., 1.], [0., 0.]),
                                 ([0., 2., 1.], [0., 0., 0.]),
                                 ([0., 2.], [0., float('nan')])])
def test_reference_model_assumptions_are_checked(s, ey):
    with pytest.raises(ValueError):
        CsvFrenetCore(s, ey)


@pytest.mark.parametrize('change', [dict(reference_frame_id=''), dict(body_frame_id=''),
                                   dict(body_frame_id='fixture_map'), dict(model_point='base_link'),
                                   dict(reference_contract=''), dict(progress_origin=''),
                                   dict(progress_direction='reverse'), dict(future_slack=-1.)])
def test_composition_must_declare_model_reference_and_frames(change):
    with pytest.raises(ValueError):
        contract(**change)


@pytest.mark.parametrize('change', [dict(stamp=0.), dict(stamp=9.2), dict(stamp=10.04),
                                   dict(frame_id='other_map'), dict(child_frame_id='base_link'),
                                   dict(x=float('nan')), dict(vx=float('inf')),
                                   dict(vx=1.7e308, vy=1.7e308),
                                   dict(quaternion=(0., 0., 0., 0.)),
                                   dict(quaternion=(0., 0., float('nan'), 1.))])
def test_invalid_measurement_does_not_initialize_progress_or_renew_lease(change):
    policy = contract()
    assert policy.accept(**sample(**change)) is None
    assert policy.core.prev_x is None and policy.lease.stamp is None
    assert policy.last_rejection


def test_duplicate_out_of_order_and_paused_clock_cannot_renew():
    policy = contract()
    assert policy.accept(**sample()) == pytest.approx((.5, 0.))
    deadline = policy.lease.deadline
    assert policy.accept(**sample(monotonic_now=100.19)) is None
    assert policy.accept(**sample(stamp=9.99, monotonic_now=100.19)) is None
    assert policy.lease.deadline == deadline
    assert not policy.lease.fresh(10.01, 100.21)


def test_recovery_after_gap_keeps_reference_alignment_and_only_uses_accepted_positions():
    policy = contract()
    policy.accept(**sample())
    assert policy.accept(**sample(stamp=10.1, ros_now=10.11, x=5., child_frame_id='wrong')) is None
    assert policy.accept(**sample(stamp=10.4, ros_now=10.41, monotonic_now=100.4, x=.4)) is not None
    assert policy.core.s == pytest.approx(.4)
    assert policy.core.prev_x == .4


def test_position_step_exceeding_lap_is_rejected_without_changing_state():
    policy = contract()
    policy.accept(**sample())
    assert policy.accept(**sample(stamp=10.05, ros_now=10.06, x=20.)) is None
    assert policy.last_rejection == 'progress_jump'
    assert policy.core.s == 0. and policy.lease.stamp == 10.


def test_reverse_displacement_still_advances_scalar_reference_as_original():
    core = CsvFrenetCore([0., 2., 6.], [0., .2, 0.])
    core.update_odometry(0., 0., 0., 0.)
    core.update_odometry(-.4, 0., math.pi, 0.)
    assert core.s == pytest.approx(.4)
    assert core.command()[0] > 0.  # This model is not a signed progress follower.


def test_curve_sides_and_output_limits_are_model_targets_not_actuator_calibration():
    core = CsvFrenetCore([0., 1.2, 2.4], [0., 10., 0.])
    core.update_odometry(0., 0., 0., 0.)
    assert core.command() == pytest.approx((.4, .4))
    opposite = CsvFrenetCore([0., 1.2, 2.4], [0., -10., 0.])
    opposite.update_odometry(0., 0., 0., 0.)
    assert opposite.command() == pytest.approx((.4, -.4))

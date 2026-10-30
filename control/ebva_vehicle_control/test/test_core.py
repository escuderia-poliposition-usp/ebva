"""Temporal/failure cases with controlled time; no ROS or hardware."""
from dataclasses import replace
import math
import pytest
from ebva_vehicle_control.core import (
    Authority, AuthorityPolicy, Actuation, RcSample, Mode, Backend, Mapping, Selection)
from ebva_vehicle_control.framing import RcFramer


def rc(throttle=1500., steering=1500., valid=True, rf=0):
    return RcSample(throttle, steering, valid, rf, "test")


def ready():
    authority = Authority()
    authority.autonomy_enabled = True
    authority.receive_rc(rc(), 10.)
    authority.receive_autonomy(Actuation(300., .53), 10.)
    return authority


def test_disabled_start_and_enable_needs_both_inputs():
    a = Authority()
    assert a.decide(1).mode == Mode.STOP
    a.autonomy_enabled = True
    a.receive_autonomy(Actuation(300., .5), 1.)
    assert a.decide(1.).mode == Mode.STOP
    a.receive_rc(rc(), 1.)
    assert a.decide(1.).mode == Mode.AUTONOMY


def test_manual_wins_and_neutral_automatically_returns_with_valid_inputs():
    a = ready()
    a.receive_rc(rc(1505., 1700.), 10.01)
    assert a.decide(10.01).mode == Mode.MANUAL
    a.receive_rc(rc(), 10.02)
    assert a.decide(10.02).mode == Mode.AUTONOMY


def test_manual_does_not_depend_on_autonomy_enable_or_freshness():
    a = ready()
    a.autonomy_enabled = False
    a.receive_rc(rc(1700.), 11.)
    assert a.decide(11.).mode == Mode.MANUAL
    a.receive_autonomy(Actuation(float("nan"), .5), 11.)
    assert a.decide(11.).mode == Mode.MANUAL


def test_neutral_never_refreshes_an_expired_autonomy():
    a = ready()
    a.receive_rc(rc(), 10.21)
    assert a.decide(10.21).reason == "autonomy_invalid_or_expired"


@pytest.mark.parametrize("sample", [rc(valid=False), rc(rf=2), rc(float("nan")), rc(0.), rc(2500.)])
def test_invalid_or_lost_rc_blocks_even_fresh_autonomy(sample):
    a = ready()
    a.receive_rc(sample, 10.)
    assert a.decide(10.).mode == Mode.STOP


def test_expiry_boundary_and_recovery_require_new_data():
    a = ready()
    assert a.decide(10.199).mode == Mode.AUTONOMY
    assert a.decide(10.201).mode == Mode.STOP
    a.receive_autonomy(Actuation(300., .5), 10.21)
    assert a.decide(10.21).mode == Mode.STOP
    a.receive_rc(rc(), 10.21)
    assert a.decide(10.21).mode == Mode.AUTONOMY


def test_manual_hysteresis_prevents_jitter_resuming_autonomy():
    a = ready()
    a.receive_rc(rc(1601.), 10.01)
    assert a.decide(10.01).mode == Mode.MANUAL
    a.receive_rc(rc(1590.), 10.02)
    assert a.decide(10.02).mode == Mode.MANUAL
    a.receive_rc(rc(1580.), 10.03)
    assert a.decide(10.03).mode == Mode.AUTONOMY


def test_clock_reversal_expires_both_sources():
    a = ready()
    a.decide(10.01)
    assert a.decide(9.).reason == "clock_reset"
    a.receive_rc(rc(), 9.)
    assert a.decide(9.).mode == Mode.STOP


def test_steering_only_manual_has_zero_motor_despite_center_jitter():
    command = Mapping().manual(rc(1505., 1700.))
    assert command.erpm == 0. and command.servo == .6


def test_mapping_sign_limits_and_selected_legacy_formula():
    mapping = Mapping(erpm_limit=4000., servo_min=.15, servo_max=.85)
    assert mapping.manual(rc(1601., 1700.)).erpm == pytest.approx(-808.)
    assert mapping.manual(rc(1601., 1700.)).servo == pytest.approx(.675)
    assert mapping.manual(rc(2000.)).erpm == -4000.
    assert mapping.manual(rc(1000.)).erpm == 4000.
    assert Mapping().manual(rc(2000., 2000.)) == Actuation(-600., .6)


@pytest.mark.parametrize("command", [Actuation(float("nan"), .5), Actuation(0., math.inf), Actuation(0., 1.1)])
def test_invalid_autonomy_never_reaches_backend(command):
    a = ready()
    a.receive_autonomy(command, 10.)
    assert a.decide(10.).mode == Mode.STOP
    with pytest.raises(ValueError):
        Mapping().bounded(command)


def test_backend_needs_fresh_neutral_rc_to_arm():
    backend = Backend()
    assert not backend.enable(True, 1.)
    backend.receive(Selection(Mode.STOP, "waiting"), 1.)
    assert not backend.enable(True, 1.)
    backend.receive(Selection(Mode.MANUAL, "manual", rc(1700.), rc_age=0), 1.)
    assert not backend.enable(True, 1.)
    backend.receive(Selection(Mode.STOP, "disabled", rc(), rc_age=0), 1.)
    assert backend.enable(True, 1.)
    assert backend.output(1.)[0].erpm == 0.


def test_backend_independent_lease_and_source_age_preserve_stop_steering():
    b = Backend()
    selected = Selection(Mode.AUTONOMY, "auto", rc(), Actuation(400., .55), .15, .15)
    b.receive(selected, 1.)
    assert b.enable(True, 1.)
    assert b.output(1.01)[0] == Actuation(400., .55)
    assert b.output(1.06)[0] == Actuation(0., .55)
    assert b.output(1.21)[1] == "selection_expired"


def test_dead_authority_and_disable_zero_the_motor():
    b = Backend()
    b.receive(Selection(Mode.AUTONOMY, "auto", rc(), Actuation(300., .5), 0, 0), 1.)
    assert b.enable(True, 1.)
    assert b.output(1.01)[0].erpm == 300.
    assert b.output(1.21)[0].erpm == 0.
    b.receive(Selection(Mode.AUTONOMY, "auto", rc(), Actuation(300., .5), 0, 0), 2.)
    b.enable(False, 2.)
    assert b.output(2.)[0].erpm == 0.


def test_serial_fragments_do_not_replay_partial_or_malformed_frames():
    f = RcFramer()
    assert f.feed(b"1500,") == []
    assert f.feed(b"1490\n")[0][:3] == (True, 1500., 1490.)
    for raw in [b"nan,1500\n", b"0,1500\n", b"1,2,3\n", b"\xff,1500\n"]:
        assert f.feed(raw)[-1][0] is False
    assert f.feed(b"x"*100)[0][0] is False
    assert f.feed(b"1500,1500\n") == []  # discard remainder through newline
    assert f.feed(b"1500,1500\n")[-1][0] is True
    assert len(f.buffer) == 0


@pytest.mark.parametrize("kwargs", [{"rc_timeout": 0}, {"autonomy_timeout": math.inf},
                                    {"release": 101}, {"release": -1}])
def test_invalid_policy_is_rejected(kwargs):
    with pytest.raises(ValueError):
        AuthorityPolicy(**kwargs)


def test_bad_mapping_configuration_is_rejected():
    for kwargs in [{"erpm_limit": 0}, {"servo_center": 2}, {"erpm_per_us": math.nan}]:
        with pytest.raises(ValueError):
            Mapping(**kwargs)


def test_manual_only_neutral_does_not_require_an_autonomy_sample():
    authority = Authority()
    authority.receive_rc(rc(), 1.)
    selection = authority.decide(1.)
    assert selection.mode == Mode.MANUAL and selection.reason == "manual_only"
    assert not authority.autonomy_enabled
    backend = Backend()
    backend.receive(selection, 1.)
    assert backend.enable(True, 1.)
    assert backend.output(1.)[0] == Actuation(0., .5)


def test_manual_release_returns_steering_to_neutral_while_autonomy_is_disabled():
    authority, backend = Authority(), Backend()
    authority.receive_rc(rc(), 1.)
    backend.receive(authority.decide(1.), 1.)
    assert backend.enable(True, 1.)
    authority.receive_rc(rc(1700., 1700.), 1.05)
    backend.receive(authority.decide(1.05), 1.05)
    assert backend.output(1.05)[0] == Actuation(-600., .6)
    authority.receive_rc(rc(), 1.1)
    selection = authority.decide(1.1)
    assert selection.mode == Mode.MANUAL and not authority.autonomy_enabled
    backend.receive(selection, 1.1)
    assert backend.output(1.1)[0] == Actuation(0., .5)


def test_disarmed_backend_does_not_apply_manual_only_commands():
    authority, backend = Authority(), Backend()
    authority.receive_rc(rc(1700., 1700.), 1.)
    backend.receive(authority.decide(1.), 1.)
    assert not backend.enabled
    assert backend.output(1.)[0] == Actuation(0., .5)
    assert not backend.enable(True, 1.)  # The new manual mode still requires neutral.


def test_manual_only_source_expiry_stops_and_does_not_use_stale_steering():
    authority, backend = Authority(), Backend()
    authority.receive_rc(rc(), 1.)
    backend.receive(authority.decide(1.), 1.)
    assert backend.enable(True, 1.)
    authority.receive_rc(rc(1700., 1700.), 1.05)
    backend.receive(authority.decide(1.05), 1.05)
    assert backend.output(1.05)[0] == Actuation(-600., .6)
    expired = authority.decide(1.251)
    assert expired.mode == Mode.STOP and expired.reason == "rc_invalid_or_expired"
    backend.receive(expired, 1.251)
    assert backend.output(1.251)[0] == Actuation(0., .6)


def test_enabling_autonomy_at_neutral_still_requires_valid_autonomy():
    authority = Authority()
    authority.receive_rc(rc(), 1.)
    assert authority.decide(1.).mode == Mode.MANUAL
    authority.autonomy_enabled = True
    assert authority.decide(1.).mode == Mode.STOP
    authority.receive_autonomy(Actuation(float("nan"), .5), 1.)
    assert authority.decide(1.).mode == Mode.STOP
    authority.receive_autonomy(Actuation(200., .55), 1.)
    assert authority.decide(1.).mode == Mode.AUTONOMY


@pytest.mark.parametrize("sample", [rc(valid=False), rc(rf=2), rc(float("nan")), rc(0.)])
def test_manual_only_does_not_accept_invalid_rc(sample):
    authority = Authority()
    authority.receive_rc(sample, 1.)
    selection = authority.decide(1.)
    assert selection.mode == Mode.STOP
    backend = Backend()
    backend.receive(selection, 1.)
    assert not backend.enable(True, 1.)

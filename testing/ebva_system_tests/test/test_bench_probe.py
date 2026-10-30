"""Probe sequence verification with a fake observer; no Node/device construction."""
from types import SimpleNamespace
import pytest
from ebva_control_msgs.msg import SelectedControl
from ebva_system_tests import bench_probe as probe


@pytest.mark.parametrize("duration", [float("nan"), float("inf"), 0, 1.9, 5.1, 60])
def test_observation_plan_rejects_unbounded_duration(duration):
    with pytest.raises(ValueError):
        probe.steering_step_plan(duration)


def test_observation_plan_returns_to_reference_inside_existing_limits():
    plan = probe.steering_step_plan(5.)
    assert sum(duration for _, duration in plan) == 45.
    assert min(value for value, _ in plan) == .4
    assert max(value for value, _ in plan) == .6
    assert plan[0][0] == plan[-1][0] == .5
    assert [.45, .55] == [plan[1][0], plan[3][0]]


class FakeObserver:
    def __init__(self, clock, failure=None):
        self.clock, self.failure = clock, failure
        self.latest = {}
        self.events = []
        self.armed = False

    def arm(self):
        self.armed = True

    def spin_for(self, duration):
        self.clock.now += duration
        mode = SelectedControl.MANUAL if self.failure == "manual" else SelectedControl.AUTONOMY
        self.latest["selection"] = {"mode":mode}
        self.record("motor_command", {"erpm":1. if self.failure == "motor" else self.erpm})
        self.record("servo_command", {"servo":self.servo})
        if self.failure != "missing_echo":
            self.record("servo_driver_echo", {"servo":self.servo})

    def record(self, kind, fields):
        row = {"monotonic":self.clock.now, "kind":kind, **fields}
        self.events.append(row)
        return row


def test_observation_sequence_checks_all_positions_and_zero_motor(monkeypatch):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(probe.time, "monotonic", lambda:clock.now)
    observer = FakeObserver(clock)
    result = probe.Probe.steering_steps(observer, 2.)
    assert observer.armed and observer.erpm == 0.
    assert [row["servo"] for row in result["steering_steps"]] == list(probe.STEERING_STEPS)
    assert result["physical_symmetry_qualified"] is False
    assert result["servo_center_assumed"] is False


@pytest.mark.parametrize("failure", ["manual", "missing_echo", "motor"])
def test_observation_sequence_rejects_override_or_unconfirmed_output(monkeypatch, failure):
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(probe.time, "monotonic", lambda:clock.now)
    with pytest.raises(RuntimeError):
        probe.Probe.steering_steps(FakeObserver(clock, failure), 2.)


@pytest.mark.parametrize("failure", [False, True, "pause_interrupted"])
def test_acquisition_pause_always_resumes_same_process(tmp_path, monkeypatch, failure):
    process = tmp_path/"999"
    process.mkdir()
    (process/"cmdline").write_bytes(b"/opt/venv/bin/python\0/install/rc_acquisition\0")
    calls = []
    monkeypatch.setattr(probe.os, "pidfd_open", lambda pid:calls.append(("open",pid)) or 87)
    def send_signal(fd, sig):
        calls.append(("signal",fd,sig))
        if failure == "pause_interrupted" and sig == probe.signal.SIGSTOP:
            raise RuntimeError("interrupted during pause")
    monkeypatch.setattr(probe.signal, "pidfd_send_signal", send_signal)
    monkeypatch.setattr(probe.os, "close", lambda fd:calls.append(("close",fd)))
    try:
        with probe.paused_acquisition(tmp_path) as pid:
            assert pid == 999
            if failure:
                raise RuntimeError("observer failed")
    except RuntimeError:
        assert failure
    assert calls == [("open",999), ("signal",87,probe.signal.SIGSTOP),
                     ("signal",87,probe.signal.SIGCONT), ("close",87)]


@pytest.mark.parametrize("count", [0, 2])
def test_acquisition_pause_refuses_absent_or_ambiguous_target(tmp_path, monkeypatch, count):
    for number in range(count):
        directory = tmp_path/str(900+number)
        directory.mkdir()
        (directory/"cmdline").write_bytes(b"python\0/installed/rc_acquisition\0")
    monkeypatch.setattr(probe.os, "pidfd_open", lambda pid:pytest.fail("must not signal"))
    with pytest.raises(RuntimeError, match="exactly one"):
        with probe.paused_acquisition(tmp_path):
            pytest.fail("must not pause")


def test_acquisition_pause_refuses_changed_command_line(tmp_path, monkeypatch):
    directory = tmp_path/"999"
    directory.mkdir()
    command = directory/"cmdline"
    command.write_bytes(b"python\0/installed/rc_acquisition\0")
    closed = []
    def open_pid(pid):
        command.write_bytes(b"unrelated\0")
        return 87
    monkeypatch.setattr(probe.os, "pidfd_open", open_pid)
    monkeypatch.setattr(probe.signal, "pidfd_send_signal", lambda *args:pytest.fail("must not signal"))
    monkeypatch.setattr(probe.os, "close", closed.append)
    with pytest.raises(RuntimeError, match="changed"):
        with probe.paused_acquisition(tmp_path):
            pytest.fail("must not pause")
    assert closed == [87]


class FakeSilenceObserver(FakeObserver):
    def __init__(self, clock, failure=None):
        super().__init__(clock, failure)
        self.paused = False
        self.pause_start = 0.
        self.resumed = False

    def spin_for(self, duration):
        for _ in range(round(duration/.01)):
            self.clock.now += .01
            expired = self.paused and self.clock.now-self.pause_start >= .2
            mode = SelectedControl.STOP if expired and self.failure != "no_stop" else SelectedControl.AUTONOMY
            if self.resumed and self.failure == "no_recovery":
                mode = SelectedControl.STOP
            self.latest["selection"] = {"mode":mode}
            reason = "rc_invalid_or_expired" if mode == SelectedControl.STOP else "neutral_with_valid_autonomy"
            self.record("selection", {"mode":mode, "reason":reason})
            self.record("motor_command", {"erpm":1. if self.failure == "motor" else 0.})
            if not self.paused and self.failure != "no_recovery":
                self.record("rc", {"valid":True})


@pytest.mark.parametrize("failure", [None, "no_stop", "no_recovery", "motor"])
def test_rc_silence_requires_stop_fresh_recovery_and_zero_motor(monkeypatch, failure):
    from contextlib import contextmanager
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(probe.time, "monotonic", lambda:clock.now)
    observer = FakeSilenceObserver(clock, failure)
    @contextmanager
    def pause():
        observer.paused, observer.pause_start = True, clock.now
        try:
            yield 999
        finally:
            observer.paused, observer.resumed = False, True
    monkeypatch.setattr(probe, "paused_acquisition", pause)
    if failure:
        with pytest.raises(RuntimeError):
            probe.Probe.rc_silence(observer)
    else:
        result = probe.Probe.rc_silence(observer)
        assert result["rc_silence_stop"] and result["fresh_rc_recovery"] and result["motor_zero"]
        assert result["stop_observed_after_pause_s"] <= .35
        assert not result["rf_loss_tested"] and not result["physical_braking_tested"]
    assert observer.resumed and not observer.paused


def test_authority_pause_uses_same_guarded_process_handle(tmp_path, monkeypatch):
    directory = tmp_path/"999"
    directory.mkdir()
    (directory/"cmdline").write_bytes(b"python\0/installed/command_authority\0")
    calls = []
    monkeypatch.setattr(probe.os, "pidfd_open", lambda pid:calls.append(("open",pid)) or 87)
    monkeypatch.setattr(probe.signal, "pidfd_send_signal",
                        lambda fd,sig:calls.append(("signal",fd,sig)))
    monkeypatch.setattr(probe.os, "close", lambda fd:calls.append(("close",fd)))
    with probe.paused_control_process("command_authority", tmp_path) as pid:
        assert pid == 999
    assert calls == [("open",999), ("signal",87,probe.signal.SIGSTOP),
                     ("signal",87,probe.signal.SIGCONT), ("close",87)]


@pytest.mark.parametrize("name", ["vesc_backend", "vesc_driver_node"])
def test_silence_probe_refuses_pausing_actuator_processes(tmp_path, monkeypatch, name):
    monkeypatch.setattr(probe.os, "pidfd_open", lambda pid:pytest.fail("must not signal"))
    with pytest.raises(ValueError, match="Only acquisition/authority"):
        with probe.paused_control_process(name, tmp_path):
            pytest.fail("must not pause")


class FakeAuthorityObserver(FakeSilenceObserver):
    def spin_for(self, duration):
        for _ in range(round(duration/.01)):
            self.clock.now += .01
            if not self.paused or self.failure == "selection_still_published":
                mode = SelectedControl.STOP if self.resumed and self.failure == "no_recovery" else SelectedControl.AUTONOMY
                self.latest["selection"] = self.record("selection", {"mode":mode})
            if not (self.paused and self.failure == "rc_silent"):
                self.record("rc", {"valid":True})
            expired = (self.paused and self.clock.now-self.pause_start >= .2)
            if self.failure == "no_expiry":
                expired = False
            reason = "selection_expired" if expired else "neutral_with_valid_autonomy"
            if self.resumed and self.failure == "no_recovery":
                reason = "selection_expired"
            enabled = not (self.paused and self.failure == "backend_disarmed")
            self.latest["backend_status"] = self.record("backend_status", {"enabled":enabled, "reason":reason})
            self.record("motor_command", {"erpm":1. if self.failure == "motor" else 0.})


@pytest.mark.parametrize("failure", [None, "no_expiry", "backend_disarmed", "rc_silent",
                                      "selection_still_published", "no_recovery", "motor"])
def test_authority_silence_requires_independent_expiry_live_rc_and_recovery(monkeypatch, failure):
    from contextlib import contextmanager
    clock = SimpleNamespace(now=0.)
    monkeypatch.setattr(probe.time, "monotonic", lambda:clock.now)
    observer = FakeAuthorityObserver(clock, failure)
    @contextmanager
    def pause(name):
        assert name == "command_authority"
        observer.paused, observer.pause_start = True, clock.now
        try:
            yield 999
        finally:
            observer.paused, observer.resumed = False, True
    monkeypatch.setattr(probe, "paused_control_process", pause)
    if failure:
        with pytest.raises(RuntimeError):
            probe.Probe.authority_silence(observer)
    else:
        result = probe.Probe.authority_silence(observer)
        assert result["independent_backend_expiry"] and result["rc_remained_live"]
        assert result["selection_silence_verified"] and result["recovered_without_rearming"]
        assert result["motor_zero"]
        assert not result["firmware_watchdog_tested"] and not result["physical_braking_tested"]
    assert observer.resumed and not observer.paused

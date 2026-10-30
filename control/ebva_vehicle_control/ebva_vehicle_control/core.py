"""Pure authority and actuator mapping; injected monotonic seconds, no ROS."""
from dataclasses import dataclass
from enum import IntEnum
import math


class Mode(IntEnum):
    STOP = 0
    MANUAL = 1
    AUTONOMY = 2


@dataclass(frozen=True)
class RcSample:
    throttle: float = 0.0
    steering: float = 0.0
    valid: bool = False
    rf_state: int = 0
    reason: str = "no_rc"


@dataclass(frozen=True)
class Actuation:
    erpm: float = 0.0
    servo: float = 0.5

    def valid(self):
        return math.isfinite(self.erpm) and math.isfinite(self.servo) and 0 <= self.servo <= 1


@dataclass(frozen=True)
class AuthorityPolicy:
    rc_timeout: float = 0.2
    autonomy_timeout: float = 0.2
    center: float = 1500.0
    enter: float = 100.0
    release: float = 80.0

    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in (
                self.rc_timeout, self.autonomy_timeout, self.center, self.enter)):
            raise ValueError("Invalid authority policy")
        if not math.isfinite(self.release) or not 0 <= self.release <= self.enter:
            raise ValueError("Invalid neutral hysteresis")

    def neutral(self, sample):
        return max(abs(sample.throttle-self.center), abs(sample.steering-self.center)) <= self.release


@dataclass(frozen=True)
class Selection:
    mode: Mode
    reason: str
    rc: RcSample = RcSample()
    autonomy: Actuation = Actuation()
    rc_age: float = math.inf
    autonomy_age: float = math.inf


class Authority:
    def __init__(self, policy=AuthorityPolicy()):
        self.policy = policy
        self.rc, self.auto = RcSample(), Actuation()
        self.rc_time = self.auto_time = None
        self.auto_valid = False
        self.autonomy_enabled = False
        self.manual = False
        self.last_tick = None

    def receive_rc(self, sample, now):
        valid = (sample.valid and sample.rf_state in (0, 1)
                 and all(math.isfinite(x) and 900 <= x <= 2100
                         for x in (sample.throttle, sample.steering)))
        self.rc = sample if valid else RcSample(reason=sample.reason or "invalid_rc")
        self.rc_time = now

    def receive_autonomy(self, command, now):
        self.auto, self.auto_time = command, now
        self.auto_valid = command.valid()

    def decide(self, now):
        if not math.isfinite(now):
            raise ValueError("Invalid monotonic time")
        if self.last_tick is not None and now < self.last_tick:
            self.rc_time = self.auto_time = None
            self.rc = RcSample(reason="clock_reset")
            self.auto_valid = False
            self.manual = False
            self.last_tick = now
            return Selection(Mode.STOP, "clock_reset")
        self.last_tick = now
        rc_age = math.inf if self.rc_time is None else now-self.rc_time
        auto_age = math.inf if self.auto_time is None else now-self.auto_time
        rc_ok = self.rc.valid and 0 <= rc_age < self.policy.rc_timeout
        auto_ok = self.auto_valid and 0 <= auto_age < self.policy.autonomy_timeout
        args = dict(rc=self.rc, autonomy=self.auto, rc_age=rc_age, autonomy_age=auto_age)
        if not rc_ok:
            return Selection(Mode.STOP, "rc_invalid_or_expired", **args)
        deviation = max(abs(self.rc.throttle-self.policy.center), abs(self.rc.steering-self.policy.center))
        if deviation > self.policy.enter:
            self.manual = True
        elif deviation <= self.policy.release:
            self.manual = False
        if self.manual:
            return Selection(Mode.MANUAL, "rc_override", **args)
        if not self.autonomy_enabled:
            # Valid RC remains the complete command source in manual-only mode,
            # including neutral steering. STOP would hold the previous servo.
            return Selection(Mode.MANUAL, "manual_only", **args)
        if not auto_ok:
            return Selection(Mode.STOP, "autonomy_invalid_or_expired", **args)
        return Selection(Mode.AUTONOMY, "neutral_with_valid_autonomy", **args)


@dataclass(frozen=True)
class Mapping:
    # Selected legacy RC linear map: -4000/+4000 at 2000/1000 us.
    erpm_per_us: float = -8.0
    throttle_center: float = 1500.0
    throttle_deadband: float = 100.0
    steering_min_us: float = 1100.0
    steering_max_us: float = 1900.0
    steering_min_servo: float = 0.15
    steering_max_servo: float = 0.85
    servo_center: float = 0.5
    erpm_limit: float = 600.0
    servo_min: float = 0.4
    servo_max: float = 0.6

    def __post_init__(self):
        if not all(math.isfinite(x) for x in self.__dict__.values()):
            raise ValueError("Nonfinite mapping")
        if not (self.erpm_limit > 0 and self.throttle_deadband >= 0
                and self.steering_max_us > self.steering_min_us
                and 0 <= self.servo_min <= self.servo_center <= self.servo_max <= 1
                and 0 <= self.steering_min_servo <= self.steering_max_servo <= 1):
            raise ValueError("Invalid mapping limits")

    def bounded(self, command):
        if not command.valid():
            raise ValueError("Invalid actuator command")
        return Actuation(max(-self.erpm_limit, min(self.erpm_limit, command.erpm)),
                         max(self.servo_min, min(self.servo_max, command.servo)))

    def manual(self, rc):
        if not rc.valid or not all(math.isfinite(x) and 900 <= x <= 2100
                                   for x in (rc.throttle, rc.steering)):
            raise ValueError("Invalid RC for conversion")
        delta = rc.throttle-self.throttle_center
        # Fix: steering-only override must not turn throttle-center jitter into motion.
        erpm = 0.0 if abs(delta) <= self.throttle_deadband else delta*self.erpm_per_us
        ratio = (rc.steering-self.steering_min_us)/(self.steering_max_us-self.steering_min_us)
        servo = self.steering_min_servo+ratio*(self.steering_max_servo-self.steering_min_servo)
        return self.bounded(Actuation(erpm, max(0.0, min(1.0, servo))))


class Backend:
    """Lease expiry independent of authority; arming requires fresh neutral RC."""
    def __init__(self, mapping=Mapping(), timeout=0.2, policy=AuthorityPolicy()):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Invalid backend timeout")
        self.mapping, self.timeout, self.policy = mapping, timeout, policy
        self.enabled = False
        self.selection, self.received = None, None
        self.last_servo = mapping.servo_center

    def receive(self, selection, now):
        self.selection, self.received = selection, now

    def fresh(self, now):
        return self.received is not None and 0 <= now-self.received < self.timeout

    def enable(self, value, now):
        if value:
            if not (self.fresh(now) and self.selection.rc.valid
                    and self.policy.neutral(self.selection.rc)
                    and 0 <= self.selection.rc_age+now-self.received < self.policy.rc_timeout):
                return False
        self.enabled = bool(value)
        return True

    def output(self, now):
        reason = "backend_disabled"
        command = Actuation(0.0, self.last_servo)
        if self.enabled and self.fresh(now):
            selected = self.selection
            reason = selected.reason
            elapsed = now-self.received
            rc_fresh = (selected.rc.valid and selected.rc.rf_state in (0, 1)
                        and 0 <= selected.rc_age+elapsed < self.policy.rc_timeout)
            auto_fresh = 0 <= selected.autonomy_age+elapsed < self.policy.autonomy_timeout
            try:
                if not rc_fresh:
                    reason = "rc_expired_in_backend"
                elif selected.mode == Mode.MANUAL:
                    command = self.mapping.manual(selected.rc)
                elif selected.mode == Mode.AUTONOMY and auto_fresh:
                    command = self.mapping.bounded(selected.autonomy)
                elif selected.mode != Mode.STOP:
                    reason = "invalid_selection"
            except ValueError:
                reason = "invalid_selection"
        elif self.enabled:
            reason = "selection_expired"
        self.last_servo = command.servo
        return command, reason

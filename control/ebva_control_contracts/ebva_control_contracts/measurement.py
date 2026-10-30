"""Measurement lease: original ROS age plus bounded local monotonic lifetime."""
import math


class MeasurementLease:
    def __init__(self, timeout, frame, future_slack=.02):
        if not math.isfinite(timeout) or timeout <= 0 or not frame:
            raise ValueError("Positive observation timeout and explicit frame required")
        self.timeout, self.frame, self.future_slack = timeout, frame, future_slack
        self.clear()

    def clear(self):
        self.stamp = self.received = self.deadline = None

    def observe(self, stamp, ros_now, monotonic_now, frame):
        if frame != self.frame or not all(math.isfinite(v) for v in (stamp, ros_now, monotonic_now)):
            return False
        age = ros_now-stamp
        if stamp <= 0 or not -self.future_slack <= age < self.timeout:
            return False
        if self.stamp is not None and stamp <= self.stamp:
            return False  # Repeated source time cannot renew through a paused /clock.
        self.stamp, self.received = stamp, monotonic_now
        self.deadline = monotonic_now+self.timeout-max(0., age)
        return True

    def fresh(self, ros_now, monotonic_now):
        return (self.stamp is not None and self.received <= monotonic_now < self.deadline
                and -self.future_slack <= ros_now-self.stamp < self.timeout)

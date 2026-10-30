from enum import Enum


class LaneState(Enum):
    TRACKING = 1
    DEGRADED = 2
    LOST = 3


class ConfidenceFSM:
    """
    Robust lane-confidence state machine.

    TRACKING:
        Normal lane detection.

    DEGRADED:
        Detection temporarily unreliable.

    LOST:
        Lane unavailable for a sustained period.
    """

    def __init__(
        self,
        degraded_after=0.15,
        lost_after=0.50,
        recovery_time=0.20,
    ):
        self.degraded_after = float(degraded_after)
        self.lost_after = float(lost_after)
        self.recovery_time = float(recovery_time)

        self.state = LaneState.LOST
        self.invalid_time = 0.0
        self.valid_time = 0.0

    def update(self, detected, dt):
        dt = max(float(dt), 0.0)

        if detected:
            self.invalid_time = 0.0
            self.valid_time += dt

            if self.state == LaneState.LOST:
                if self.valid_time >= self.recovery_time:
                    self.state = LaneState.DEGRADED

            if self.state == LaneState.DEGRADED:
                if self.valid_time >= self.recovery_time:
                    self.state = LaneState.TRACKING

        else:
            self.valid_time = 0.0
            self.invalid_time += dt

            if self.invalid_time >= self.lost_after:
                self.state = LaneState.LOST

            elif self.invalid_time >= self.degraded_after:
                self.state = LaneState.DEGRADED

        return self.state

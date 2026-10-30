"""CSV/Frenet scalar-progress model, independent of ROS and actuator maps."""
import math
import numpy as np


class CsvFrenetCore:
    """Preserve the selected model's laws; references use metres, not XY poses."""

    def __init__(self, s_ref, ey_ref):
        self.L = 0.26
        self.Ld = 1.2
        self.max_steer = 0.4
        self.v_min = 0.4
        self.v_max = 0.5
        self.k_speed = 0.8
        self.s_ref = np.asarray(s_ref, dtype=float).copy()
        self.ey_ref = np.asarray(ey_ref, dtype=float).copy()
        if (self.s_ref.ndim != 1 or self.s_ref.shape != self.ey_ref.shape
                or len(self.s_ref) < 2 or not np.isfinite(self.s_ref).all()
                or not np.isfinite(self.ey_ref).all() or self.s_ref[0] != 0.
                or not np.all(np.diff(self.s_ref) > 0.)
                or self.s_ref[-1] < self.Ld):
            raise ValueError('Closed progress reference requires s=0, increasing finite s/e_y and length >= Ld')
        self.s_ref.setflags(write=False)
        self.ey_ref.setflags(write=False)
        self.s_max = self.s_ref[-1]
        self.s = 0.0
        self.prev_x = None
        self.prev_y = None
        self.yaw = 0.0
        self.v = 0.0

    def update_odometry(self, x, y, yaw, v):
        if not all(math.isfinite(value) for value in (x, y, yaw, v)):
            raise ValueError('Odometry values must be finite')
        if self.prev_x is not None:
            ds = math.hypot(x-self.prev_x, y-self.prev_y)
            # The preserved model wraps once. Reject undefined jumps rather than
            # silently clamping interpolation after a multi-lap increment.
            if not math.isfinite(ds) or ds > self.s_max:
                raise ValueError('Position step exceeds the reference length')
            self.s += ds
            if self.s > self.s_max:
                self.s -= self.s_max
        self.prev_x, self.prev_y = x, y
        self.yaw, self.v = yaw, v

    def command(self):
        if self.prev_x is None:
            return None
        s_look = self.s + self.Ld
        if s_look > self.s_max:
            s_look -= self.s_max
        ey_look = np.interp(s_look, self.s_ref, self.ey_ref)
        kappa = 2.0 * ey_look / (self.Ld ** 2)
        delta = math.atan(self.L * kappa)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))
        v_cmd = self.v_max / (1.0 + self.k_speed * abs(kappa))
        v_cmd = float(np.clip(v_cmd, self.v_min, self.v_max))
        return v_cmd, delta

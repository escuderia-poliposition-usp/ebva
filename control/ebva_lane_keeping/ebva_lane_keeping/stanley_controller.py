import math


class StanleyController:
    """
    Competition-oriented Stanley controller.

    Convention:
        lateral_error > 0:
            vehicle is LEFT of reference path.

        heading_error:
            desired path heading - vehicle heading.

        curvature > 0:
            path curves LEFT.

        steering > 0:
            LEFT
        steering < 0:
            RIGHT

    k_soft (2026-10-15): termino de suavizado de Stanley.
        lateral_term = atan2(-k * e, k_soft + v)
        k_soft = 0.0 reproduce EXACTAMENTE el comportamiento anterior.
        A baja velocidad (0.15 m/s) sin k_soft la direccion satura con ~7 cm de error.
    """

    def __init__(
        self,
        k_stanley=1.0,
        max_steering=0.45,
        min_speed=0.10,
        speed_gain=0.0,
        feedforward_gain=1.0,
        wheelbase=0.33,
        steering_rate=3.0,
        k_soft=0.0,
    ):
        self.k_stanley = float(k_stanley)
        self.max_steering = abs(float(max_steering))
        self.min_speed = max(float(min_speed), 1e-3)

        self.speed_gain = float(speed_gain)
        self.feedforward_gain = float(feedforward_gain)
        self.wheelbase = float(wheelbase)
        self.k_soft = max(float(k_soft), 0.0)

        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be > 0")

        self.steering_rate = abs(float(steering_rate))
        self.previous_steering = 0.0

    def reset(self):
        self.previous_steering = 0.0

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    @staticmethod
    def _finite(value, default=0.0):
        value = float(value)

        if not math.isfinite(value):
            return float(default)

        return value

    def compute_steering(
        self,
        lateral_error,
        heading_error,
        vehicle_speed,
        curvature=0.0,
        dt=None,
    ):
        lateral_error = self._finite(lateral_error)
        heading_error = self._finite(heading_error)
        vehicle_speed = self._finite(vehicle_speed)
        curvature = self._finite(curvature)

        speed = max(
            vehicle_speed,
            self.min_speed,
        )

        heading_error = self._normalize_angle(
            heading_error
        )

        k = (
            self.k_stanley
            + self.speed_gain * speed
        )

        k = max(k, 0.0)

        # Positive ey = vehicle left of path.
        # Therefore correction must steer right.
        lateral_term = math.atan2(
            -k * lateral_error,
            self.k_soft + speed,
        )

        # Curvature feed-forward.
        # Positive curvature = left turn.
        feedforward = (
            self.feedforward_gain
            * math.atan(
                self.wheelbase * curvature
            )
        )

        steering_raw = (
            heading_error
            + lateral_term
            + feedforward
        )

        steering_raw = self._clamp(
            steering_raw
        )

        steering = self._rate_limit(
            steering_raw,
            dt,
        )

        steering = self._clamp(
            steering
        )

        self.previous_steering = steering

        return steering

    def _rate_limit(self, target, dt):
        if dt is None:
            return target

        dt = float(dt)

        if not math.isfinite(dt) or dt <= 0.0:
            return target

        max_delta = (
            self.steering_rate * dt
        )

        delta = (
            target
            - self.previous_steering
        )

        if delta > max_delta:
            return (
                self.previous_steering
                + max_delta
            )

        if delta < -max_delta:
            return (
                self.previous_steering
                - max_delta
            )

        return target

    def _clamp(self, value):
        return max(
            -self.max_steering,
            min(
                self.max_steering,
                value,
            ),
        )

"""Bounded, nonblocking two-channel serial framing; no ROS or device I/O."""
import math


class RcFramer:
    def __init__(self, minimum=900.0, maximum=2100.0, max_line=64):
        if not (math.isfinite(minimum) and math.isfinite(maximum) and 0 < minimum < maximum):
            raise ValueError("Invalid PWM validity interval")
        self.minimum, self.maximum, self.max_line = minimum, maximum, max_line
        self.buffer = bytearray()
        self.discarding = False

    def feed(self, data):
        """Return complete frames as (valid, throttle, steering, reason).

        Overlong input is discarded through its newline; a trailing fragment
        never becomes a complete fresh sample. Work is bounded by caller bytes.
        """
        frames = []
        for byte in data:
            if self.discarding:
                if byte == 10:
                    self.discarding = False
                continue
            if byte == 10:
                raw = bytes(self.buffer)
                self.buffer.clear()
                try:
                    parts = raw.decode("ascii").strip().split(",")
                    if len(parts) != 2:
                        raise ValueError("expected_two_channels")
                    a, b = map(float, parts)
                    if not all(math.isfinite(x) and self.minimum <= x <= self.maximum for x in (a, b)):
                        raise ValueError("invalid_pwm")
                    frames.append((True, a, b, "serial_frame_rf_unknown"))
                except (ValueError, UnicodeDecodeError):
                    frames.append((False, 0.0, 0.0, "invalid_serial_frame"))
            else:
                self.buffer.append(byte)
                if len(self.buffer) > self.max_line:
                    self.buffer.clear()
                    self.discarding = True
                    frames.append((False, 0.0, 0.0, "serial_frame_too_long"))
        return frames

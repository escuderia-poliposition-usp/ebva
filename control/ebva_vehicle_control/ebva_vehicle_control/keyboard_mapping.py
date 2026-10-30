"""Manual keyboard axes mapped to vehicle actuator limits, without SI claims."""
import math
from dataclasses import dataclass, fields
from pathlib import Path

import yaml

from .core import Actuation, AuthorityPolicy, Mapping


@dataclass(frozen=True)
class KeyboardSettings:
    source_timeout_s: float = 0.2
    input_topic: str = "key_vel"
    expected_publisher: str = "key_teleop"
    expected_frame: str = "key_teleop"
    linear_full_scale: float = 1.0
    angular_full_scale: float = 1.0
    motor_direction: float = 1.0
    steering_direction: float = -1.0
    enable_external_on_startup: bool = True

    def __post_init__(self):
        positive = (self.source_timeout_s, self.linear_full_scale, self.angular_full_scale)
        if not all(type(v) in (int, float) and math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("Finite positive keyboard timeout/scales required")
        if any(type(v) not in (int, float) or v not in (-1.0, 1.0)
               for v in (self.motor_direction, self.steering_direction)):
            raise ValueError("Keyboard directions must be -1 or 1")
        if type(self.enable_external_on_startup) is not bool:
            raise ValueError("enable_external_on_startup must be boolean")
        if not all(isinstance(v, str) and v for v in
                   (self.input_topic, self.expected_publisher, self.expected_frame)):
            raise ValueError("Explicit keyboard topic/publisher/frame required")

    def convert(self, linear, angular, vehicle):
        if not all(math.isfinite(v) for v in (linear, angular)):
            raise ValueError("Nonfinite keyboard axes")
        if abs(linear) > self.linear_full_scale or abs(angular) > self.angular_full_scale:
            raise ValueError("Keyboard axes exceed configured scale")
        erpm = self.motor_direction * linear / self.linear_full_scale * vehicle.erpm_limit
        turn = self.steering_direction * angular / self.angular_full_scale
        extent = (vehicle.steering_max_servo - vehicle.servo_center if turn >= 0
                  else vehicle.servo_center - vehicle.steering_min_servo)
        return vehicle.bounded(Actuation(erpm, vehicle.servo_center + turn * extent))


def load_vehicle(path):
    data = yaml.safe_load(Path(path).read_text())
    params = data["vesc_backend"]["ros__parameters"]
    mapping = Mapping(**{field.name: params[field.name] for field in fields(Mapping)})
    policy = AuthorityPolicy(params["rc_timeout_s"], params["autonomy_timeout_s"],
                             params["rc_center_us"], params["override_enter_us"],
                             params["override_release_us"])
    return mapping, policy


def inspect_keyboard(params_file, vehicle_params_file, key_params_file):
    """Domain validation only: no DDS, terminal or devices."""
    data = yaml.safe_load(Path(params_file).read_text())
    if set(data) != {"keyboard_adapter"} or set(data["keyboard_adapter"]) != {"ros__parameters"}:
        raise ValueError("Expected keyboard_adapter parameters")
    settings = KeyboardSettings(**data["keyboard_adapter"]["ros__parameters"])
    mapping, policy = load_vehicle(vehicle_params_file)
    key = yaml.safe_load(Path(key_params_file).read_text())["key_teleop"]["ros__parameters"]
    if key.get("twist_stamped_enabled") is not True:
        raise ValueError("key_teleop must publish stamped commands")
    hz = key.get("hz", 10)
    if not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz <= 0 or 1 / hz >= settings.source_timeout_s:
        raise ValueError("key_teleop rate must exceed the keyboard lease")
    for name, limit in (("forward_rate", settings.linear_full_scale),
                        ("backward_rate", settings.linear_full_scale),
                        ("rotation_rate", settings.angular_full_scale)):
        value = key[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= limit:
            raise ValueError("key_teleop rate exceeds keyboard scale: " + name)
    if settings.expected_publisher != "key_teleop":
        raise ValueError("The selected key_teleop node must own the keyboard input")
    return settings, mapping, policy

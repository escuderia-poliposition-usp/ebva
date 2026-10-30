"""Keyboard conversion, stamped leases, public launch and real vendor TTY."""
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import fcntl

import pytest
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ebva_vehicle_control.core import Mapping
from ebva_vehicle_control.keyboard_mapping import KeyboardSettings, inspect_keyboard
from ebva_vehicle_control.keyboard_adapter import KeyboardAdapter
from ebva_vehicle_control.nodes import qos

BASE = Path(__file__).resolve().parents[1]


def paths():
    key = BASE / "config/key_teleop.yaml"
    return BASE / "config/keyboard_adapter.yaml", BASE / "config/vehicle/manual_core.yaml", key


def test_keyboard_defaults_resolve_owner_configuration():
    settings, vehicle, policy = inspect_keyboard(*paths())
    assert settings.expected_publisher == "key_teleop"
    assert settings.enable_external_on_startup
    assert vehicle.erpm_limit > 0 and policy.rc_timeout > 0


def test_normalized_axes_use_vehicle_bounds_and_asymmetric_steering():
    settings = KeyboardSettings()
    vehicle = Mapping(erpm_limit=2000., servo_min=.15, servo_max=.85)
    assert settings.convert(.8, 1., vehicle).erpm == 1600.
    assert settings.convert(.8, 1., vehicle).servo == pytest.approx(.15)
    assert settings.convert(-.5, -1., vehicle).erpm == -1000.
    assert settings.convert(-.5, -1., vehicle).servo == pytest.approx(.85)
    assert settings.convert(0., 0., vehicle).servo == .5
    bounded = Mapping()
    assert settings.convert(1., 1., bounded).servo == .4
    assert settings.convert(1., -1., bounded).servo == .6


@pytest.mark.parametrize("linear,angular", [(math.nan, 0.), (0., math.inf), (1.01, 0.), (0., -1.01)])
def test_invalid_or_out_of_scale_axes_are_rejected(linear, angular):
    with pytest.raises(ValueError):
        KeyboardSettings().convert(linear, angular, Mapping())


@pytest.mark.parametrize("change", [
    {"source_timeout_s": 0.}, {"linear_full_scale": math.inf},
    {"motor_direction": 0.}, {"steering_direction": True},
    {"enable_external_on_startup": "true"}, {"input_topic": ""},
])
def test_invalid_keyboard_settings_are_rejected(change):
    with pytest.raises(ValueError):
        KeyboardSettings(**change)


@pytest.mark.parametrize("change", ["unstamped", "slow_rate", "overscale"])
def test_incompatible_vendor_configuration_fails_before_launch(tmp_path, change):
    adapter, vehicle, key = paths()
    data = yaml.safe_load(key.read_text())
    values = data["key_teleop"]["ros__parameters"]
    if change == "unstamped":
        values["twist_stamped_enabled"] = False
    elif change == "slow_rate":
        values["hz"] = 1
    else:
        values["forward_rate"] = 2.
    target = tmp_path / "key.yaml"
    target.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        inspect_keyboard(adapter, vehicle, target)


@pytest.mark.parametrize("failure", ["old", "frame", "repeat", "nan", "unused_axis", "publisher"])
def test_adapter_preserves_source_stamp_and_rejects_bad_input(failure):
    context = Context()
    rclpy.init(context=context)
    node = KeyboardAdapter(context=context)
    items = []
    class Sink:
        def publish(self, message):
            items.append(message)
    node.pub = Sink()
    node.input_unique = lambda: True
    try:
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "key_teleop"
        message.twist.linear.x = .5
        node.receive(message)
        assert len(items) == 1 and items[0].header.stamp == message.header.stamp
        message.header.stamp = node.get_clock().now().to_msg()
        if failure == "old":
            message.header.stamp.sec -= 1
        elif failure == "frame":
            message.header.frame_id = "other"
        elif failure == "repeat":
            node.last_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        elif failure == "nan":
            message.twist.linear.x = math.nan
        elif failure == "unused_axis":
            message.twist.linear.y = 1.
        else:
            node.input_unique = lambda: False
        node.receive(message)
        assert len(items) == 1 and node.last_input is None
    finally:
        node.destroy_node()
        context.shutdown()


def test_keyboard_launch_is_composed_from_vehicle_interfaces(monkeypatch):
    from types import SimpleNamespace
    spec = importlib.util.spec_from_file_location("keyboard_launch", BASE / "launch/keyboard.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    adapter, vehicle, key = paths()
    monkeypatch.setattr(module, "require_terminal", lambda: None)
    actions = module.compose_keyboard(SimpleNamespace(launch_configurations={
        "params_file": str(adapter), "vehicle_params_file": str(vehicle), "key_params_file": str(key)}))
    # Include the public manual chain; each keyboard process is a critical exit.
    from launch.actions import IncludeLaunchDescription, RegisterEventHandler
    from launch_ros.actions import Node as LaunchNode
    assert isinstance(actions[0], IncludeLaunchDescription)
    assert sum(isinstance(item, LaunchNode) for item in actions) == 2
    assert sum(isinstance(item, RegisterEventHandler) for item in actions) == 2


def test_terminal_wrapper_fails_without_controlling_terminal():
    result = subprocess.run(
        [sys.executable, "-c", "from ebva_vehicle_control.key_teleop_terminal import main; main()"],
        stdin=subprocess.DEVNULL, capture_output=True, text=True, start_new_session=True, timeout=5.)
    assert result.returncode != 0
    assert "interactive terminal" in result.stderr


def test_real_key_teleop_works_in_controlling_pty_and_restores_terminal(tmp_path):
    """Real vendor UI and messages, on an isolated test domain without actuators."""
    context = Context()
    rclpy.init(context=context)
    node = Node("keyboard_pty_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    messages = []
    subscription = node.create_subscription(
        TwistStamped, "/keyboard_pty_test/key_vel", lambda m: messages.append(m), qos())
    master, slave = pty.openpty()
    before = termios.tcgetattr(slave)
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 30, 120, 0, 0))
    def attach():
        os.setsid()
        fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
    vendor = yaml.safe_load(paths()[2].read_text())["key_teleop"]["ros__parameters"]
    launch_file = tmp_path / "keyboard_terminal.launch.py"
    launch_file.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import RegisterEventHandler, EmitEvent\n"
        "from launch.event_handlers import OnProcessExit\n"
        "from launch.events import Shutdown\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    node = Node(package='ebva_vehicle_control', executable='key_teleop_terminal', "
        "name='key_teleop', namespace='/keyboard_pty_test', output='screen', "
        "parameters=[" + repr(vendor) + "])\n"
        "    return LaunchDescription([RegisterEventHandler(OnProcessExit(target_action=node, "
        "on_exit=[EmitEvent(event=Shutdown(reason='keyboard exited'))])), node])\n")
    child = subprocess.Popen([
        sys.executable, "/opt/ros/humble/bin/ros2", "launch", str(launch_file)],
        stdin=slave, stdout=slave, stderr=slave, preexec_fn=attach,
        env={**os.environ, "TERM": "xterm"})
    def pump(predicate, timeout=4., keypress=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert child.poll() is None
            if keypress is not None:
                os.write(master, keypress)
            executor.spin_once(timeout_sec=.02)
            if select.select([master], [], [], 0)[0]:
                os.read(master, 65536)
            if predicate():
                return
        raise AssertionError("key_teleop PTY publication deadline")
    try:
        pump(lambda: bool(messages))
        assert messages[-1].header.frame_id == "key_teleop"
        pump(lambda: any(m.twist.linear.x > 0 for m in messages), keypress=b"\x1bOA")
        messages.clear()
        pump(lambda: bool(messages) and messages[-1].twist.linear.x == 0., timeout=3.)
        os.write(master, b"q")
        assert child.wait(timeout=5.) == 0
        assert termios.tcgetattr(slave) == before
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        os.close(master)
        os.close(slave)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()

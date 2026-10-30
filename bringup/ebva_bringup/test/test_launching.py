"""Validate ROS launch composition without executing vehicle nodes."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ebva_bringup.validation import ConfigError


def test_unexpected_exit_requests_shutdown_and_normal_shutdown_does_not():
    from launch.actions import EmitEvent, LogInfo
    from ebva_bringup.launching import stop_on_exit
    event = SimpleNamespace(process_name='test_process', returncode=7)
    actions = stop_on_exit(event, SimpleNamespace(is_shutdown=False))
    assert isinstance(actions[0], LogInfo)
    assert isinstance(actions[1], EmitEvent)
    assert stop_on_exit(event, SimpleNamespace(is_shutdown=True)) == []


def test_exit_handler_stops_a_benign_sleeping_process():
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from ebva_bringup.launching import stop_on_exit
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_exit)),
        ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(0.2); raise SystemExit(7)']),
        ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(5)']),
    ]))
    start = time.monotonic()
    service.run()
    assert time.monotonic() - start < 4


def test_discovery_uses_its_own_ros_context(monkeypatch):
    from ebva_bringup.launching import running_conflicts
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    running_conflicts([], discovery_seconds=0.05)


def test_discovery_ignores_application_launch_arguments(monkeypatch):
    from ebva_bringup.launching import running_conflicts
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    monkeypatch.setattr(sys, 'argv', ['ros2', 'launch', 'f1tenth.vesc_config:=/a path/vesc.yaml'])
    running_conflicts([], discovery_seconds=0.05)


def test_discovery_detects_an_existing_benign_node(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from ebva_bringup.launching import running_conflicts
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    context = Context()
    rclpy.init(context=context)
    node = Node('escuderia_test_existing', context=context, enable_rosout=False, start_parameter_services=False)
    try:
        with pytest.raises(ConfigError, match='ja estao ativos.*escuderia_test_existing'):
            running_conflicts(['/escuderia_test_existing'], discovery_seconds=1.0)
    finally:
        node.destroy_node()
        context.shutdown()

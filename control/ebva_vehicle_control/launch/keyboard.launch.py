"""key_teleop, physical RC priority and the vehicle-owned actuator chain."""
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ebva_vehicle_control.components import default_path
from ebva_vehicle_control.key_teleop_terminal import require_terminal
from ebva_vehicle_control.keyboard_mapping import inspect_keyboard


def compose_keyboard(context):
    paths = {key: Path(context.launch_configurations[key]).expanduser()
             for key in ("params_file", "vehicle_params_file", "key_params_file")}
    inspect_keyboard(**paths)
    require_terminal()
    adapter_params = yaml.safe_load(paths["params_file"].read_text())["keyboard_adapter"]["ros__parameters"]
    key_params = yaml.safe_load(paths["key_params_file"].read_text())["key_teleop"]["ros__parameters"]
    manual = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(default_path("launch/manual.launch.py"))),
        launch_arguments={"params_file": str(paths["vehicle_params_file"])}.items())
    adapter = Node(package="ebva_vehicle_control", executable="keyboard_adapter",
                   name="keyboard_adapter", output="screen",
                   parameters=[adapter_params, {"vehicle_params_file": str(paths["vehicle_params_file"])}])
    keyboard = Node(package="ebva_vehicle_control", executable="key_teleop_terminal",
                    name="key_teleop", output="screen",
                    parameters=[key_params])
    actions = [manual]
    for node in (adapter, keyboard):
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason="Keyboard control component exited"))])), node])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=str(default_path("config/keyboard_adapter.yaml"))),
        DeclareLaunchArgument("vehicle_params_file", default_value=str(
            default_path("config/vehicle/manual_core.yaml"))),
        DeclareLaunchArgument("key_params_file", default_value=str(default_path("config/key_teleop.yaml"))),
        OpaqueFunction(function=compose_keyboard)])

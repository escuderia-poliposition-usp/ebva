"""Explicit bounded bench composition; no hardware by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ebva_vehicle_control.launching import compose


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("hardware", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("check_only", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=compose)])

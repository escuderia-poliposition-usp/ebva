"""Compose installed resources from one declarative YAML profile."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ebva_bringup.profiles import launch_system

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="manual"),
        DeclareLaunchArgument("check_only", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_system)])

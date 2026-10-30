"""Record the running system; environment preparation belongs to the caller."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from pathlib import Path
from ebva_recording.session import launch_recording


def generate_launch_description():
    preset = Path(get_package_share_directory("ebva_recording")) / "config/images.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("preset", default_value=str(preset)),
        DeclareLaunchArgument("duration_s", default_value=""),
        DeclareLaunchArgument("output_root", default_value=""),
        DeclareLaunchArgument("standalone", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=launch_recording),
    ])

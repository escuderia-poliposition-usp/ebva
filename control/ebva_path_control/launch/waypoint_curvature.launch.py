from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = Path(get_package_share_directory("ebva_path_control")) / "config/waypoint_curvature.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=str(default)),
        Node(package="ebva_path_control", executable="waypoint_curvature",
             name="waypoint_curvature", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
    ])

"""Selected supervisor law; classifier input is external, no device provider."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ebva_supervisor.configuration import inspect_configuration


def compose(context):
    params = inspect_configuration(context.launch_configurations["params_file"])
    return [Node(package="ebva_supervisor", executable="supervisor_node",
                 name="supervisor", parameters=[params], output="screen")]


def generate_launch_description():
    default = str(Path(get_package_share_directory("ebva_supervisor")) / "config/supervised.yaml")
    return LaunchDescription([DeclareLaunchArgument("params_file", default_value=default),
                              OpaqueFunction(function=compose)])

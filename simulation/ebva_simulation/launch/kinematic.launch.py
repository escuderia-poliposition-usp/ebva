from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    config=Path(get_package_share_directory("ebva_simulation"))/"config/kinematic.yaml"
    return LaunchDescription([DeclareLaunchArgument("params_file",default_value=str(config)),
        Node(package="ebva_simulation",executable="sim_kinematic",parameters=[LaunchConfiguration("params_file")],output="screen")])

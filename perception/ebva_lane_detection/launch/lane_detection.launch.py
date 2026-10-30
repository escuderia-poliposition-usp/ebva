from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    
    config=Path(get_package_share_directory("ebva_lane_detection"))/"config/lane_detection_params.yaml"

    return LaunchDescription([DeclareLaunchArgument("params_file",default_value=str(config)),
        Node(package="ebva_lane_detection",executable="lane_detection_node",name="lane_detection_node",
             parameters=[LaunchConfiguration("params_file")],output="screen")])

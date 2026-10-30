"""Nav2 and Twist/Ackermann integration; no sensors, localization or driver."""
from pathlib import Path
def share(package="ebva_navigation"):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))
def description(kind,filename):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument,OpaqueFunction
    return LaunchDescription([DeclareLaunchArgument("params_file",default_value=str(share()/filename)),
        OpaqueFunction(function=lambda c:compose(c,kind))])
def compose(context,kind):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    path=Path(context.launch_configurations["params_file"])
    if not path.is_file():raise ValueError("Navigation configuration missing")
    if kind=="navigation":
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share("nav2_bringup")/"launch/navigation_launch.py")),
                                         launch_arguments={"params_file":str(path)}.items())]
    if kind=="bridge":
        return [Node(package="ebva_navigation",executable="cmd_vel_to_ackermann_node",name="cmd_vel_to_ackermann_node",
                     parameters=[str(path)],output="screen")]
    raise ValueError("Unknown navigation unit")

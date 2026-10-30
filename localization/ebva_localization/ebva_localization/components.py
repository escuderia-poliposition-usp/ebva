"""Localization-only launches; acquisition and authority belong to other packages."""
from pathlib import Path
import yaml
def share():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("ebva_localization"))
def description(kind,filename):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument,OpaqueFunction
    return LaunchDescription([DeclareLaunchArgument("params_file",default_value=str(share()/filename)),
        OpaqueFunction(function=lambda c:compose(c,kind))])
def compose(context,kind):
    from launch_ros.actions import Node
    path=Path(context.launch_configurations["params_file"]);data=yaml.safe_load(path.read_text())
    if kind.startswith("mapping"):
        return [Node(package="slam_toolbox",executable="async_slam_toolbox_node",name="slam_toolbox",parameters=[str(path)],output="screen")]
    if kind=="slam_localization":
        params=data["slam_toolbox"]["ros__parameters"]
        basename=Path(params.get("map_file_name","")).expanduser()
        if not params.get("map_file_name") or not all(Path(str(basename)+suffix).is_file() for suffix in (".posegraph",".data")):
            raise ValueError("Set an existing serialized map basename in the localization package YAML")
        return [Node(package="slam_toolbox",executable="localization_slam_toolbox_node",name="slam_toolbox",parameters=[str(path)],output="screen")]
    if kind=="ekf":
        return [Node(package="robot_localization",executable="ekf_node",name="ekf_filter_node",parameters=[str(path)],output="screen")]
    if kind=="imu_prior":
        return [Node(package="ebva_localization",executable="legacy_imu_prior",name="imu_odometry",parameters=[str(path)],output="screen")]
    if kind=="amcl":
        filename=data["map_server"]["ros__parameters"].get("yaml_filename","")
        if not filename or not Path(filename).expanduser().is_file():
            raise ValueError("Set map_server.yaml_filename in the localization package YAML")
        return [Node(package="nav2_map_server",executable="map_server",name="map_server",parameters=[str(path)],output="screen"),
                Node(package="nav2_amcl",executable="amcl",name="amcl",parameters=[str(path)],output="screen"),
                Node(package="nav2_lifecycle_manager",executable="lifecycle_manager",name="lifecycle_manager_localization",parameters=[str(path)],output="screen")]
    raise ValueError("Unknown localization unit")

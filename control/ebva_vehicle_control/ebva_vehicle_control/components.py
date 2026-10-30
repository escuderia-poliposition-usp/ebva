"""Vehicle-owned functional launches. No bringup dependency."""
from pathlib import Path
import yaml

def default_path(filename):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("ebva_vehicle_control"))/filename

def description(kind, filename):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    return LaunchDescription([DeclareLaunchArgument("params_file", default_value=str(default_path(filename))),
        OpaqueFunction(function=lambda context: compose(context, kind))])

def compose(context, kind):
    from launch_ros.actions import Node
    from launch.actions import RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    path=Path(context.launch_configurations["params_file"]).expanduser()
    if kind=="manual":
        from .manual import inspect_profile
        from .runtime_guards import running_conflicts
        report,params,_=inspect_profile(hardware=True,check_devices=True,params_file=path)
        namespace=context.launch_configurations.get("ros_namespace","").rstrip("/")
        running_conflicts([namespace+"/"+row[2] for row in report["nodes"]]+
                          ["/rc_reader_node","/ackermann_to_vesc_node","/ackermann_mux","/vesc_driver_node"])
        nodes=[Node(package=p,executable=e,name=n,parameters=[params[n]],output="screen")
               for p,e,n in report["nodes"]]
    elif kind=="vesc_driver":
        from .driver_only import inspect_driver, conflicting_nodes
        from .runtime_guards import running_conflicts
        params = inspect_driver(path, check_device=True)
        namespace = context.launch_configurations.get("ros_namespace", "").rstrip("/")
        running_conflicts(conflicting_nodes(namespace))
        nodes = [Node(package="vesc_driver", executable="vesc_driver_node",
                      name="vesc_driver_node", parameters=[params], output="screen")]
    elif kind=="wheel_odometry":
        data=yaml.safe_load(path.read_text())
        nodes=[Node(package="vesc_ackermann",executable="vesc_to_odom_node",name="vesc_to_odom_node",
                    parameters=[data["vesc_to_odom_node"]["ros__parameters"]],
                    remappings=[("odom",data["output_topic"])],output="screen")]
    elif kind=="navigation_adapter":
        nodes=[Node(package="ebva_vehicle_control",executable="navigation_adapter",name="navigation_adapter",
                    parameters=[str(path)],output="screen")]
    else:
        raise ValueError("Unknown vehicle component")
    return [RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[
              EmitEvent(event=Shutdown(reason="Vehicle component exited"))])) for n in nodes]+nodes

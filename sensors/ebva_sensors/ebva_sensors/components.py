"""Sensor functional units and lifecycle management; all values come from this package."""
from pathlib import Path
import yaml
def share(package="ebva_sensors"):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))
def description(kind,filename):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument,OpaqueFunction
    return LaunchDescription([DeclareLaunchArgument("params_file",default_value=str(share()/filename)),
        OpaqueFunction(function=lambda c:compose(c,kind))])
def compose(context,kind):
    from launch_ros.actions import Node,LifecycleNode
    from launch.actions import EmitEvent,RegisterEventHandler,IncludeLaunchDescription
    from launch.event_handlers import OnProcessStart,OnProcessExit
    from launch.events import Shutdown
    from launch_ros.event_handlers import OnStateTransition
    from launch_ros.events.lifecycle import ChangeState
    from lifecycle_msgs.msg import Transition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    path=Path(context.launch_configurations["params_file"]);data=yaml.safe_load(path.read_text())
    if kind.startswith("lidar"):
        driver=LifecycleNode(package="urg_node2",executable="urg_node2_node",name="urg_node",
                             namespace="",parameters=[data["urg_node"]["ros__parameters"]],output="screen")
        actions=[RegisterEventHandler(OnProcessStart(target_action=driver,on_start=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda n:n is driver,transition_id=Transition.TRANSITION_CONFIGURE))])),
            RegisterEventHandler(OnStateTransition(target_lifecycle_node=driver,start_state="configuring",goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=lambda n:n is driver,transition_id=Transition.TRANSITION_ACTIVATE))]))]
        nodes=[driver]
        if kind=="lidar":
            nodes.append(Node(package="ebva_sensors",executable="laser_filer_escuderia",name="scan_filter",
                              parameters=[data["scan_filter"]["ros__parameters"]],output="screen"))
        tf=data["static_transform"]
        arguments=[arg for key in ("x","y","z","roll","pitch","yaw") for arg in ("--"+key,str(tf[key]))]
        arguments+=["--frame-id",tf["parent"],"--child-frame-id",tf["child"]]
        nodes.append(Node(package="tf2_ros",executable="static_transform_publisher",name="lidar_mount",arguments=arguments,output="screen"))
        actions.extend(RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason="Sensor component exited"))])) for n in nodes)
        return actions+nodes
    if kind.startswith("zed"):
        ref=data["ros_parameters_file"];override=share(ref["package"])/ref["file"]
        if not override.is_file():raise ValueError("ZED parameter resource missing: "+str(override))
        args={k:str(v).lower() if isinstance(v,bool) else str(v) for k,v in data["launch_arguments"].items()}
        args["ros_params_override_path"]=str(override)
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share("zed_wrapper")/"launch/zed_camera.launch.py")),launch_arguments=args.items())]
    if kind=="imu_si":
        return [Node(package="ebva_sensors",executable="vesc_imu_to_si",name="vesc_imu_to_si",parameters=[str(path)],output="screen")]
    raise ValueError("Unknown sensor unit")

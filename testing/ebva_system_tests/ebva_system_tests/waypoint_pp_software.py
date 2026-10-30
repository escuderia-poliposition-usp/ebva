from .resources import fixture_path
"""Waypoint geometry consumes an exact optional Odometry pose projection in software."""
import math
from .software_profile import load_profile,validate_arguments,launch_profile
NAMESPACE="waypoint_pp_software"
SPECS=(("ebva_localization","odometry_pose","odometry_pose"),
       ("ebva_path_control","waypoint_pp","waypoint_pp"),
       ("ebva_vehicle_control","ackermann_adapter","ackermann_adapter"),
       ("ebva_vehicle_control","command_authority","command_authority"),
       ("ebva_vehicle_control","vesc_backend","vesc_backend"))


def inspect_profile(root=None,get_prefix=None):
    report,p,root=load_profile(NAMESPACE,SPECS,root,get_prefix)
    pose,node,adapter=(p[k] for k in ("odometry_pose","waypoint_pp","ackermann_adapter"))
    if (pose["parent_frame"]!=node["path_frame_id"] or pose["child_frame"]!=node["body_frame_id"]
            or node["body_frame_id"]!=adapter["expected_frame"] or node["body_frame_id"]!="fixture_rear_axle_flu"
            or node["path_frame_id"]!="fixture_map" or node["model_point"]!="software_fixture_rear_axle"
            or node["path_contract"]!="ordered_closed_xy_si_v1" or node["pose_contract"]!="cartesian_body_pose_si_v1"
            or node["path_origin"]!="shared_reference_frame" or node["path_direction"]!="csv_order"):
        raise ValueError("Explicit Cartesian frame/body/model-point contract required")
    if (pose["input_topic"]!="state/odometry" or pose["output_topic"]!=node["pose_topic"]
            or node["pose_topic"]!="state/pose" or node["command_topic"]!="control/autonomy/ackermann"):
        raise ValueError("Pose chain topics do not match")
    if (not math.isfinite(node["observation_timeout"]) or node["observation_timeout"]<=0
            or pose["max_age_sec"]>node["observation_timeout"]):raise ValueError("Pose/controller timing mismatch")
    from dataclasses import fields
    from ebva_vehicle_control.ackermann import AckermannMapping
    AckermannMapping(**{f.name:adapter[f.name] for f in fields(AckermannMapping)})
    from ebva_localization.state_contract import PosePolicy
    PosePolicy(pose["parent_frame"],pose["child_frame"],pose["max_age_sec"],pose["future_tolerance_sec"],
        pose.get("quaternion_norm_tolerance",1e-6),pose.get("covariance_tolerance",1e-10))
    if (not math.isfinite(node["future_slack"]) or not 0<=node["future_slack"]<node["observation_timeout"]
            or pose["future_tolerance_sec"]!=node["future_slack"]):
        raise ValueError("Pose/controller future policy mismatch")
    state=report["state_contract"]
    if state["future_tolerance_sec"]!=pose["future_tolerance_sec"]:
        raise ValueError("Declared future tolerance differs from runtime")
    if state["require_tf"] or state["edges"]:
        raise ValueError("This fixture passes coherent pose directly without consuming TF")
    expected={"source":("nav_msgs/Odometry","/rosbag2_player","state/odometry"),
              "pose":("geometry_msgs/PoseWithCovarianceStamped","/"+NAMESPACE+"/odometry_pose","state/pose")}
    if {port["name"] for port in state["state_ports"]}!=set(expected):raise ValueError("Unexpected state ports")
    for port in state["state_ports"]:
        type_name,publisher,topic=expected[port["name"]]
        if (port["type"]!=type_name or port["publisher"]!=publisher or port["topic"]!="/"+NAMESPACE+"/"+topic
                or port["frame"]!=node["path_frame_id"] or port["body"]!=node["body_frame_id"]
                or port["model_point"]!=node["model_point"] or port["max_age_sec"]!=pose["max_age_sec"]):
            raise ValueError("Declared pose port does not match composition")
    if {c["name"] for c in state["consumers"]}!={"odometry_pose","waypoint_pp"}:raise ValueError("Missing pose-chain consumers")
    for consumer in state["consumers"]:
        expected_age=pose["max_age_sec"] if consumer["name"]=="odometry_pose" else node["observation_timeout"]
        if consumer["max_age_sec"]!=expected_age:raise ValueError("Declared consumer age differs from runtime")
    reference=fixture_path("waypoint_pp_reference.csv",root)
    if not reference.is_file():raise ValueError("Missing installed waypoint reference")
    report["reference_file"]=str(reference)
    return report


def compose(context):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile())

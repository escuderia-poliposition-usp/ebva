"""One coherent CBF yaw-rate model, synthetic nominal and existing RC authority."""
import math
from .software_profile import load_profile,validate_arguments,launch_profile
NAMESPACE="cbf_yaw_rate_software"
SPECS=(("ebva_cbf_control","yaw_rate_stamped","cbf_yaw_rate"),
       ("ebva_vehicle_control","cbf_yaw_rate_adapter","cbf_yaw_rate_adapter"),
       ("ebva_vehicle_control","command_authority","command_authority"),
       ("ebva_vehicle_control","vesc_backend","vesc_backend"))


def inspect_profile(root=None,get_prefix=None):
    report,p,_=load_profile(NAMESPACE,SPECS,root,get_prefix,"cbf_yaw_rate_adapter")
    node,adapter=p["cbf_yaw_rate"],p["cbf_yaw_rate_adapter"]
    if (node["control_frame"]!=adapter["expected_frame"] or node["velocity_parent_frame"]!=adapter["velocity_parent_frame"]
            or node["model_reference_point"]!="rear_axle_center" or node["scan_geometry"]!="coincident_aligned"
            or adapter["expected_frame"]!="fixture_rear_axle_flu" or adapter["velocity_parent_frame"]!="fixture_odom"):
        raise ValueError("Explicit aligned model-point fixture required")
    if (not math.isfinite(node["timeout_s"]) or node["timeout_s"]<=0 or node["max_skew_s"]!=adapter["max_input_skew_s"]
            or not 0<=node["future_tolerance_s"]<node["timeout_s"]):raise ValueError("CBF timing contract mismatch")
    from dataclasses import fields
    from ebva_vehicle_control.cbf_yaw_rate_mapping import CbfYawRateMapping
    CbfYawRateMapping(**{f.name:adapter[f.name] for f in fields(CbfYawRateMapping)})
    if (adapter["model_wheelbase_m"]!=.33 or adapter["max_acceleration_mps2"]!=3.
            or abs(adapter["max_yaw_rate_rps"]-1./6.)>1e-12
            or adapter["positive_acceleration_cap_mps2"]!=.24 or adapter["speed_floor_mps"]!=.1
            or abs(adapter["max_steering_rad"]-math.pi/6)>1e-12):
        raise ValueError("Mapping must match the selected yaw-rate model projection")
    state=report["state_contract"]
    if state["require_tf"] or state["edges"] or len(state["state_ports"])!=1 or len(state["consumers"])!=1:
        raise ValueError("This fixture uses one direct state port, without TF")
    port=state["state_ports"][0];consumer=state["consumers"][0]
    if (len(state["state_ports"])!=1 or len(state["consumers"])!=1
            or port["frame"]!=node["velocity_parent_frame"] or port["body"]!=node["control_frame"]
            or port["model_point"]!=node["model_reference_point"] or port["max_age_sec"]!=node["timeout_s"]
            or port["topic"]!="/"+NAMESPACE+"/state/odometry" or port["publisher"]!="/rosbag2_player"
            or consumer["name"]!="cbf_yaw_rate" or consumer["needs"]!=["longitudinal_velocity"]):
        raise ValueError("Declared CBF state port must match actual composition")
    return report


def compose(context):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile())

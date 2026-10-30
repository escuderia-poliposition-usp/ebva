"""Separate explicit fixture compositions for margin and steering-tangent CBF."""
import math
from dataclasses import fields
from .software_profile import load_profile,validate_arguments,launch_profile

MODELS=("yaw_rate_margin","steering_tangent")
def specs(model):
    if model not in MODELS:raise ValueError("Unknown CBF model")
    node="cbf_"+model;adapter=node+"_adapter"
    return (("ebva_cbf_control",model+"_stamped",node),
        ("ebva_vehicle_control",adapter,adapter),("ebva_vehicle_control","command_authority","command_authority"),
        ("ebva_vehicle_control","vesc_backend","vesc_backend"))

def inspect_profile(model,root=None,get_prefix=None):
    namespace="cbf_"+model+"_software";name="cbf_"+model;adapter_name=name+"_adapter"
    report,p,_=load_profile(namespace,specs(model),root,get_prefix,adapter_name)
    node,adapter=p[name],p[adapter_name]
    if (node["control_frame"]!=adapter["expected_frame"] or node["velocity_parent_frame"]!=adapter["velocity_parent_frame"]
            or node["model_reference_point"]!="rear_axle_center" or node["scan_geometry"]!="coincident_aligned"
            or adapter["expected_frame"]!="fixture_rear_axle_flu" or adapter["velocity_parent_frame"]!="fixture_odom"):
        raise ValueError("Explicit coherent scan/body/reference-point required")
    age,future,skew=(node[k] for k in ("timeout_s","future_tolerance_s","max_skew_s"))
    if (not all(math.isfinite(v) for v in (age,future,skew)) or age<=0 or not 0<=future<age
            or not 0<skew<=age or age!=adapter["source_timeout_s"] or skew!=adapter["max_input_skew_s"] or future!=.02):
        raise ValueError("CBF source/adapter time policy mismatch")
    if model=="yaw_rate_margin":
        from ebva_vehicle_control.cbf_yaw_rate_mapping import CbfYawRateMapping as Mapping
        expected={"model_wheelbase_m":.33,"speed_floor_mps":.1,"positive_acceleration_cap_mps2":.24,
            "max_acceleration_mps2":3.,"max_yaw_rate_rps":1./6.,"max_steering_rad":math.pi/6}
    else:
        from ebva_vehicle_control.cbf_alternate_mapping import CbfSteeringTangentMapping as Mapping
        expected={"positive_acceleration_cap_mps2":.24,"max_acceleration_mps2":3.,
            "max_steering_tangent":.4,"max_steering_rad":math.pi/9}
    Mapping(**{f.name:adapter[f.name] for f in fields(Mapping)})
    if any(abs(adapter[k]-v)>1e-12 for k,v in expected.items()):
        raise ValueError("Projection does not match selected model")
    state=report["state_contract"]
    if state["require_tf"] or state["edges"] or len(state["state_ports"])!=1 or len(state["consumers"])!=1:
        raise ValueError("One direct velocity port without artificial TF required")
    port,consumer=state["state_ports"][0],state["consumers"][0]
    if (port["type"]!="nav_msgs/Odometry" or port["frame"]!=node["velocity_parent_frame"]
            or port["body"]!=node["control_frame"] or port["model_point"]!=node["model_reference_point"]
            or port["max_age_sec"]!=age or port["topic"]!="/"+namespace+"/state/odometry"
            or port["publisher"]!="/rosbag2_player" or port["provides"]!=["longitudinal_velocity"]
            or consumer["name"]!=name or consumer["needs"]!=["longitudinal_velocity"]
            or consumer["max_age_sec"]!=age or state["future_tolerance_sec"]!=future):
        raise ValueError("State declaration differs from actual selected model")
    report["cbf_model"]=model
    report["longitudinal_conversion"]="artificial_acceleration_gain_not_speed_tracking"
    return report

def compose(context, model):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile(model))

"""Fixed-zero-reference Stanley with two independent measured inputs in software."""
import math
from dataclasses import fields
from .software_profile import load_profile,validate_arguments,launch_profile
NAMESPACE="stanley_frenet_software"
SPECS=(("ebva_path_control","stanley_frenet","stanley_frenet"),
    ("ebva_vehicle_control","ackermann_adapter","ackermann_adapter"),
    ("ebva_vehicle_control","command_authority","command_authority"),
    ("ebva_vehicle_control","vesc_backend","vesc_backend"))

def inspect_profile(root=None,get_prefix=None):
    report,p,_=load_profile(NAMESPACE,SPECS,root,get_prefix)
    node,adapter=p["stanley_frenet"],p["ackermann_adapter"]
    expected={"reference_frame_id":"fixture_map","body_frame_id":"fixture_rear_axle_flu",
        "model_point":"software_fixture_rear_axle","reference_id":"fixture_straight_corridor",
        "reference_contract":"stanley_frenet_clearance_fixed_zero_v1"}
    if any(node.get(k)!=v for k,v in expected.items()):raise ValueError("Explicit clearance/reference/body/point required")
    age,future,skew=(node[k] for k in ("observation_timeout","future_slack","maximum_skew"))
    if (not all(math.isfinite(v) for v in (age,future,skew)) or age<=0 or not 0<=future<age
            or not 0<skew<age or age!=adapter["source_timeout_s"] or future!=.02):
        raise ValueError("Stanley paired observation timing mismatch")
    from ebva_path_control.stanley_frenet.contract import StanleyFrenetContract
    StanleyFrenetContract(**expected,observation_timeout=age,future_slack=future,maximum_skew=skew)
    from ebva_vehicle_control.ackermann import AckermannMapping
    AckermannMapping(**{f.name:adapter[f.name] for f in fields(AckermannMapping)})
    if adapter["max_speed_mps"]<.6 or adapter["max_steering_rad"]<.4:
        raise ValueError("Map must cover signed Stanley target domain")
    state=report["state_contract"]
    if len(state["state_ports"])!=1 or len(state["consumers"])!=1:
        raise ValueError("One direct odometry state port required")
    port=state["state_ports"][0];consumer=state["consumers"][0]
    if (len(state["state_ports"])!=1 or len(state["consumers"])!=1 or state["require_tf"] or state["edges"]
            or state["future_tolerance_sec"]!=future or port["topic"]!="/"+NAMESPACE+"/state/odometry"
            or port["publisher"]!="/rosbag2_player" or port["max_age_sec"]!=age
            or port["model_point"]!=node["model_point"] or port["frame"]!=node["reference_frame_id"]
            or port["body"]!=node["body_frame_id"] or port["provides"]!=["pose","longitudinal_velocity","lateral_velocity"]
            or consumer["name"]!="stanley_frenet" or consumer["max_age_sec"]!=age
            or consumer["needs"]!=port["provides"]):raise ValueError("State declaration differs from Stanley input")
    report["observation_contract"]={**expected,"type":"ebva_control_msgs/FrenetLateralErrorStamped",
        "topic":"/"+NAMESPACE+"/state/lateral_error","publisher":"/rosbag2_player",
        "units":"m_half_left_minus_right_clearance","max_age_sec":age,"maximum_skew":skew}
    report["signed_model_speed_mps"]=-.6
    return report

def compose(context):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile())

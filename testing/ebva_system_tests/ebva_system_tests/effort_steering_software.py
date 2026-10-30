from .resources import fixture_path
"""Two independent effort/angle fixture profiles; no speed inferred from effort."""
import math
from dataclasses import fields
from .software_profile import load_profile, validate_arguments, launch_profile

METHODS=("waypoint_control", "lidar_frenet_pp")

def specs(method):
    if method not in METHODS: raise ValueError("Unknown effort/angle fixture method")
    return (("ebva_path_control",method,method),
        ("ebva_vehicle_control","effort_steering_adapter","effort_steering_adapter"),
        ("ebva_vehicle_control","command_authority","command_authority"),
        ("ebva_vehicle_control","vesc_backend","vesc_backend"))

def inspect_profile(method, root=None, get_prefix=None):
    namespace=method+"_software"
    frame="fixture_map" if method=="waypoint_control" else "fixture_lidar_flu"
    report,p,root=load_profile(namespace,specs(method),root,get_prefix,
        adapter_name="effort_steering_adapter",command_frame=frame,state_required=method=="waypoint_control")
    controller,adapter=p[method],p["effort_steering_adapter"]
    age,slack=controller["observation_timeout"],controller["future_slack"]
    if not math.isfinite(age) or age<=0 or not math.isfinite(slack) or not 0<=slack<age:
        raise ValueError("Explicit finite measurement lease required")
    if age!=adapter["source_timeout_s"] or slack!=.02:
        raise ValueError("Controller/adapter time policy mismatch")
    from ebva_vehicle_control.effort_steering import EffortSteeringMapping
    EffortSteeringMapping(**{f.name:adapter[f.name] for f in fields(EffortSteeringMapping)})
    if method=="waypoint_control":
        mode=controller["control_mode"]
        if mode not in ("carrot","stanley","pure_pursuit"):
            raise ValueError("Select an implemented waypoint law")
        contract="waypoint_control_"+mode+"_v1"
        expected={"reference_frame_id":"fixture_map","body_frame_id":"fixture_rear_axle_flu",
            "model_point":"software_fixture_rear_axle","reference_contract":"ordered_xy_si_v1",
            "reference_origin":"shared_reference_frame","reference_direction":"csv_order",
            "pose_contract":"cartesian_body_pose_si_v1"}
        if any(controller.get(k)!=v for k,v in expected.items()):
            raise ValueError("Explicit coherent waypoint reference/body/point required")
        state=report["state_contract"]
        port={"name":"source","type":"nav_msgs/Odometry","role":"local","frame":"fixture_map",
            "body":"fixture_rear_axle_flu","model_point":"software_fixture_rear_axle","stamp":"measurement",
            "provides":["pose"],"topic":"/"+namespace+"/state/odometry","publisher":"/rosbag2_player",
            "max_age_sec":age,"twist_frame":"fixture_rear_axle_flu"}
        consumer={"name":method,"port":"source","type":port["type"],"frame":port["frame"],
            "body":port["body"],"model_point":port["model_point"],"needs":["pose"],"max_age_sec":age}
        if (state["require_tf"] or state["edges"] or state["state_ports"]!=[port]
                or state["consumers"]!=[consumer] or state["future_tolerance_sec"]!=slack):
            raise ValueError("Declared state ports differ from actual direct-pose consumer")
        reference=fixture_path("waypoint_control_reference.csv",root)
        from ebva_path_control.effort_contract import load_effort_waypoints
        # The controller loads this installed resource too; no foreign package lookup.
        load_effort_waypoints(reference)
        report["reference_file"]=str(reference)
    else:
        contract="lidar_frenet_pp_v1"
        if controller["scan_frame_id"]!=frame or controller["scan_contract"]!="planar_body_flu_ranges_m_v1":
            raise ValueError("Explicit scan alignment and range units required")
    if adapter["expected_contract"]!=contract:
        raise ValueError("Controller law/adapter dialect mismatch")
    report["controller_contract"]=contract
    report["command_header_frame"]=frame
    report["longitudinal_units"]="dimensionless_effort"
    report["steering_units"]="rad"
    return report

def compose(context, method):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile(method))

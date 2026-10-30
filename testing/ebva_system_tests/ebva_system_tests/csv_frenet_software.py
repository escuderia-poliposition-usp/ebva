"""CSV/Frenet reference controller composed with the existing RC authority in software."""
import json
import math
import os
from pathlib import Path
import yaml
from .resources import share, fixture_data, fixture_path, profile_path

NAMESPACE = "csv_frenet_software"
SPECS = (("ebva_path_control", "csv_frenet_pp", "csv_frenet_pp"),
         ("ebva_vehicle_control", "ackermann_adapter", "ackermann_adapter"),
         ("ebva_vehicle_control", "command_authority", "command_authority"),
         ("ebva_vehicle_control", "vesc_backend", "vesc_backend"))


def inspect_profile(root=None, get_prefix=None):
    root = Path(root) if root else share()
    path = root / "data/csv_frenet_software.yaml"
    data = fixture_data(path.stem, root)
    if set(data) != {"/" + NAMESPACE + "/" + name for _, _, name in SPECS}:
        raise ValueError("Only the four software nodes are allowed")
    params = {name:data["/"+NAMESPACE+"/"+name]["ros__parameters"] for _, _, name in SPECS}
    controller, bridge, authority, backend = (params[name] for name in
        ("csv_frenet_pp", "ackermann_adapter", "command_authority", "vesc_backend"))
    contract = {"reference_frame_id":"fixture_map", "body_frame_id":"fixture_rear_axle_flu",
        "model_point":"software_fixture_rear_axle", "reference_contract":"closed_progress_si_v1",
        "progress_origin":"first_accepted_odometry_s0", "progress_direction":"path_order"}
    if any(controller.get(key) != value for key,value in contract.items()):
        raise ValueError("Explicit CSV/Frenet software reference contract required")
    if (bridge["mapping_purpose"] != "software_fixture" or not bridge["calibration_id"]
            or bridge["expected_frame"] != controller["body_frame_id"]):
        raise ValueError("Explicit fixture mapping and model point required")
    for key in ("rc_timeout_s", "autonomy_timeout_s", "rc_center_us", "override_enter_us", "override_release_us"):
        if authority[key] != backend[key]:
            raise ValueError("Authority/backend mismatch: " + key)
    if not authority["enforce_graph"] or not backend["enforce_graph"] or authority["rc_publisher_name"] != "rosbag2_player":
        raise ValueError("Replay RC must have explicit ownership")
    for key in ("erpm_limit", "servo_min", "servo_max", "servo_center"):
        if bridge[key] != backend[key]:
            raise ValueError("Adapter/backend bounds mismatch")
    for value in (controller["observation_timeout"], bridge["source_timeout_s"], backend["backend_timeout_s"]):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("Finite positive timeouts required")
    slack=controller["future_slack"]
    if not math.isfinite(slack) or slack < 0 or slack >= controller["observation_timeout"]:
        raise ValueError("Future tolerance must be finite and smaller than the lease")
    reference = fixture_path("csv_frenet_reference.csv",root)
    if not reference.is_file():
        raise ValueError("Missing installed synthetic reference")
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix = get_package_prefix
    for package, executable, _ in SPECS:
        target = Path(get_prefix(package)) / "lib" / package / executable
        if not target.is_file() or not os.access(target, os.X_OK):
            raise ValueError("Missing installed executable: " + str(target))
    return {"profile":"csv_frenet_software", "namespace":NAMESPACE, "profile_file":profile_path(NAMESPACE),
        "reference_file":str(reference), "nodes":[list(spec) for spec in SPECS],
        "physical_actuation":False, "devices":[], "calibration_qualification":"software_fixture",
        "reference_contract":contract, "rc_provider":"rosbag2_player",
        "command_contract":{"status":"declared_software_only", "final_writer":"vesc_backend",
            "neutral_resume":"automatic_if_inputs_valid"},
        "final_topics":["/"+NAMESPACE+"/commands/motor/speed", "/"+NAMESPACE+"/commands/servo/position"]}


def compose(context):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile())

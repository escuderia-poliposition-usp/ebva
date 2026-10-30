"""Explicit software-only FRM control profile; no serial, sensors or VESC driver."""
import json
import math
import os
from pathlib import Path
import yaml
from .resources import share, fixture_data, fixture_path, profile_path

NAMESPACE="frm_control_software"
SPECS=(("ebva_frm_control","point_sample","frm_point_sample"),
       ("ebva_vehicle_control","frm_effort_adapter","frm_effort_adapter"),
       ("ebva_vehicle_control","command_authority","command_authority"),
       ("ebva_vehicle_control","vesc_backend","vesc_backend"))


def inspect_profile(root=None, get_prefix=None):
    root=Path(root) if root else share()
    path=root/"data/frm_control_software.yaml"
    data=fixture_data(path.stem, root)
    expected={"/"+NAMESPACE+"/"+name for _,_,name in SPECS}
    if set(data)!=expected:
        raise ValueError("Software profile must contain only its four declared nodes")
    params={name:data["/"+NAMESPACE+"/"+name]["ros__parameters"] for _,_,name in SPECS}
    lane,bridge,authority,backend=(params[k] for k in ("frm_point_sample","frm_effort_adapter","command_authority","vesc_backend"))
    if (bridge["mapping_purpose"]!="software_fixture" or not bridge["calibration_id"]
            or lane["expected_scan_frame"]!=bridge["expected_frame"]
            or bridge["expected_frame"]!="fixture_lidar_flu"):
        raise ValueError("Explicit FRM fixture frame/calibration required")
    for key in ("rc_timeout_s","autonomy_timeout_s","rc_center_us","override_enter_us","override_release_us"):
        if authority[key]!=backend[key]:
            raise ValueError("Authority/backend mismatch: "+key)
    if not authority["enforce_graph"] or not backend["enforce_graph"] or authority["rc_publisher_name"]!="rosbag2_player":
        raise ValueError("Explicit replay RC ownership required")
    for key in ("erpm_limit","servo_min","servo_max","servo_center"):
        if bridge[key]!=backend[key]:
            raise ValueError("Adapter/backend bounds mismatch")
    for value in (lane["source_timeout_s"],bridge["source_timeout_s"],backend["backend_timeout_s"]):
        if not math.isfinite(value) or value<=0:
            raise ValueError("Finite positive source/lease timeouts required")
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix=get_package_prefix
    for package,executable,_ in SPECS:
        target=Path(get_prefix(package))/"lib"/package/executable
        if not target.is_file() or not os.access(target,os.X_OK):
            raise ValueError("Missing installed executable: "+str(target))
    return {"profile":"frm_control_software","namespace":NAMESPACE,"profile_file":profile_path(NAMESPACE),
            "nodes":[list(spec) for spec in SPECS],"physical_actuation":False,"devices":[],
            "calibration_qualification":"software_fixture","rc_provider":"rosbag2_player",
            "command_contract":{"status":"declared_software_only","final_writer":"vesc_backend",
                                "neutral_resume":"automatic_if_inputs_valid"},
            "final_topics":["/"+NAMESPACE+"/commands/motor/speed","/"+NAMESPACE+"/commands/servo/position"]}


def compose(context):
    from .software_profile import validate_arguments, launch_profile
    values = validate_arguments(context)
    return launch_profile(values, inspect_profile())

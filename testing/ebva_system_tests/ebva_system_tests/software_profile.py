"""Shared construction for explicit software profiles with declared state ports."""
import json
import math
import os
from pathlib import Path
import yaml
from .resources import share, fixture_data, fixture_path, profile_path


def load_profile(namespace, specs, root=None, get_prefix=None, adapter_name="ackermann_adapter", command_frame=None, state_required=True):
    root=Path(root) if root else share()
    path=root/("data/"+namespace+".yaml")
    data=fixture_data(path.stem, root)
    if set(data)!={"/"+namespace+"/"+name for _,_,name in specs}:
        raise ValueError("Unexpected software node selection")
    params={name:data["/"+namespace+"/"+name]["ros__parameters"] for _,_,name in specs}
    bridge,authority,backend=(params[n] for n in (adapter_name,"command_authority","vesc_backend"))
    if bridge["mapping_purpose"]!="software_fixture" or not bridge["calibration_id"]:
        raise ValueError("Explicit software fixture map required")
    for key in ("rc_timeout_s","autonomy_timeout_s","rc_center_us","override_enter_us","override_release_us"):
        if authority[key]!=backend[key]:raise ValueError("Authority/backend mismatch: "+key)
    if not authority["enforce_graph"] or not backend["enforce_graph"] or authority["rc_publisher_name"]!="rosbag2_player":
        raise ValueError("Explicit replay RC ownership required")
    for key in ("erpm_limit","servo_min","servo_max","servo_center"):
        if bridge[key]!=backend[key]:raise ValueError("Adapter/backend bounds mismatch")
    for value in (bridge["source_timeout_s"],backend["backend_timeout_s"],authority["rc_timeout_s"],authority["autonomy_timeout_s"]):
        if not math.isfinite(value) or value<=0:raise ValueError("Finite positive timeouts required")
    state=None
    if state_required:
        from ebva_localization.tf_contract import validate_profile
        state_path=root/("data/"+namespace+"_state.json")
        state=validate_profile(json.loads(state_path.read_text()))
        if state["profile_id"]!=namespace or (command_frame or state["body_frame"])!=bridge["expected_frame"]:
            raise ValueError("State declaration/profile command frame mismatch")
    elif command_frame!=bridge["expected_frame"]:
        raise ValueError("Explicit measurement command frame required")
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix=get_package_prefix
    for package,executable,_ in specs:
        target=Path(get_prefix(package))/"lib"/package/executable
        if not target.is_file() or not os.access(target,os.X_OK):raise ValueError("Missing installed executable: "+str(target))
    return ({"profile":namespace,"namespace":namespace,"profile_file":profile_path(namespace),"nodes":[list(s) for s in specs],
        "physical_actuation":False,"devices":[],"calibration_qualification":"software_fixture","rc_provider":"rosbag2_player",
        "state_contract":state,"command_contract":{"status":"declared_software_only","final_writer":"vesc_backend","neutral_resume":"automatic_if_inputs_valid"},
        "final_topics":["/"+namespace+"/commands/motor/speed","/"+namespace+"/commands/servo/position"]},params,root)


def validate_arguments(context):
    values=context.launch_configurations
    if values.get("hardware","false")!="false":raise ValueError("This fixture profile has no hardware mode")
    if any(values.get(k,d) not in ("true","false") for k,d in (("check_only","false"),("use_sim_time","true"))):
        raise ValueError("Expected boolean check_only/use_sim_time")
    return values


def launch_profile(values, report, overrides=None, remappings=None):
    from launch.actions import LogInfo
    from ebva_bringup.profiles import inspect_profile as inspect_orchestration, actions_for
    selected = inspect_orchestration(profile_path(report["profile"]))
    if values.get("check_only","false") == "true":
        import json
        return [LogInfo(msg=json.dumps(report))]
    return actions_for(selected)

"""Portable bench launch configuration; no device access during preflight."""
import json
import math
import os
from pathlib import Path
import yaml
from .core import AuthorityPolicy, Mapping


def inspect_bench(params_file, hardware, get_prefix=None):
    path = Path(params_file).expanduser().resolve()
    if not path.is_file():
        raise ValueError("params_file must name an existing vehicle configuration")
    data = yaml.safe_load(path.read_text())
    params = {name: data[name]["ros__parameters"] for name in
              ("rc_acquisition", "command_authority", "vesc_backend", "vesc_driver_node")}
    authority, backend, driver = (params[name] for name in
                                 ("command_authority", "vesc_backend", "vesc_driver_node"))
    selected_policy = AuthorityPolicy(
        rc_timeout=authority["rc_timeout_s"], autonomy_timeout=authority["autonomy_timeout_s"],
        center=authority["rc_center_us"], enter=authority["override_enter_us"],
        release=authority["override_release_us"])
    mapping = Mapping(**{name: backend.get(name, value) for name, value in Mapping().__dict__.items()})
    if not math.isfinite(backend["backend_timeout_s"]) or backend["backend_timeout_s"] <= 0:
        raise ValueError("Invalid backend expiry")
    for key in ("rc_timeout_s", "autonomy_timeout_s", "rc_center_us", "override_enter_us", "override_release_us"):
        if backend[key] != authority[key]:
            raise ValueError("Authority/backend policy mismatch: " + key)
    if params["rc_acquisition"]["rc_timeout_s"] != authority["rc_timeout_s"]:
        raise ValueError("Acquisition/authority expiry mismatch")
    if hardware:
        if not authority["enforce_graph"] or not backend["enforce_graph"]:
            raise ValueError("Physical bench requires publisher ownership checks")
        if mapping.erpm_limit > 600 or mapping.servo_min < .4 or mapping.servo_max > .6:
            raise ValueError("Physical bench exceeds its bounded test envelope")
        if (driver["speed_min"] != -mapping.erpm_limit or driver["speed_max"] != mapping.erpm_limit
                or driver["servo_min"] != mapping.servo_min or driver["servo_max"] != mapping.servo_max):
            raise ValueError("Driver/backend limits must agree")
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix = get_package_prefix
    required = {"ebva_vehicle_control": ["command_authority", "vesc_backend"]}
    if hardware:
        required["ebva_vehicle_control"].append("rc_acquisition")
        required["vesc_driver"] = ["vesc_driver_node"]
    for package, executables in required.items():
        prefix = Path(get_prefix(package))
        for executable in executables:
            target = prefix/"lib"/package/executable
            if not target.is_file() or not os.access(target, os.X_OK):
                raise ValueError("Missing installed executable: " + str(target))
    return {"profile": "isolated_rc_bench", "hardware": hardware, "params_file": str(path),
            "command_contract": {"status": "declared", "rc_priority": True,
                                 "neutral_resume": "automatic_if_inputs_valid",
                                 "rc_timeout_s": selected_policy.rc_timeout,
                                 "autonomy_timeout_s": selected_policy.autonomy_timeout,
                                 "final_writer": "vesc_backend",
                                 "backend_timeout_s": backend["backend_timeout_s"]},
            "limits": {"erpm": mapping.erpm_limit, "servo": [mapping.servo_min, mapping.servo_max]},
            "physical_qualification": False}


def compose(context):
    from launch.actions import LogInfo, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch_ros.actions import Node
    values = context.launch_configurations
    if any(values[key] not in ("true", "false") for key in ("hardware", "check_only")):
        raise ValueError("Expected hardware/check_only booleans")
    hardware = values["hardware"] == "true"
    report = inspect_bench(values["params_file"], hardware)
    if values["check_only"] == "true":
        return [LogInfo(msg=json.dumps(report))]
    specs = [("ebva_vehicle_control", "command_authority"),
             ("ebva_vehicle_control", "vesc_backend")]
    if hardware:
        specs += [("ebva_vehicle_control", "rc_acquisition"), ("vesc_driver", "vesc_driver_node")]
    actions = [RegisterEventHandler(OnProcessExit(on_exit=[
        EmitEvent(event=Shutdown(reason="Critical bench process exited"))]))]
    actions += [Node(package=package, executable=executable, name=executable,
                     parameters=[report["params_file"]], output="screen")
                for package, executable in specs]
    return actions

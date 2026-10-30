"""Direct manual composition; importing or inspecting never opens a device."""
import json
import math
import os
from pathlib import Path
import stat
import yaml
from .resources import share

NAMESPACE = "ebva"
CORE = (("ebva_vehicle_control", "command_authority", "command_authority"),
        ("ebva_vehicle_control", "vesc_backend", "vesc_backend"))
ACQUISITION = ("ebva_vehicle_control", "rc_acquisition", "rc_acquisition")
DRIVER = ("vesc_driver", "vesc_driver_node", "vesc_driver_node")
LIDAR = ("urg_node2", "urg_node2_node", "urg_node")


def _bool(values, name, default="false"):
    value = values.get(name, default)
    if value not in ("true", "false"):
        raise ValueError("Expected true/false for " + name)
    return value == "true"


def inspect_profile(root=None, get_prefix=None, *, hardware=False, lidar=False,
                    rc_port="", vesc_port="", check_devices=False, params_file=None):
    """Check selected packages/config/routes; hardware ports are inspected, never opened."""
    
    root = Path(root) if root else share()
    path = Path(params_file) if params_file else root / "config/vehicle/manual_core.yaml"
    config = yaml.safe_load(path.read_text())
    names = {"rc_acquisition", "command_authority", "vesc_backend", "vesc_driver_node"}

    if set(config) != names or any(set(value) != {"ros__parameters"} for value in config.values()):
        raise ValueError("Unexpected manual core node configuration")

    params = {name: dict(config[name]["ros__parameters"]) for name in names}
    authority, backend, driver = (params[name] for name in
                                  ("command_authority", "vesc_backend", "vesc_driver_node"))

    if not authority.get("enforce_graph") or not backend.get("enforce_graph"):
        raise ValueError("Manual core requires publisher ownership checks")

    auto_enable = backend.get("auto_enable_on_startup", False)
    if not isinstance(auto_enable, bool):
        raise ValueError("auto_enable_on_startup must be a boolean")

    if authority.get("rc_publisher_name") != "rc_acquisition":
        raise ValueError("The physical RC provider must be rc_acquisition")

    for key in ("rc_timeout_s", "autonomy_timeout_s", "rc_center_us", "override_enter_us", "override_release_us"):
        if authority[key] != backend[key]:
            raise ValueError("Authority/backend mismatch: " + key)

    for value in (authority["rc_timeout_s"], authority["autonomy_timeout_s"],
                  backend["backend_timeout_s"], params["rc_acquisition"]["rc_timeout_s"]):
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError("Finite positive timeouts required")

    if params["rc_acquisition"]["rc_timeout_s"] != authority["rc_timeout_s"]:
        raise ValueError("Acquisition/authority RC lease mismatch")

    from ebva_vehicle_control.core import AuthorityPolicy, Mapping

    AuthorityPolicy(authority["rc_timeout_s"], authority["autonomy_timeout_s"],
                    authority["rc_center_us"], authority["override_enter_us"], authority["override_release_us"])
    Mapping(**{key: backend[key] for key in Mapping().__dict__})

    for key, expected in (("speed_min", -backend["erpm_limit"]), ("speed_max", backend["erpm_limit"]),
                          ("servo_min", backend["servo_min"]), ("servo_max", backend["servo_max"])):
        if driver[key] != expected:
            raise ValueError("Driver/backend bounds mismatch: " + key)

    for key in ("duty_cycle_min", "duty_cycle_max", "current_min", "current_max", "brake_min", "brake_max",
                "position_min", "position_max"):
        if driver[key] != 0.0:
            raise ValueError("Only speed/servo driver commands are selected: " + key)

    params["rc_acquisition"]["serial_port"] = rc_port or params["rc_acquisition"]["serial_port"]
    driver["port"] = vesc_port or driver["port"]
    ports = [params["rc_acquisition"]["serial_port"], driver["port"]]

    if any(not isinstance(port, str) or not port.startswith("/dev/") or ".." in Path(port).parts for port in ports):
        raise ValueError("RC/VESC ports must be explicit /dev paths")

    if Path(ports[0]).resolve() == Path(ports[1]).resolve():
        raise ValueError("RC and VESC must use different devices")

    if hardware and check_devices:
        for port in ports:
            if not Path(port).exists() or not stat.S_ISCHR(Path(port).stat().st_mode):
                raise ValueError("Serial device unavailable: " + port)

    if lidar and not hardware:
        raise ValueError("LiDAR requires hardware=true; software profile opens no device")

    if not hardware:
        authority["rc_publisher_name"] = "rosbag2_player"

    specs = [*CORE, *([ACQUISITION, DRIVER] if hardware else []), *([LIDAR] if lidar else [])]
    lidar_params = None

    if lidar:
        lidar_params = yaml.safe_load((share("ebva_sensors") / "config/sensors/sensors.yaml").read_text())["urg_node"]["ros__parameters"]
        if not lidar_params.get("ip_address") or not isinstance(lidar_params.get("ip_port"), int):
            raise ValueError("LiDAR endpoint must be explicit")

    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix = get_package_prefix

    for package, executable, _ in specs:
        target = Path(get_prefix(package)) / "lib" / package / executable
        if not target.is_file() or not os.access(target, os.X_OK):
            raise ValueError("Missing selected executable: " + str(target))

    report = {"profile": "manual_core", "namespace": NAMESPACE,
              "nodes": [list(item) for item in specs], "params_file": str(path),
              "hardware": hardware, "physical_actuation": hardware,
              "devices": ports if hardware else [], "lidar_enabled": lidar,
              "rc_provider": "rc_acquisition" if hardware else "rosbag2_player",
              "startup": {"autonomy_enabled": False, "actuation_enabled": False,
                          "automatic_rearm": False, "auto_enable_on_startup": auto_enable},
              "calibration_qualification": "inherited_suspended_bench_limits_not_full_vehicle_calibration",
              "parameter_origin": "config/vehicle/rc_bench.yaml (manual map and bounded driver limits)",
              "command_contract": {"final_writer": "vesc_backend", "manual_priority": "whole_pair",
                                   "neutral_resume": "automatic_if_inputs_valid"},
              "final_topics": ["/ebva/commands/motor/speed", "/ebva/commands/servo/position"],
              "telemetry": ["/ebva/sensors/core"] if hardware else [],
              "scan_topic": "/ebva/scan" if lidar else None}
    return report, params, lidar_params


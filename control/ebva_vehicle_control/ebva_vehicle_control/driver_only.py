"""Inspect a standalone VESC driver configuration without opening serial ports."""
import math
import os
from pathlib import Path
import stat

import yaml


def inspect_driver(params_file, *, get_prefix=None, check_device=False):
    data = yaml.safe_load(Path(params_file).read_text())
    try:
        params = dict(data["vesc_driver_node"]["ros__parameters"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Expected vesc_driver_node.ros__parameters") from exc
    port = params.get("port")
    if (not isinstance(port, str) or not port.startswith("/dev/")
            or ".." in Path(port).parts):
        raise ValueError("VESC port must be an explicit /dev path")
    for name in ("speed", "servo", "duty_cycle", "current", "brake", "position"):
        low, high = params.get(name + "_min"), params.get(name + "_max")
        if not all(type(value) in (int, float) and math.isfinite(value) for value in (low, high)):
            raise ValueError("Finite VESC bounds required: " + name)
        if low > high:
            raise ValueError("VESC minimum exceeds maximum: " + name)
        if name == "servo" and not 0 <= low <= high <= 1:
            raise ValueError("VESC servo bounds must be within [0, 1]")
    if check_device:
        path = Path(port)
        if not path.exists() or not stat.S_ISCHR(path.stat().st_mode):
            raise ValueError("VESC serial device unavailable: " + port)
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix = get_package_prefix
    executable = Path(get_prefix("vesc_driver")) / "lib/vesc_driver/vesc_driver_node"
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise ValueError("Missing VESC driver executable: " + str(executable))
    return params


def conflicting_nodes(namespace):
    """Reject the physical control chain before starting a standalone driver."""
    names = ("vesc_driver_node", "vesc_backend", "command_authority",
             "rc_acquisition", "keyboard_adapter", "key_teleop",
             "rc_reader_node", "ackermann_to_vesc_node", "ackermann_mux")
    prefix = namespace.rstrip("/")
    return sorted({ns + "/" + name for ns in ("", prefix) for name in names})

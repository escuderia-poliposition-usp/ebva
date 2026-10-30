"""Validate the owner's configuration without ROS or device imports."""
from dataclasses import fields
from pathlib import Path
import yaml
from .core import Settings


def inspect_configuration(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or set(data) != {"supervisor"}:
        raise ValueError("Expected the supervisor configuration")
    node = data["supervisor"]
    if not isinstance(node, dict) or set(node) != {"ros__parameters"}:
        raise ValueError("Expected supervisor ros__parameters")
    params = node["ros__parameters"]
    if not isinstance(params, dict):
        raise ValueError("Supervisor parameters must be a mapping")
    settings_names = {field.name for field in fields(Settings)}
    allowed = settings_names | {"input_topic", "output_topic", "use_sim_time"}
    if set(params) - allowed:
        raise ValueError("Unknown supervisor parameters")
    Settings(**{key: value for key, value in params.items() if key in settings_names})
    for key in ("input_topic", "output_topic"):
        if not isinstance(params.get(key), str) or not params[key]:
            raise ValueError("Explicit supervisor topics required")
    if type(params.get("use_sim_time", False)) is not bool:
        raise ValueError("use_sim_time must be boolean")
    return params

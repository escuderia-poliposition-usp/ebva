"""Standalone driver composition/configuration; no serial or driver execution."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from ebva_vehicle_control.driver_only import inspect_driver, conflicting_nodes


@pytest.fixture
def driver_config(tmp_path):
    source = Path(__file__).resolve().parents[1] / "config/vehicle/manual_core.yaml"
    data = yaml.safe_load(source.read_text())
    data["rc_acquisition"]["ros__parameters"]["serial_port"] = "/dev/rc_not_required_for_direct_commands"
    path = tmp_path / "vehicle.yaml"
    path.write_text(yaml.safe_dump(data))
    prefix = tmp_path / "vesc_driver"
    executable = prefix / "lib/vesc_driver/vesc_driver_node"
    executable.parent.mkdir(parents=True)
    executable.touch()
    executable.chmod(0o755)
    return path, prefix, data


def test_driver_only_extracts_current_driver_without_requiring_rc(driver_config):
    path, prefix, data = driver_config
    result = inspect_driver(path, get_prefix=lambda package: prefix)
    assert result == data["vesc_driver_node"]["ros__parameters"]
    assert "auto_enable_on_startup" not in result and "rc_timeout_s" not in result
    assert not Path(data["rc_acquisition"]["ros__parameters"]["serial_port"]).exists()


@pytest.mark.parametrize("change", [
    ("port", "/tmp/vesc"), ("port", "/dev/../tmp/vesc"), ("port", None),
    ("speed_max", float("inf")), ("speed_min", True),
    ("speed_max", -9000.), ("servo_min", -.1), ("servo_max", 1.1),
])
def test_bad_driver_configuration_is_rejected(driver_config, change):
    path, prefix, data = driver_config
    data["vesc_driver_node"]["ros__parameters"][change[0]] = change[1]
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        inspect_driver(path, get_prefix=lambda package: prefix)


def test_driver_only_checks_selected_device_and_executable(driver_config):
    path, prefix, data = driver_config
    data["vesc_driver_node"]["ros__parameters"]["port"] = "/dev/null"
    path.write_text(yaml.safe_dump(data))
    assert inspect_driver(path, get_prefix=lambda package: prefix, check_device=True)["port"] == "/dev/null"
    data["vesc_driver_node"]["ros__parameters"]["port"] = "/dev/ebva_direct_test_missing"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="unavailable"):
        inspect_driver(path, get_prefix=lambda package: prefix, check_device=True)
    (prefix / "lib/vesc_driver/vesc_driver_node").unlink()
    with pytest.raises(ValueError, match="executable"):
        inspect_driver(path, get_prefix=lambda package: prefix)


def test_standalone_launch_creates_only_driver_and_checks_competing_chain(monkeypatch, driver_config):
    import launch_ros.actions
    from launch.actions import RegisterEventHandler
    from ebva_vehicle_control import components, driver_only, manual, runtime_guards
    path, prefix, data = driver_config
    params = deepcopy(data["vesc_driver_node"]["ros__parameters"])
    calls, conflicts, inspection = [], [], []
    real_node = launch_ros.actions.Node
    def inspect(selected, **kwargs):
        inspection.append((selected, kwargs))
        return params
    def make_node(**kwargs):
        calls.append(kwargs)
        return real_node(**kwargs)
    def forbidden(*args, **kwargs):
        raise AssertionError("Standalone driver must not inspect/open RC or backend")
    monkeypatch.setattr(driver_only, "inspect_driver", inspect)
    monkeypatch.setattr(manual, "inspect_profile", forbidden)
    monkeypatch.setattr(runtime_guards, "running_conflicts", lambda names: conflicts.extend(names))
    monkeypatch.setattr(launch_ros.actions, "Node", make_node)
    actions = components.compose(SimpleNamespace(launch_configurations={
        "params_file": str(path), "ros_namespace": "/ebva"}), "vesc_driver")
    assert inspection == [(path, {"check_device": True})]
    assert len(calls) == 1 and calls[0]["package"] == "vesc_driver"
    assert calls[0]["executable"] == calls[0]["name"] == "vesc_driver_node"
    assert calls[0]["parameters"] == [params]
    assert not calls[0].get("remappings")
    assert len(actions) == 2 and isinstance(actions[0], RegisterEventHandler)
    assert "/ebva/vesc_backend" in conflicts and "/ebva/command_authority" in conflicts
    assert "/ebva/vesc_driver_node" in conflicts and "/vesc_driver_node" in conflicts


def test_default_root_namespace_still_rejects_control_chain():
    names = conflicting_nodes("")
    assert "/vesc_backend" in names and "/command_authority" in names
    assert len(names) == len(set(names))

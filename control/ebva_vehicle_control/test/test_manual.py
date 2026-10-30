"""Composition checks never instantiate acquisition or physical drivers."""
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from ebva_vehicle_control.manual import CORE, ACQUISITION, DRIVER, LIDAR, inspect_profile


def prepare(tmp_path, mutate=None):
    source = Path(__file__).resolve().parents[1] / "config"
    data = yaml.safe_load((source / "vehicle/manual_core.yaml").read_text())
    # Device aliases are deployment configuration, not fixed test expectations.
    data["rc_acquisition"]["ros__parameters"]["serial_port"] = "/dev/test_rc"
    data["vesc_driver_node"]["ros__parameters"]["port"] = "/dev/test_vesc"
    if mutate:
        name, key, value = mutate
        data[name]["ros__parameters"][key] = value
    root = tmp_path / "share"
    (root / "config/vehicle").mkdir(parents=True)
    (root / "config/sensors").mkdir()
    (root / "config/vehicle/manual_core.yaml").write_text(yaml.safe_dump(data))
    (root / "config/sensors/sensors.yaml").write_text((Path(__import__("ament_index_python.packages",fromlist=["get_package_share_directory"]).get_package_share_directory("ebva_sensors")) / "config/sensors/sensors.yaml").read_text())
    prefixes = {}
    for package, executable, _ in [*CORE, ACQUISITION, DRIVER, LIDAR]:
        prefix = tmp_path / package
        prefixes[package] = prefix
        target = prefix / "lib" / package / executable
        target.parent.mkdir(parents=True, exist_ok=True)
        target.touch(); target.chmod(0o755)
    return root, prefixes


def test_default_contains_only_authority_backend_and_starts_disabled(tmp_path):
    root, prefixes = prepare(tmp_path)
    called = []
    def selected(package):
        called.append(package)
        return prefixes[package]
    report, params, lidar = inspect_profile(root, selected)
    assert report["nodes"] == [list(item) for item in CORE]
    assert report["startup"] == {"autonomy_enabled": False, "actuation_enabled": False,
      "automatic_rearm": False, "auto_enable_on_startup": True}
    assert not report["physical_actuation"] and report["devices"] == [] and lidar is None
    assert called == ["ebva_vehicle_control", "ebva_vehicle_control"]
    assert params["command_authority"]["rc_publisher_name"] == "rosbag2_player"
    assert report["command_contract"]["final_writer"] == "vesc_backend"
    assert report["command_contract"]["manual_priority"] == "whole_pair"


def test_physical_profile_selects_only_own_rc_and_raw_vesc_driver(tmp_path):
    root, prefixes = prepare(tmp_path)
    report, params, _ = inspect_profile(root, prefixes.__getitem__, hardware=True)
    assert report["nodes"] == [list(item) for item in [*CORE, ACQUISITION, DRIVER]]
    assert report["devices"] == ["/dev/test_rc", "/dev/test_vesc"]
    assert params["command_authority"]["rc_publisher_name"] == "rc_acquisition"
    assert report["startup"]["actuation_enabled"] is False
    assert "bench" in report["calibration_qualification"]
    assert not any("ackermann_to_vesc" in item[1] or "autodrive" in item[1] for item in report["nodes"])


def test_optional_lidar_needs_only_its_selected_driver(tmp_path):
    root, prefixes = prepare(tmp_path)
    report, _, lidar = inspect_profile(root, prefixes.__getitem__, hardware=True, lidar=True)
    assert report["nodes"][-1] == list(LIDAR)
    assert report["scan_topic"] == "/ebva/scan" and lidar["laser_frame_id"] == "laser"
    with pytest.raises(ValueError, match="hardware"):
        inspect_profile(root, prefixes.__getitem__, lidar=True)


@pytest.mark.parametrize("change", [
    ("command_authority", "enforce_graph", False),
    ("vesc_backend", "enforce_graph", False),
    ("command_authority", "rc_publisher_name", "legacy_rc"),
    ("command_authority", "rc_timeout_s", .4),
    ("rc_acquisition", "rc_timeout_s", .5),
    ("vesc_backend", "backend_timeout_s", float("nan")),
    ("vesc_backend", "backend_timeout_s", True),
    ("vesc_backend", "auto_enable_on_startup", "true"),
    ("vesc_backend", "erpm_limit", float("inf")),
    ("vesc_driver_node", "speed_max", 900.),
    ("vesc_driver_node", "servo_min", .1),
    ("vesc_driver_node", "current_max", 10.),
    ("vesc_driver_node", "brake_min", -100.),
    ("rc_acquisition", "serial_port", "/tmp/serial"),
    ("vesc_driver_node", "port", "/dev/../tmp/serial"),
])
def test_invalid_command_and_device_compositions_are_rejected(tmp_path, change):
    root, prefixes = prepare(tmp_path, change)
    with pytest.raises(ValueError):
        inspect_profile(root, prefixes.__getitem__)


def test_device_paths_are_explicit_and_different(tmp_path):
    root, prefixes = prepare(tmp_path)
    with pytest.raises(ValueError, match="different"):
        inspect_profile(root, prefixes.__getitem__, rc_port="/dev/shared", vesc_port="/dev/shared")
    with pytest.raises(ValueError, match="/dev"):
        inspect_profile(root, prefixes.__getitem__, rc_port="relative-port")
    with pytest.raises(ValueError, match="unavailable"):
        inspect_profile(root, prefixes.__getitem__, hardware=True, check_devices=True,
                        rc_port="/dev/ebva_test_unavailable_rc", vesc_port="/dev/ebva_test_unavailable_vesc")


def test_only_selected_installed_executables_are_required(tmp_path):
    root, prefixes = prepare(tmp_path)
    (prefixes[DRIVER[0]] / "lib" / DRIVER[0] / DRIVER[1]).unlink()
    inspect_profile(root, prefixes.__getitem__)
    with pytest.raises(ValueError, match="executable"):
        inspect_profile(root, prefixes.__getitem__, hardware=True)


def test_runtime_launch_requests_complete_manual_before_instantiating_nodes(monkeypatch):
    from types import SimpleNamespace
    from ebva_vehicle_control import components, manual
    called={}
    def inspect(**kwargs):
        called.update(kwargs)
        raise ValueError("inspection sentinel")
    monkeypatch.setattr(manual,"inspect_profile",inspect)
    with pytest.raises(ValueError,match="inspection sentinel"):
        components.compose(SimpleNamespace(launch_configurations={"params_file":"/tmp/config.yaml"}),"manual")
    assert called["hardware"] is True and called["check_devices"] is True


def test_manual_config_can_require_explicit_actuation_enable(tmp_path):
    root, prefixes = prepare(tmp_path, ("vesc_backend", "auto_enable_on_startup", False))
    report, params, _ = inspect_profile(root, prefixes.__getitem__, hardware=True)
    assert report["startup"]["auto_enable_on_startup"] is False
    assert params["vesc_backend"]["auto_enable_on_startup"] is False

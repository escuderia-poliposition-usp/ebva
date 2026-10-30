"""Installed launch configuration checks; no device/process execution."""
from copy import deepcopy
from pathlib import Path
import yaml
import pytest
from ebva_vehicle_control.launching import inspect_bench
from ebva_vehicle_control.core import Mapping


def fixture_config():
    authority={"rc_timeout_s":.2,"autonomy_timeout_s":.2,"rc_center_us":1500.,
               "override_enter_us":100.,"override_release_us":80.,"enforce_graph":True}
    return {"rc_acquisition":{"ros__parameters":{"rc_timeout_s":.2}},
            "command_authority":{"ros__parameters":authority},
            "vesc_backend":{"ros__parameters":{**authority,"backend_timeout_s":.2,**Mapping().__dict__}},
            "vesc_driver_node":{"ros__parameters":{"speed_min":-600.,"speed_max":600.,
                                                  "servo_min":.4,"servo_max":.6}}}


def prepare(tmp_path, data):
    p=tmp_path/"params.yaml"
    p.write_text(yaml.safe_dump(data))
    for package, executables in [("ebva_vehicle_control",["rc_acquisition","command_authority","vesc_backend"]),
                                 ("vesc_driver",["vesc_driver_node"])]:
        for name in executables:
            target=tmp_path/package/"lib"/package/name
            target.parent.mkdir(parents=True,exist_ok=True)
            target.touch();target.chmod(0o755)
    return p, lambda name: tmp_path/name


def test_bench_preflight_uses_explicit_bounded_config_and_installed_nodes(tmp_path):
    path, prefix=prepare(tmp_path,fixture_config())
    report=inspect_bench(path,True,get_prefix=prefix)
    assert report["limits"]["erpm"]==600.
    assert report["command_contract"]["final_writer"]=="vesc_backend"
    assert report["physical_qualification"] is False
    assert report["command_contract"]["neutral_resume"]=="automatic_if_inputs_valid"


@pytest.mark.parametrize("change",["mismatch","no_graph","unbounded","infinite_timeout"])
def test_bench_preflight_rejects_inconsistent_or_unbounded_policy(tmp_path,change):
    data=fixture_config()
    if change=="mismatch":data["vesc_backend"]["ros__parameters"]["rc_timeout_s"]=.3
    if change=="no_graph":data["command_authority"]["ros__parameters"]["enforce_graph"]=False
    if change=="unbounded":data["vesc_backend"]["ros__parameters"]["erpm_limit"]=4000.
    if change=="infinite_timeout":data["vesc_backend"]["ros__parameters"]["backend_timeout_s"]=float("inf")
    path,prefix=prepare(tmp_path,data)
    with pytest.raises(ValueError):inspect_bench(path,True,get_prefix=prefix)


def test_bench_preflight_cannot_claim_a_missing_executable(tmp_path):
    path,prefix=prepare(tmp_path,fixture_config())
    (prefix("vesc_driver")/"lib/vesc_driver/vesc_driver_node").unlink()
    with pytest.raises(ValueError,match="Missing installed executable"):
        inspect_bench(path,True,get_prefix=prefix)

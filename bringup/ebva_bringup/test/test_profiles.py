"""YAML contract tested without component callbacks or devices."""
from pathlib import Path
from types import SimpleNamespace
import pytest,yaml
from ebva_bringup.profiles import ProfileError,inspect_profile,launch_system
def setup_resources(tmp_path):
    roots={}
    for package in ("ebva_bringup","demo"):
        p=tmp_path/package;roots[package]=p;(p/"profiles").mkdir(parents=True);(p/"launch").mkdir();(p/"config").mkdir()
    (roots["demo"]/"launch/opaque.launch.py").write_text("raise RuntimeError('must never import child')")
    (roots["demo"]/"config/node.yaml").write_text("/**:\n  ros__parameters:\n    gain: 2.0\n")
    exe=roots["demo"]/"lib/demo/node";exe.parent.mkdir(parents=True);exe.touch();exe.chmod(0o755)
    return roots
def read(tmp_path,data):
    roots=setup_resources(tmp_path);p=roots["ebva_bringup"]/"profiles/manual.yaml";p.write_text(yaml.safe_dump(data))
    return inspect_profile(get_share=lambda p:roots[p],get_prefix=lambda p:roots[p]),roots
def minimal():
    return {"version":1,"name":"manual","launches":[{"id":"demo","package":"demo","file":"launch/opaque.launch.py"}]}
def test_defaults_reported_without_import(tmp_path):
    r,_=read(tmp_path,minimal())
    assert r["launches"][0]["configuration"]=="default do pacote"
    assert r["scope"]=="static_resources_only"
def test_explicit_references(tmp_path):
    d=minimal();ref={"package":"demo","file":"config/node.yaml"};d["namespace"]="/robot"
    d["launches"][0]["arguments"]={"params_file":ref}
    d["nodes"]=[{"id":"direct","package":"demo","executable":"node","name":"other","parameters":[ref]}]
    r,_=read(tmp_path,d)
    assert r["nodes"][0]["parameters"][0].endswith("config/node.yaml")
    assert r["launches"][0]["arguments"]["params_file"].endswith("config/node.yaml")
@pytest.mark.parametrize("change",[
    lambda d:d.update(version=2),lambda d:d.update(version=True),lambda d:d.update(ros__parameters={}),
    lambda d:d.update(nodes="invalid"),lambda d:d["launches"][0].update(arguments={"hardware":True}),
    lambda d:d["launches"][0].update(file="../outside.launch.py"),
    lambda d:d["launches"][0].update(file="/outside.launch.py"),
    lambda d:d["launches"][0].update(file="missing.py"),
    lambda d:d["launches"].append(dict(d["launches"][0])),
    lambda d:d.update(namespace="/not valid"),lambda d:d.update(launches=[]),
    lambda d:d.update(nodes=[{"id":"x","package":"demo","executable":"../node"}]),
    lambda d:d.update(nodes=[{"id":"x","package":"demo","executable":"node","parameters":[{"gain":2}]}]),
    lambda d:d["launches"][0].update(arguments={"params_file":{"package":"ebva_bringup","file":"config/node.yaml"}})])
def test_invalid_profiles_fail_statically(tmp_path,change):
    d=minimal();change(d)
    with pytest.raises(ProfileError):read(tmp_path,d)
def test_missing_executable(tmp_path):
    d=minimal();d["nodes"]=[{"id":"x","package":"demo","executable":"missing"}]
    with pytest.raises(ProfileError,match="Executable"):read(tmp_path,d)
def test_duplicate_yaml_keys(tmp_path):
    roots=setup_resources(tmp_path);p=roots["ebva_bringup"]/"profiles/manual.yaml"
    p.write_text("version: 1\nversion: 1\nname: manual\n")
    with pytest.raises(ProfileError,match="Repeated"):inspect_profile(get_share=lambda p:roots[p])
def test_external_profile_path(tmp_path):
    roots=setup_resources(tmp_path);p=tmp_path/"external.yaml";p.write_text(yaml.safe_dump(minimal()))
    assert inspect_profile(str(p),get_share=lambda p:roots[p])["name"]=="manual"
def test_inspection_does_not_create_actions_lock_or_dds(monkeypatch):
    import ebva_bringup.profiles as module
    import ebva_bringup.launching as guards
    monkeypatch.setattr(module,"inspect_profile",lambda *a,**k:{"name":"manual"})
    def forbidden(*a,**k):raise AssertionError("Runtime during inspection")
    monkeypatch.setattr(module,"actions_for",forbidden);monkeypatch.setattr(guards,"claim_stack",forbidden);monkeypatch.setattr(guards,"running_conflicts",forbidden)
    assert len(launch_system(SimpleNamespace(launch_configurations={"check_only":"true"})))==1
def test_installed_catalogue_and_resource_ownership():
    from ament_index_python.packages import get_package_share_directory
    root=Path(get_package_share_directory("ebva_bringup"))
    expected = {
        "manual", "keyboard", "direct_commands", "sensors", "perception", "lanes",
        "signs", "slam_mapping_zed", "slam_mapping_imu", "slam_mapping_ekf",
        "localization_slam", "localization_amcl", "navigation_slam", "navigation_amcl",
        "supervisor", "supervisor_pi_straight", "supervisor_p_curve",
        "supervisor_p_intersection", "supervisor_wall_recovery", "supervisor_straight_rule",
    }
    assert {p.stem for p in (root/"profiles").glob("*.yaml")}==expected
    assert [p.name for p in (root/"launch").rglob("*.py")]==["bringup.launch.py"]
    assert not (root/"config").exists()
    for name in expected:
        assert inspect_profile(name)["name"]==name
        assert "ros__parameters" not in (root/"profiles"/(name+".yaml")).read_text()
def test_no_test_dependencies():
    import xml.etree.ElementTree as ET
    path=Path(__file__).resolve().parents[1]/"package.xml"
    assert not {"ebva_system_tests","ebva_simulation"} & {n.text for n in ET.parse(path).getroot()}

@pytest.mark.parametrize("name",["reference","reference_images"])
def test_retired_profiles_are_not_executable(name):
    with pytest.raises(ProfileError,match="Cannot load"):inspect_profile(name)

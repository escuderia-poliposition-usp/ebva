from ebva_system_tests.resources import fixture_bytes, fixture_exists
"""Preflight is portable; no controller import or process/device launch."""
import copy
from pathlib import Path
import pytest
import yaml
from ebva_system_tests.ftg_pd_software import inspect_profile, compose, SPECS, NAMESPACE

def prepare(tmp_path, change=None):
    source = Path(__file__).resolve().parents[1] / 'data/ftg_pd_software.yaml'
    data = yaml.safe_load(fixture_bytes(source.name).decode())
    if change:
        (name, key, value) = change
        data['/' + NAMESPACE + '/' + name]['ros__parameters'][key] = value
    target = tmp_path / 'share/data/ftg_pd_software.yaml'
    target.parent.mkdir(parents=True)
    target.write_text(yaml.safe_dump(data))
    prefixes = {}
    for (package, executable, _) in SPECS:
        prefix = tmp_path / package
        prefixes[package] = prefix
        path = prefix / 'lib' / package / executable
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        path.chmod(493)
    return (tmp_path / 'share', prefixes)

def test_profile_only_declares_software_nodes_and_namespaced_outputs(tmp_path):
    (root, prefixes) = prepare(tmp_path)
    report = inspect_profile(root, prefixes.__getitem__)
    assert not report['physical_actuation'] and report['devices'] == []
    assert report['calibration_qualification'] == 'software_fixture'
    assert len(report['nodes']) == 4
    assert not any((e in {'rc_acquisition', 'vesc_driver_node'} for (_, e, _) in report['nodes']))
    assert all((topic.startswith('/ftg_pd_software/') for topic in report['final_topics']))

@pytest.mark.parametrize('change', [('ftg_pd_adapter', 'mapping_purpose', 'measured_vehicle'), ('ftg_pd_adapter', 'expected_frame', 'base_link'), ('command_authority', 'rc_publisher_name', 'rc_acquisition'), ('vesc_backend', 'servo_max', 0.8), ('ftg_pd', 'source_timeout_s', float('inf'))])
def test_fixture_contract_mismatch_is_rejected_before_actions(tmp_path, change):
    (root, prefixes) = prepare(tmp_path, change)
    with pytest.raises(ValueError):
        inspect_profile(root, prefixes.__getitem__)

def test_missing_installed_controller_is_rejected(tmp_path):
    (root, prefixes) = prepare(tmp_path)
    (prefixes['ebva_vehicle_control'] / 'lib/ebva_vehicle_control/ftg_pd_adapter').unlink()
    with pytest.raises(ValueError, match='Missing installed'):
        inspect_profile(root, prefixes.__getitem__)

def test_hardware_flag_is_rejected_without_inspecting_or_creating_nodes():

    class Context:
        launch_configurations = {'hardware': 'true'}
    with pytest.raises(ValueError, match='no hardware mode'):
        compose(Context())

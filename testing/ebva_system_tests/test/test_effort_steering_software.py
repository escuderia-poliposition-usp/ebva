from ebva_system_tests.resources import fixture_bytes, fixture_exists
"""Preflight catches incompatible law, measurement and actuator contracts."""
from pathlib import Path
import shutil
import json
import pytest
import yaml
from ebva_system_tests.effort_steering_software import inspect_profile, specs, compose

@pytest.fixture(params=['waypoint_control', 'lidar_frenet_pp'])
def profile(request, tmp_path):
    method = request.param
    namespace = method + '_software'
    root = tmp_path / 'share'
    target = root / 'data'
    target.mkdir(parents=True)
    source = Path(__file__).resolve().parents[1] / 'data'
    for name in (namespace + '.yaml', namespace + '_state.json', 'waypoint_control_reference.csv'):
        if fixture_exists((source / name).name):
            (target / name).write_bytes(fixture_bytes((source / name).name))
    prefixes = {}
    for (package, executable, _) in specs(method):
        prefix = tmp_path / package
        prefixes[package] = prefix
        path = prefix / 'lib' / package / executable
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        path.chmod(493)
    return (method, root, prefixes, target)

def test_only_software_nodes_and_explicit_measurement_semantics(profile):
    (method, root, prefixes, _) = profile
    report = inspect_profile(method, root, prefixes.__getitem__)
    assert not report['physical_actuation'] and report['devices'] == []
    assert report['longitudinal_units'] == 'dimensionless_effort' and report['steering_units'] == 'rad'
    assert report['command_header_frame'] == ('fixture_map' if method == 'waypoint_control' else 'fixture_lidar_flu')
    assert not any((s[1] in ('rc_acquisition', 'vesc_driver') for s in report['nodes']))
    if method == 'lidar_frenet_pp':
        assert report['state_contract'] is None
    else:
        assert report['state_contract']['declaration_valid'] and (not report['state_contract']['require_tf'])

@pytest.mark.parametrize('mutation', ['physical_map', 'contract', 'frame', 'nan_age', 'future', 'backend_bounds', 'rc_owner'])
def test_composition_mismatch_rejected(profile, mutation):
    (method, root, prefixes, target) = profile
    ns = method + '_software'
    path = target / (ns + '.yaml')
    data = yaml.safe_load(path.read_text())

    def params(name):
        return data['/' + ns + '/' + name]['ros__parameters']
    adapter = params('effort_steering_adapter')
    if mutation == 'physical_map':
        adapter['mapping_purpose'] = 'measured_vehicle'
    elif mutation == 'contract':
        adapter['expected_contract'] = 'frm_point_sample_v1'
    elif mutation == 'frame':
        adapter['expected_frame'] = 'camera'
    elif mutation == 'nan_age':
        params(method)['observation_timeout'] = float('nan')
    elif mutation == 'future':
        params(method)['future_slack'] = 0.1
    elif mutation == 'backend_bounds':
        params('vesc_backend')['servo_max'] = 0.8
    else:
        params('command_authority')['rc_publisher_name'] = 'rc_acquisition'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        inspect_profile(method, root, prefixes.__getitem__)

def test_hardware_mode_refused_before_node_construction(profile):
    (method, *_) = profile

    class Context:
        launch_configurations = {'hardware': 'true'}
    with pytest.raises(ValueError):
        compose(Context(), method)

def test_reference_and_state_contract_are_checked(profile):
    (method, root, prefixes, target) = profile
    if method != 'waypoint_control':
        return
    path = target / 'waypoint_control_software_state.json'
    data = json.loads(path.read_text())
    data['state_ports'][0]['model_point'] = 'camera'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        inspect_profile(method, root, prefixes.__getitem__)

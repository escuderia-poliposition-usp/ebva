from ebva_system_tests.resources import fixture_bytes, fixture_exists
"""New profiles reject incompatible state composition before launching any nodes."""
from pathlib import Path
import importlib
import json
import pytest
import yaml

def prepare_profile(name, tmp_path):
    module = importlib.import_module('ebva_system_tests.' + name)
    source = Path(__file__).resolve().parents[1] / 'data'
    target = tmp_path / 'share/data'
    target.mkdir(parents=True)
    for name in (name + '.yaml', name + '_state.json', 'waypoint_pp_reference.csv'):
        (target / name).write_bytes(fixture_bytes((source / name).name))
    prefixes = {}
    for (package, executable, _) in module.SPECS:
        prefix = tmp_path / package
        prefixes[package] = prefix
        path = prefix / 'lib' / package / executable
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        path.chmod(493)
    return (module, tmp_path / 'share', prefixes, target)

@pytest.fixture(params=['cbf_yaw_rate_software', 'waypoint_pp_software'])
def profile(request, tmp_path):
    return prepare_profile(request.param, tmp_path)

def test_explicit_state_declares_only_actual_software_nodes(profile):
    (module, root, prefixes, _) = profile
    report = module.inspect_profile(root, prefixes.__getitem__)
    assert not report['physical_actuation'] and report['devices'] == []
    assert report['state_contract']['declaration_valid']
    assert report['state_contract']['tf_validation_scope'] == 'not_requested'
    assert all((topic.startswith('/' + module.NAMESPACE + '/') for topic in report['final_topics']))
    assert not any((row[1] in ('rc_acquisition', 'vesc_driver_node') for row in report['nodes']))

@pytest.mark.parametrize('mutation', ['body', 'consumer_point', 'publisher', 'clock_policy'])
def test_incompatible_state_declaration_is_rejected(profile, mutation):
    (module, root, prefixes, target) = profile
    path = target / (module.NAMESPACE + '_state.json')
    data = json.loads(path.read_text())
    if mutation == 'body':
        data['state_ports'][0]['body'] = 'camera'
    elif mutation == 'consumer_point':
        data['consumers'][0]['model_point'] = 'different_point'
    elif mutation == 'publisher':
        data['state_ports'][0]['publisher'] = '/unexpected'
    else:
        data['state_ports'][0]['stamp'] = 'reception'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        module.inspect_profile(root, prefixes.__getitem__)

@pytest.mark.parametrize('mutation', ['physical_map', 'rc_owner', 'infinite_timeout', 'bounds'])
def test_actuation_contract_is_rejected_before_node_construction(profile, mutation):
    (module, root, prefixes, target) = profile
    path = target / (module.NAMESPACE + '.yaml')
    data = yaml.safe_load(path.read_text())
    adapter = 'cbf_yaw_rate_adapter' if module.NAMESPACE.startswith('cbf') else 'ackermann_adapter'

    def params(name):
        return data['/' + module.NAMESPACE + '/' + name]['ros__parameters']
    if mutation == 'physical_map':
        params(adapter)['mapping_purpose'] = 'measured_vehicle'
    elif mutation == 'rc_owner':
        params('command_authority')['rc_publisher_name'] = 'rc_acquisition'
    elif mutation == 'infinite_timeout':
        params(adapter)['source_timeout_s'] = float('inf')
    else:
        params('vesc_backend')['servo_max'] = 0.8
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        module.inspect_profile(root, prefixes.__getitem__)

def test_hardware_mode_is_rejected_before_inspecting_or_importing_ros_actions(profile):
    (module, _, _, _) = profile

    class Context:
        launch_configurations = {'hardware': 'true'}
    with pytest.raises(ValueError, match='no hardware mode'):
        module.compose(Context())

@pytest.mark.parametrize('key,value', [('speed_floor_mps', 0.2), ('max_steering_rad', 0.4)])
def test_cbf_projection_must_match_preserved_model_limits(tmp_path, key, value):
    from ebva_system_tests import cbf_yaw_rate_software as module
    source = Path(__file__).resolve().parents[1] / 'data'
    target = tmp_path / 'data'
    target.mkdir(parents=True)
    for name in (module.NAMESPACE + '.yaml', module.NAMESPACE + '_state.json'):
        (target / name).write_bytes(fixture_bytes((source / name).name))
    path = target / (module.NAMESPACE + '.yaml')
    data = yaml.safe_load(path.read_text())
    data['/' + module.NAMESPACE + '/cbf_yaw_rate_adapter']['ros__parameters'][key] = value
    path.write_text(yaml.safe_dump(data))
    for (package, executable, _) in module.SPECS:
        executable_path = tmp_path / 'lib' / package / executable
        executable_path.parent.mkdir(parents=True, exist_ok=True)
        executable_path.touch()
        executable_path.chmod(493)
    with pytest.raises(ValueError, match='selected yaw-rate model'):
        module.inspect_profile(tmp_path, lambda _: tmp_path)

@pytest.mark.parametrize('mutation', ['nan_age', 'nan_future', 'future_disagrees', 'declaration_age'])
def test_waypoint_projection_policy_matches_runtime_and_declaration(tmp_path, mutation):
    (module, root, prefixes, target) = prepare_profile('waypoint_pp_software', tmp_path)
    path = target / (module.NAMESPACE + '.yaml')
    data = yaml.safe_load(path.read_text())
    pose = data['/' + module.NAMESPACE + '/odometry_pose']['ros__parameters']
    if mutation == 'nan_age':
        pose['max_age_sec'] = float('nan')
    elif mutation == 'nan_future':
        pose['future_tolerance_sec'] = float('nan')
    elif mutation == 'future_disagrees':
        pose['future_tolerance_sec'] += 0.01
    else:
        state_path = target / (module.NAMESPACE + '_state.json')
        state = json.loads(state_path.read_text())
        state['state_ports'][1]['max_age_sec'] = 0.1
        state_path.write_text(json.dumps(state))
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        module.inspect_profile(root, prefixes.__getitem__)

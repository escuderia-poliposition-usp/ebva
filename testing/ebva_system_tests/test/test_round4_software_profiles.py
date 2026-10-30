from ebva_system_tests.resources import fixture_bytes, fixture_exists
"""Reject incompatible model/time/state configurations before any fixture launch."""
from pathlib import Path
import shutil
import json
import pytest
import yaml
from ebva_system_tests import alternate_cbf_software, stanley_frenet_software

@pytest.fixture(params=['stanley_frenet', 'yaw_rate_margin', 'steering_tangent'])
def profile(request, tmp_path):
    method = request.param
    cbf = method != 'stanley_frenet'
    ns = ('cbf_' if cbf else '') + method + '_software'
    inspect = (lambda root, prefix: alternate_cbf_software.inspect_profile(method, root, prefix)) if cbf else stanley_frenet_software.inspect_profile
    specs = alternate_cbf_software.specs(method) if cbf else stanley_frenet_software.SPECS
    root = tmp_path / 'share'
    target = root / 'data'
    target.mkdir(parents=True)
    source = Path(__file__).resolve().parents[1] / 'data'
    for suffix in ('.yaml', '_state.json'):
        (target / (ns + suffix)).write_bytes(fixture_bytes((source / (ns + suffix)).name))
    prefixes = {}
    for (package, executable, _) in specs:
        prefix = tmp_path / package
        prefixes[package] = prefix
        path = prefix / 'lib' / package / executable
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        path.chmod(493)
    return (method, ns, inspect, root, prefixes, target)

def test_distinct_models_and_explicit_fixture_only(profile):
    (method, ns, inspect, root, prefixes, _) = profile
    report = inspect(root, prefixes.__getitem__)
    assert not report['physical_actuation'] and report['devices'] == []
    assert report['state_contract']['declaration_valid'] and (not report['state_contract']['require_tf'])
    assert all((topic.startswith('/' + ns + '/') for topic in report['final_topics']))
    if method == 'stanley_frenet':
        assert report['signed_model_speed_mps'] == -0.6
    else:
        assert report['cbf_model'] == method

@pytest.mark.parametrize('mutation', ['frame', 'model', 'nan_age', 'future', 'skew', 'physical', 'bounds', 'rc_owner'])
def test_mismatched_composition_refused(profile, mutation):
    (method, ns, inspect, root, prefixes, target) = profile
    path = target / (ns + '.yaml')
    data = yaml.safe_load(path.read_text())
    cbf = method != 'stanley_frenet'
    node = 'cbf_' + method if cbf else method
    adapter = node + '_adapter' if cbf else 'ackermann_adapter'

    def params(name):
        return data['/' + ns + '/' + name]['ros__parameters']
    if mutation == 'frame':
        params(adapter)['expected_frame'] = 'camera'
    elif mutation == 'model':
        params(node)['model_reference_point' if cbf else 'reference_contract'] = 'other_model'
    elif mutation == 'nan_age':
        params(node)['timeout_s' if cbf else 'observation_timeout'] = float('nan')
    elif mutation == 'future':
        params(node)['future_tolerance_s' if cbf else 'future_slack'] = 0.1
    elif mutation == 'skew':
        params(node)['max_skew_s' if cbf else 'maximum_skew'] = 0.4
    elif mutation == 'physical':
        params(adapter)['mapping_purpose'] = 'measured_vehicle'
    elif mutation == 'bounds':
        params('vesc_backend')['servo_max'] = 0.9
    else:
        params('command_authority')['rc_publisher_name'] = 'rc_acquisition'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        inspect(root, prefixes.__getitem__)

def test_declared_state_provenance_cannot_be_relabelled(profile):
    (method, ns, inspect, root, prefixes, target) = profile
    path = target / (ns + '_state.json')
    state = json.loads(path.read_text())
    state['state_ports'][0]['publisher'] = '/unidentified_source'
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError):
        inspect(root, prefixes.__getitem__)

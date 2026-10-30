from ebva_system_tests.resources import fixture_bytes, fixture_exists
"""Check the CSV/Frenet fixture boundary before any software processes start."""
from pathlib import Path
import pytest
import yaml
from ebva_system_tests.csv_frenet_software import inspect_profile, compose, NAMESPACE, SPECS

def prepare(tmp_path, change=None):
    source = Path(__file__).resolve().parents[1] / 'data'
    data = yaml.safe_load(fixture_bytes((source / 'csv_frenet_software.yaml').name).decode())
    if change:
        (name, key, value) = change
        data['/' + NAMESPACE + '/' + name]['ros__parameters'][key] = value
    target = tmp_path / 'share/data'
    target.mkdir(parents=True)
    (target / 'csv_frenet_software.yaml').write_text(yaml.safe_dump(data))
    (target / 'csv_frenet_reference.csv').write_text(fixture_bytes((source / 'csv_frenet_reference.csv').name).decode())
    prefixes = {}
    for (package, executable, _) in SPECS:
        prefix = tmp_path / package
        prefixes[package] = prefix
        path = prefix / 'lib' / package / executable
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
        path.chmod(493)
    return (tmp_path / 'share', prefixes)

def test_only_software_nodes_with_explicit_reference_and_existing_rc_authority(tmp_path):
    (root, prefixes) = prepare(tmp_path)
    report = inspect_profile(root, prefixes.__getitem__)
    assert report['devices'] == [] and (not report['physical_actuation'])
    assert report['reference_contract']['model_point'] == 'software_fixture_rear_axle'
    assert [row[1] for row in report['nodes']] == ['csv_frenet_pp', 'ackermann_adapter', 'command_authority', 'vesc_backend']
    assert all((topic.startswith('/csv_frenet_software/') for topic in report['final_topics']))

@pytest.mark.parametrize('change', [('csv_frenet_pp', 'model_point', 'base_link'), ('csv_frenet_pp', 'reference_frame_id', 'map'), ('csv_frenet_pp', 'reference_contract', 'legacy_throttle'), ('csv_frenet_pp', 'observation_timeout', float('inf')), ('csv_frenet_pp', 'future_slack', 0.3), ('ackermann_adapter', 'mapping_purpose', 'measured_vehicle'), ('command_authority', 'rc_publisher_name', 'rc_acquisition'), ('vesc_backend', 'servo_max', 0.8)])
def test_incompatible_composition_is_rejected(tmp_path, change):
    (root, prefixes) = prepare(tmp_path, change)
    with pytest.raises(ValueError):
        inspect_profile(root, prefixes.__getitem__)

def test_reference_is_required_as_installed_resource(tmp_path):
    (root, prefixes) = prepare(tmp_path)
    (root / 'data/csv_frenet_reference.csv').unlink()
    with pytest.raises(ValueError, match='reference'):
        inspect_profile(root, prefixes.__getitem__)

def test_hardware_flag_never_starts_a_profile():

    class Context:
        launch_configurations = {'hardware': 'true'}
    with pytest.raises(ValueError, match='no hardware mode'):
        compose(Context())

"""Validation and runtime ownership used by the native profiles."""
import os
from pathlib import Path
import subprocess
import sys
import pytest
from ebva_bringup.validation import ConfigError, boolean
from ebva_bringup.launching import reject_running

def test_ros_manifest_is_valid_and_declares_ament_python():
    from catkin_pkg.package import parse_package
    import xml.etree.ElementTree as ET
    path = Path(__file__).resolve().parents[1] / 'package.xml'
    package = parse_package(path)
    package.validate()
    assert ET.parse(path).findtext('export/build_type') == 'ament_python'

def test_active_node_collision_is_rejected():
    reject_running(['/driver'], ['/unrelated'])
    with pytest.raises(ConfigError, match='ja estao ativos.*driver'):
        reject_running(['/driver'], ['/driver', '/unrelated'])


def test_invalid_boolean_is_rejected():
    assert boolean('true') is True
    assert boolean('false') is False
    with pytest.raises(ConfigError, match='Booleano invalido'):
        boolean('maybe')


def test_single_instance_lock_is_released_with_process_exit(tmp_path):
    from ebva_bringup import launching
    stream = launching.claim_stack(tmp_path)
    script = 'from ebva_bringup.launching import claim_stack; claim_stack(' + repr(str(tmp_path)) + ')'
    environment = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    result = subprocess.run([sys.executable, '-c', script], env=environment, capture_output=True, text=True)
    assert result.returncode != 0 and 'Outra instancia' in result.stderr
    stream.close()
    result = subprocess.run([sys.executable, '-c', script], env=environment, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr

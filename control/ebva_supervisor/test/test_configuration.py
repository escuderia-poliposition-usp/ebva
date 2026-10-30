from pathlib import Path
import yaml
import pytest
from ebva_supervisor.configuration import inspect_configuration
from ebva_supervisor.core import METHODS


def test_every_published_method_has_owner_configuration():
    root = Path(__file__).resolve().parents[1]
    for method in METHODS:
        params = inspect_configuration(root / "config" / (method + ".yaml"))
        assert params["method"] == method
        assert params["source_timeout_s"] == .2


def test_configuration_rejects_unknown_fields_and_missing_topics(tmp_path):
    base = {"supervisor": {"ros__parameters": {"method": "supervised",
        "input_topic": "classification/input", "output_topic": "control/autonomy/effort"}}}
    path = tmp_path / "params.yaml"
    base["supervisor"]["ros__parameters"]["typo"] = 1
    path.write_text(yaml.safe_dump(base))
    with pytest.raises(ValueError, match="Unknown"):
        inspect_configuration(path)
    base["supervisor"]["ros__parameters"].pop("typo")
    base["supervisor"]["ros__parameters"].pop("input_topic")
    path.write_text(yaml.safe_dump(base))
    with pytest.raises(ValueError, match="topics"):
        inspect_configuration(path)

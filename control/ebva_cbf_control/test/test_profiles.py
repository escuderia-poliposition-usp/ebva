import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from ebva_cbf_control.profiles import selected_nodes, MODELS


@pytest.mark.parametrize("model", MODELS)
def test_filter_only_selection_preserves_model(model):
    nodes = selected_nodes(model)
    assert len(nodes) == 1
    assert nodes[0].node_package == "ebva_cbf_control"
    assert nodes[0].node_executable == model + "_filter"


def test_unknown_model_rejected():
    with pytest.raises(ValueError):
        selected_nodes("mixed")


@pytest.mark.parametrize("argument", ["rc_input", "vehicle_output"])
def test_retired_launch_flags_cannot_restore_physical_routes(argument):
    path = Path(__file__).resolve().parents[1] / "launch/select.launch.py"
    spec = importlib.util.spec_from_file_location("cbf_filter_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with pytest.raises(ValueError, match="retired"):
        module.compose(SimpleNamespace(launch_configurations={"model": "yaw_rate", argument: "true"}))

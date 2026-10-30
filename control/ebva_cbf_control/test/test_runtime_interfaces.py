"""Only active CBF interfaces may remain in the source/runtime tree."""
import ast
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_only_active_filter_and_stamped_entrypoints_are_packaged():
    tree = ast.parse((ROOT / "setup.py").read_text())
    entries = [node.value.split(" = ", 1)[0] for node in ast.walk(tree)
               if isinstance(node, ast.Constant) and isinstance(node.value, str)
               and " = ebva_cbf_control." in node.value]
    assert set(entries) == {model + "_" + role for model in
                           ("yaw_rate", "yaw_rate_margin", "steering_tangent")
                           for role in ("filter", "stamped")}
    assert not (ROOT / "docs/retired").exists()
    assert not list(ROOT.rglob("rc2uno.py"))
    assert not list(ROOT.rglob("cbf2vehicle.py"))


def test_removed_transport_modules_cannot_be_imported():
    for model in ("yaw_rate", "yaw_rate_margin", "steering_tangent"):
        for module in ("rc2uno", "cbf2vehicle"):
            assert importlib.util.find_spec("ebva_cbf_control." + model + "." + module) is None

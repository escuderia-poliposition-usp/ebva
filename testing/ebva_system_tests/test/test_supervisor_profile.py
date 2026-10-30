"""The supervisor campaign selects software-only installed resources."""
from pathlib import Path
import yaml


def test_supervisor_campaign_declares_only_software_nodes():
    root = Path(__file__).resolve().parents[1]
    data = yaml.safe_load((root / "profiles/supervisor_software.yaml").read_text())
    assert data["namespace"] == "/supervisor_software" and not data.get("launches")
    assert {row["executable"] for row in data["nodes"]} == {
        "supervisor_node", "supervisor_effort_adapter", "command_authority", "vesc_backend"}
    assert all(row["parameters"] for row in data["nodes"])

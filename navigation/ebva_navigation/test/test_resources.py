"""Package-only defaults, without planner lifecycle activation."""
from pathlib import Path
import yaml
def test_navigation_config_ownership():
    from ament_index_python.packages import get_package_share_directory
    root=Path(get_package_share_directory("ebva_navigation"))
    data=yaml.safe_load((root/"config/nav2_params.yaml").read_text())
    assert not {"amcl","map_server","vesc_driver_node"} & set(data)
    assert "SmacPlannerHybrid" in data["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
    bridge=yaml.safe_load((root/"config/bridge.yaml").read_text())["cmd_vel_to_ackermann_node"]["ros__parameters"]
    assert bridge["ackermann_cmd_topic"]=="control/autonomy/ackermann"

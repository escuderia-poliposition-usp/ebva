from pathlib import Path
import pytest
from ebva_bringup.profiles import inspect_profile as inspect_orchestrator
from ebva_system_tests import lane_control_software,frm_control_software,ftg_pd_software,csv_frenet_software,waypoint_pp_software,cbf_yaw_rate_software,alternate_cbf_software,effort_steering_software,stanley_frenet_software,manual_fixture
CASES=[
 lane_control_software.inspect_profile,frm_control_software.inspect_profile,
 ftg_pd_software.inspect_profile,csv_frenet_software.inspect_profile,
 waypoint_pp_software.inspect_profile,cbf_yaw_rate_software.inspect_profile,
 lambda:alternate_cbf_software.inspect_profile("yaw_rate_margin"),
 lambda:alternate_cbf_software.inspect_profile("steering_tangent"),
 lambda:effort_steering_software.inspect_profile("waypoint_control"),
 lambda:effort_steering_software.inspect_profile("lidar_frenet_pp"),
 stanley_frenet_software.inspect_profile,lambda:manual_fixture.inspect_profile()[0]]
@pytest.mark.parametrize("inspect",CASES)
def test_report_matches_executed_yaml(inspect):
    report=inspect()
    assert Path(report["profile_file"]).is_file()
    actual=inspect_orchestrator(report["profile_file"])
    assert not actual["launches"] and report["physical_actuation"] is False
    assert {tuple(row) for row in report["nodes"]}=={(n["package"],n["executable"],n.get("name",n["executable"])) for n in actual["nodes"]}
    assert not {"rc_acquisition","vesc_driver_node","urg_node2_node"} & {n["executable"] for n in actual["nodes"]}

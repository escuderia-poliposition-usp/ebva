"""Declared modes/TF responsibilities, without starting localization stacks."""
from pathlib import Path
import yaml
def test_preserved_priors_and_modes():
    from ament_index_python.packages import get_package_share_directory
    root=Path(get_package_share_directory("ebva_localization"));frames=[]
    for name in ("slam_toolbox_phase2a.yaml","slam_toolbox_phase2b.yaml","slam_toolbox_phase3.yaml"):
        params=yaml.safe_load((root/"config/slam"/name).read_text())["slam_toolbox"]["ros__parameters"]
        assert params["mode"]=="mapping";frames.append(params["base_frame"])
    assert frames==["zed_camera_link","base_link","zed_camera_link"]
    data=yaml.safe_load((root/"config/slam/slam_toolbox_localization.yaml").read_text())
    assert data["slam_toolbox"]["ros__parameters"]["mode"]=="localization"
    assert (root/"config/legacy_imu_prior.yaml").is_file()
    assert (root/"config/amcl.yaml").is_file()

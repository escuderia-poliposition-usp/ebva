from pathlib import Path
import yaml
def test_sensor_defaults_and_tf_ownership():
    from ament_index_python.packages import get_package_share_directory
    root=Path(get_package_share_directory("ebva_sensors"))
    for name in ("lidar","lidar_mapping_zed","lidar_mapping_imu"):
        data=yaml.safe_load((root/"config"/(name+".yaml")).read_text())
        assert "urg_node" in data and data["static_transform"]["child"]=="laser"
    for name,publish in (("zed_images",False),("zed_odometry",True),("zed_measurements",False)):
        data=yaml.safe_load((root/"config"/(name+".yaml")).read_text())
        assert data["launch_arguments"]["publish_map_tf"] is False
        assert data["launch_arguments"]["publish_tf"] is publish
        assert (root/data["ros_parameters_file"]["file"]).is_file()

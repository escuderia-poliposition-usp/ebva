import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from ebva_calibration.intrinsics import CameraCalibration
from ebva_calibration.synchronizer import Synchronizer


def test_configured_nodes_and_synchronized_metadata(tmp_path):
    # Software-only DDS in the isolated test container; no driver/device/actuator.
    config=tmp_path/'config.yaml'
    config.write_text("camera:\n  camera_topic: /fixture/image\n  image_size: {width: 640, length: 480}\n  chessboard: {rows: 6, columns: 9, square_size_m: .03}\nlidar: {lidar_topic: /fixture/scan}\ngeneral:\n  slop: 0.05\n  max_file_saved: 2\n  data_folder: pairs\n  config_folder: intrinsics\n  camera_intrinsic_calibration: camera.yaml\n")
    rclpy.init(args=['--ros-args','-p','configuration_file:='+str(config)])
    nodes=[]
    try:
        camera=CameraCalibration();nodes.append(camera)
        assert camera.objp.shape==(54,3)
        assert camera.image_sub.callbacks  # Old implementation never registered image_callback.
        camera.save_calibration()
        assert not (tmp_path/'intrinsics/camera.yaml').exists()
        sync=Synchronizer();nodes.append(sync)
        image=CvBridge().cv2_to_imgmsg(np.zeros((4,6,3),np.uint8),encoding='bgr8')
        image.header.frame_id='camera';image.header.stamp.sec=12
        scan=LaserScan();scan.header.frame_id='laser';scan.header.stamp.sec=12
        scan.header.stamp.nanosec=15;scan.angle_min=-1.;scan.angle_increment=.5
        scan.ranges=[1.,2.,3.];scan.range_min=.1;scan.range_max=10.
        sync.synchronize_data(scan,image)
        assert sync.counter==1
        from pathlib import Path
        archives=list(Path(sync.storage_path).glob('*.npz'))
        assert len(archives)==1 and len(list(Path(sync.storage_path).glob('*.png')))==1
        with np.load(archives[0],allow_pickle=False) as saved:
            np.testing.assert_allclose(saved['ranges'],[1,2,3])
            np.testing.assert_allclose(saved['angles'],[-1,-.5,0])
            assert saved['scan_stamp_nanosec']==15 and saved['image_stamp_sec']==12
            assert saved['scan_frame']=='laser'
    finally:
        for node in nodes:node.destroy_node()
        rclpy.shutdown()

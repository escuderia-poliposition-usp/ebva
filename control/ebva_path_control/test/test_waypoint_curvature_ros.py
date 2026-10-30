"""Contrato ROS de tempo/frame da variante por curvatura, sem atuação."""
import copy
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

from ebva_path_control.curvature_node import CurvatureController


def test_curvature_output_preserves_stamp_and_rejects_invalid_sources(tmp_path):
    csv = tmp_path / "points.csv"
    csv.write_text("0,0\n1,0\n2,0.3\n3,0.8\n4,1.8\n")
    config = tmp_path / "reference.yaml"
    config.write_text("wpt_path: points.csv\nwpt_xind: 0\nwpt_yind: 1\n")
    rclpy.init(args=["--ros-args", "-p", "reference_file:=" + str(config),
                    "-p", "output_topic:=test/curvature_command"])
    controller = CurvatureController()
    observer = Node("curvature_contract_observer", use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(observer)
    outputs = []
    subscriber = observer.create_subscription(
        AckermannDriveStamped, "test/curvature_command", outputs.append, 10)
    def pump(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
    def message():
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = controller.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = 1.2, .4
        msg.pose.pose.orientation.z = math.sin(.3/2)
        msg.pose.pose.orientation.w = math.cos(.3/2)
        return msg
    try:
        deadline = time.monotonic() + 3
        while not controller.publisher.get_subscription_count() and time.monotonic() < deadline:
            pump(.05)
        assert controller.publisher.get_subscription_count()
        original = message()
        controller.receive(original)
        pump(.1)
        assert len(outputs) == 1
        assert outputs[0].header == original.header
        assert math.isfinite(outputs[0].drive.speed)
        assert math.isfinite(outputs[0].drive.steering_angle)
        controller.receive(original)  # duplicate stamp
        stale = message()
        stale.header.stamp.sec -= 5
        controller.receive(stale)
        future = message()
        future.header.stamp.sec += 5
        controller.receive(future)
        wrong_frame = message()
        wrong_frame.header.frame_id = "unknown"
        controller.receive(wrong_frame)
        invalid = message()
        invalid.pose.pose.position.x = float("nan")
        controller.receive(invalid)
        pump(.3)  # silence never renews the source timestamp
        assert len(outputs) == 1
        recovered = message()
        controller.receive(recovered)
        pump(.1)
        assert len(outputs) == 2
        assert outputs[-1].header == recovered.header
    finally:
        executor.remove_node(controller)
        executor.remove_node(observer)
        executor.shutdown()
        controller.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()

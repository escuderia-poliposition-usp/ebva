"""Projeção real do nó C++ com dados sintéticos, sem sensores ou mapa em disco."""
import math
import os
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


def test_lidar_only_projection_preserves_geometry_and_frame():
    rclpy.init()
    node = Node("pointcloud_projection_test")
    messages = []
    subscriber = node.create_subscription(PointCloud2, "/fused_cloud", messages.append, 10)
    publisher = node.create_publisher(LaserScan, "/scan", 10)
    process = subprocess.Popen([
        "ros2", "run", "ebva_pointcloud_fusion", "fusion_node", "--ros-args",
        "-p", "use_cloud:=false", "-p", "accumulate:=false",
        "-p", "target_frame:=projection_lidar"], stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        deadline = time.monotonic() + 15
        while not messages and time.monotonic() < deadline:
            assert process.poll() is None, process.stderr.read()
            scan = LaserScan()
            scan.header.frame_id = "projection_lidar"
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.angle_min = 0.0
            scan.angle_increment = math.pi / 2
            scan.angle_max = math.pi / 2
            scan.range_min, scan.range_max = 0.05, 10.0
            scan.ranges = [1.0, 2.0]
            publisher.publish(scan)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert messages, "O nó não publicou a projeção sintética"
        cloud = messages[-1]
        assert cloud.header.frame_id == "projection_lidar"
        assert cloud.width * cloud.height == 2
        offsets = {field.name: field.offset for field in cloud.fields}
        endian = ">" if cloud.is_bigendian else "<"
        points = [tuple(struct.unpack_from(endian + "f", cloud.data,
                        index * cloud.point_step + offsets[key])[0]
                        for key in ("x", "y", "z")) for index in range(2)]
        assert math.isclose(points[0][0], 1.0, abs_tol=1e-6)
        assert math.isclose(points[0][1], 0.0, abs_tol=1e-6)
        assert math.isclose(points[1][0], 0.0, abs_tol=1e-5)
        assert math.isclose(points[1][1], 2.0, abs_tol=1e-6)
        assert all(math.isclose(p[2], 0.0, abs_tol=1e-6) for p in points)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            _, errors = process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            _, errors = process.communicate(timeout=3)
            raise AssertionError("O nó não encerrou com SIGINT")
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0, errors

#!/usr/bin/env python3
"""Relative LiDAR centering from fresh scans; no odometry or heartbeat."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray
from ebva_control_msgs.msg import LongitudinalEffortSteering
from ..effort_contract import LidarEffortContract
from ..effort_ros import original_stamp, publish_result
from .core import LidarFrenetCore


class FrenetPurePursuit(Node):
    extract_left_right = LidarFrenetCore.extract_left_right

    def __init__(self, **kwargs):
        super().__init__('lidar_frenet_pp', **kwargs)
        self.contract = LidarEffortContract(
            scan_frame_id=self.declare_parameter('scan_frame_id', '').value,
            scan_contract=self.declare_parameter('scan_contract', '').value,
            observation_timeout=self.declare_parameter('observation_timeout', .2).value,
            future_slack=self.declare_parameter('future_slack', .02).value)
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE)
        self.drive_pub = self.create_publisher(LongitudinalEffortSteering, self.declare_parameter('command_topic','effort_steering_command').value, output_qos)
        self.status_pub = self.create_publisher(DiagnosticArray, 'control/path_status', output_qos)
        self.scan_sub = self.create_subscription(LaserScan, self.declare_parameter('scan_topic','scan').value, self.scan_callback, scan_qos)

    def scan_callback(self, scan):
        stamp = original_stamp(scan.header)
        if stamp is None:
            self.contract.last_rejection = 'invalid_stamp'
            command = None
        else:
            command = self.contract.accept(
                scan, stamp=stamp, ros_now=self.get_clock().now().nanoseconds*1.e-9,
                monotonic_now=time.monotonic(), frame_id=scan.header.frame_id)
        publish_result(self, scan.header, command)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FrenetPurePursuit()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

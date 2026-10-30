#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import math
import threading
import time

from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.align import Align


def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw (rotation around Z axis)
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FancyOdometryViewer(Node):

    def __init__(self):
        super().__init__('fancy_odometry_viewer')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/pose_source/filtered',
            self.odom_callback,
            qos
        )

        self.pose_data = None
        self.last_msg_time = None
        self.lock = threading.Lock()

        self.get_logger().info("📡 Subscribed to /odometry/pose_source/filtered")

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.pose_data = msg
            self.last_msg_time = time.time()

    def build_table(self):
        table = Table(title="📍 Robot Pose (Odometry)", header_style="bold cyan")

        table.add_column("Field", style="bold")
        table.add_column("Value", justify="right")

        if self.pose_data is None:
            table.add_row("Status", "Waiting for data...")
            return table

        p = self.pose_data.pose.pose.position
        o = self.pose_data.pose.pose.orientation

        yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        yaw_deg = math.degrees(yaw)

        age = time.time() - self.last_msg_time if self.last_msg_time else 0.0

        table.add_row("Frame", self.pose_data.header.frame_id)
        table.add_row("Child Frame", self.pose_data.child_frame_id)

        table.add_row("x [m]", f"{p.x:.4f}")
        table.add_row("y [m]", f"{p.y:.4f}")
        table.add_row("z [m]", f"{p.z:.4f}")

        table.add_row("Yaw [rad]", f"{yaw:.4f}")
        table.add_row("Yaw [deg]", f"{yaw_deg:.2f}")

        table.add_row("Msg age [s]", f"{age:.3f}")

        return table

    def render(self):
        return Panel(
            Align.center(self.build_table()),
            title="🚗 ROS 2 Odometry Monitor",
            border_style="green"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FancyOdometryViewer()

    with Live(refresh_per_second=10, screen=True) as live:
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                live.update(node.render())
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

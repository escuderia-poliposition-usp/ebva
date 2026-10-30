from .output_paths import output_path
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import math
import numpy as np
import csv
import time
import os
import threading


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class CenterlineLogger(Node):

    def __init__(self):
        super().__init__('centerline_logger')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(
            Odometry,
            '/odometry/pose_source/filtered',
            self.odom_callback,
            qos
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.pose = None
        self.scan = None
        self.lock = threading.Lock()

        # CSV setup
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.declare_parameter('output_file', '')
        self.csv_path = str(output_path(self.get_parameter('output_file').value))
        self.csv_file = open(self.csv_path, 'x', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'x_center',
            'y_center',
            'yaw',
            'd_left',
            'd_right'
        ])

        self.get_logger().info(f"📄 Logging centerline to {self.csv_path}")

        # Timer for logging
        self.timer = self.create_timer(1, self.process)  # 1 Hz

    def odom_callback(self, msg):
        with self.lock:
            self.pose = msg
            print(f"msg_pose:{msg.pose}")

    def scan_callback(self, msg):
        with self.lock:
            self.scan = msg
            #print(f"msg_scan:{msg}")

    def extract_left_right(self, scan: LaserScan):
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        valid = np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        left_mask = (angles > np.deg2rad(70)) & (angles < np.deg2rad(110))
        right_mask = (angles > -np.deg2rad(110)) & (angles < -np.deg2rad(70))

        if not np.any(left_mask) or not np.any(right_mask):
            return None, None

        d_left = np.median(ranges[left_mask])
        d_right = np.median(ranges[right_mask])

        return d_left, d_right

    def compute_center(self, x, y, yaw, d_left, d_right):
        delta_y = (d_left - d_right) / 2.0
        x_c = x - delta_y * math.sin(yaw)
        y_c = y + delta_y * math.cos(yaw)
        return x_c, y_c

    def process(self):
        with self.lock:
            if self.pose is None or self.scan is None:
                return

            p = self.pose.pose.pose.position
            o = self.pose.pose.pose.orientation

            yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
            d_left, d_right = self.extract_left_right(self.scan)

            if d_left is None or d_right is None:
                return

            x_c, y_c = self.compute_center(p.x, p.y, yaw, d_left, d_right)

            print('----------------')
            print(f"Xc:{x_c},Yc:{y_c}")
            print('----------------')

            self.csv_writer.writerow([
                time.time(),
                x_c,
                y_c,
                yaw,
                d_left,
                d_right
            ])

            self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


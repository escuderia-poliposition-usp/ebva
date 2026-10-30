from .output_paths import output_path
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math
import csv
import time
import threading


class CenterlineLidarLogger(Node):

    def __init__(self):
        super().__init__('centerline_lidar_logger')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        self.create_subscription(
            Odometry,
            '/odometry/pose_source/filtered',
            self.odom_callback,
            qos
        )

        self.ey_pub = self.create_publisher(
            Float32,
            '/centerline/e_y',
            10
        )

        self.scan = None
        self.pose = None
        self.lock = threading.Lock()

        # --- params ---
        self.Ld = 0.6   # lookahead distance [m]

        # --- CSV ---
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.declare_parameter('output_file', '')
        self.csv_path = str(output_path(self.get_parameter('output_file').value))
        self.csv_file = open(self.csv_path, 'x', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'e_y',
            'y_center_Ld'
        ])

        self.timer = self.create_timer(0.05, self.process)  # 20 Hz
        self.get_logger().info("📡 Centerline LiDAR LOCAL logger running")

    # ---------------------------------------------------

    def scan_callback(self, msg):
        with self.lock:
            self.scan = msg

    def odom_callback(self, msg):
        with self.lock:
            self.pose = msg   # solo para timestamp / debug si quieres

    # ---------------------------------------------------

    def extract_wall_points(self, scan: LaserScan):
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        valid = np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        # Cartesian points in vehicle frame
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # Left / Right masks
        left_mask = (angles > np.deg2rad(30)) & (angles < np.deg2rad(120))
        right_mask = (angles < -np.deg2rad(30)) & (angles > -np.deg2rad(120))

        left_pts = np.vstack((xs[left_mask], ys[left_mask])).T
        right_pts = np.vstack((xs[right_mask], ys[right_mask])).T

        return left_pts, right_pts

    # ---------------------------------------------------

    def fit_line(self, pts):
        """
        Fit y = a*x + b
        """
        if pts.shape[0] < 5:
            return None

        x = pts[:, 0]
        y = pts[:, 1]

        A = np.vstack([x, np.ones_like(x)]).T
        a, b = np.linalg.lstsq(A, y, rcond=None)[0]
        return a, b

    # ---------------------------------------------------

    def process(self):
        with self.lock:
            if self.scan is None:
                return

            left_pts, right_pts = self.extract_wall_points(self.scan)

            left_line = self.fit_line(left_pts)
            right_line = self.fit_line(right_pts)

            if left_line is None or right_line is None:
                return

            a_l, b_l = left_line
            a_r, b_r = right_line

            # Centerline parameters
            a_c = 0.5 * (a_l + a_r)
            b_c = 0.5 * (b_l + b_r)

            # Evaluate centerline at lookahead
            y_center = a_c * self.Ld + b_c
            e_y = y_center   # en frame del vehículo

            # Publish
            msg = Float32()
            msg.data = float(e_y)
            self.ey_pub.publish(msg)

            # Log
            self.csv_writer.writerow([
                time.time(),
                e_y,
                y_center
            ])
            self.csv_file.flush()

    # ---------------------------------------------------

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineLidarLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

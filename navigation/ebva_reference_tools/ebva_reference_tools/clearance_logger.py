from .output_paths import output_path
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Float32


import math
import numpy as np
import csv
import time
import threading


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FrenetLogger(Node):

    def __init__(self):
        super().__init__('frenet_logger')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

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


        self.ey_pub = self.create_publisher(
            Float32,
            '/frenet/e_y',
            10
        )


        self.pose = None
        self.scan = None
        self.lock = threading.Lock()

        # Frenet state
        self.s = 0.0
        self.prev_x = None
        self.prev_y = None

        # CSV
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.declare_parameter('output_file', '')
        self.csv_path = str(output_path(self.get_parameter('output_file').value))
        self.csv_file = open(self.csv_path, 'x', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            's',
            'e_y',
            'yaw',
            'v',
            'd_left',
            'd_right'
        ])

        self.get_logger().info(f"📄 Logging Frenet data to {self.csv_path}")

        self.timer = self.create_timer(0.1, self.process)  # 10 Hz

    def odom_callback(self, msg):
        with self.lock:
            self.pose = msg

    def scan_callback(self, msg):
        with self.lock:
            self.scan = msg

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

        return np.median(ranges[left_mask]), np.median(ranges[right_mask])

    def process(self):
        with self.lock:
            if self.pose is None or self.scan is None:
                return

            p = self.pose.pose.pose.position
            o = self.pose.pose.pose.orientation
            twist = self.pose.twist.twist

            yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
            v = math.hypot(twist.linear.x, twist.linear.y)

            # Integrate s
            if self.prev_x is not None:
                ds = math.hypot(p.x - self.prev_x, p.y - self.prev_y)
                self.s += ds

            self.prev_x = p.x
            self.prev_y = p.y

            d_left, d_right = self.extract_left_right(self.scan)
            if d_left is None:
                return

            # Frenet lateral error
            e_y = (d_left - d_right) / 2.0

            # Publish Frenet e_y message in '/frenet/e_y' topic
            msg = Float32()
            msg.data = float(e_y)
            self.ey_pub.publish(msg)


            self.csv_writer.writerow([
                time.time(),
                self.s,
                e_y,
                yaw,
                v,
                d_left,
                d_right
            ])

            self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrenetLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

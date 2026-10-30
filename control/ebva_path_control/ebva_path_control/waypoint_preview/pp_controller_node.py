from ebva_path_control.geometry import quaternion_to_yaw
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


import numpy as np
import math
import csv
import os




class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ===================== PARAMETERS =====================
        self.declare_parameter('csv_path', '')
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('throttle', 0.3)

        pkg_share = os.path.join(get_package_share_directory('ebva_path_control'), 'data', 'waypoint_preview')

        csv_param = self.get_parameter('csv_path').value
        if csv_param:
            self.csv_path = csv_param
        else:
            self.csv_path = os.path.join(pkg_share, 'centerline.csv')

        self.Ld = self.get_parameter('lookahead_distance').value
        self.L = self.get_parameter('wheelbase').value
        self.throttle = self.get_parameter('throttle').value

        # ===================== LOAD WAYPOINTS =====================
        self.waypoints = self.load_waypoints(self.csv_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} centerline points")

        # ===================== SUB / PUB =====================
        self.create_subscription(
            Odometry,
            '/odometry/pose_source/filtered',
            self.odom_callback,
            10
        )

        self.pub_throttle = self.create_publisher(
            Float32,
            '/autodrive/f1tenth_1/throttle_command',
            10
        )

        self.pub_steer = self.create_publisher(
            Float32,
            '/autodrive/f1tenth_1/steering_command',
            10
        )

        #Just for rviz debug
        self.marker_pub = self.create_publisher(Marker, '/pp_debug', 10)


    # ============================================================
    def load_waypoints(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"CSV not found: {path}")
            return np.empty((0, 2))

        pts = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pts.append([float(row['x_center']), float(row['y_center'])])
        return np.array(pts)

    # ============================================================
    def odom_callback(self, msg):
        if len(self.waypoints) == 0:
            return

        pos = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        x, y = pos.x, pos.y

        # Closest waypoint
        dists = np.linalg.norm(self.waypoints - np.array([x, y]), axis=1)
        closest_idx = np.argmin(dists)

        # Lookahead waypoint
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(self.waypoints)):
            if np.linalg.norm(self.waypoints[i] - np.array([x, y])) >= self.Ld:
                lookahead_idx = i
                break

        target = self.waypoints[lookahead_idx]

        #Just for debug
        self.publish_lookahead_marker(target[0], target[1])

        # Transform to vehicle frame
        dx = target[0] - x
        dy = target[1] - y

        x_v =  math.cos(yaw) * dx + math.sin(yaw) * dy
        y_v = -math.sin(yaw) * dx + math.cos(yaw) * dy

        if x_v <= 0.0:
            return

        # Pure Pursuit
        curvature = 2.0 * y_v / (self.Ld ** 2)
        steering = math.atan(self.L * curvature)

        # Publish steering
        steer_msg = Float32()
        steer_msg.data = float(steering)
        #self.pub_steer.publish(steer_msg)
        print(steer_msg)

        # Publish throttle
        throttle_msg = Float32()
        throttle_msg.data = float(self.throttle)
        #self.pub_throttle.publish(throttle_msg)
        print(throttle_msg)

    # ============================================================
    def publish_lookahead_marker(self, x, y):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "pure_pursuit"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.r = 1.0
        m.color.a = 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

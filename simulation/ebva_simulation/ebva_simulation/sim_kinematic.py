#!/usr/bin/env python3
"""Kinematic bicycle-model simulator — drive path_follower_node.py with no hardware.

Subscribes to the same motor/servo command topics path_follower_node.py publishes,
inverts them back into (speed, steering_angle) using the IDENTICAL calibration
constants (erpm_per_ms, steering_gain/offset — same defaults as path_follower_node.py
and ackermann_to_vesc), integrates a bicycle model, and broadcasts the
map_frame -> base_frame TF that path_follower_node.py looks up. No /scan, ZED,
VESC, EKF or slam_toolbox required — this isolates the PID control loop only.

Subscribed topics:
    /commands/motor/speed     (std_msgs/Float64) — eRPM setpoint
    /commands/servo/position  (std_msgs/Float64) — servo position [0, 1]

Published:
    TF map_frame -> base_frame

Parameters:
    start_x, start_y, start_yaw (float) — initial pose (default: 0.0)
    wheelbase       (float) — metres, default 0.26 (this car's hardware wheelbase)
    map_frame       (str)   — default 'map'
    base_frame      (str)   — default 'zed_camera_link'
    rate_hz         (float) — integration rate, default 50.0
    steering_gain   (float) — must match path_follower_node.py (default -1.2135)
    steering_offset (float) — must match path_follower_node.py (default 0.5304)
    erpm_per_ms     (float) — must match path_follower_node.py (default 4614.0)
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tf2_ros
from geometry_msgs.msg import TransformStamped


class KinematicSim(Node):
    def __init__(self):
        super().__init__('sim_kinematic')

        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('wheelbase', 0.26)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'zed_camera_link')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('steering_gain', -1.2135)
        self.declare_parameter('steering_offset', 0.5304)
        self.declare_parameter('erpm_per_ms', 4614.0)

        self._x = self.get_parameter('start_x').value
        self._y = self.get_parameter('start_y').value
        self._yaw = self.get_parameter('start_yaw').value
        self._wheelbase = self.get_parameter('wheelbase').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._sg = self.get_parameter('steering_gain').value
        self._so = self.get_parameter('steering_offset').value
        self._erpm_k = self.get_parameter('erpm_per_ms').value
        rate_hz = self.get_parameter('rate_hz').value
        self._dt = 1.0 / rate_hz

        self._speed_ms = 0.0
        self._steering_angle = 0.0

        self.create_subscription(
            Float64, '/commands/motor/speed', self._on_speed, 10)
        self.create_subscription(
            Float64, '/commands/servo/position', self._on_servo, 10)

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'KinematicSim ready: start=({self._x:.2f}, {self._y:.2f}, '
            f'{self._yaw:.2f}) wheelbase={self._wheelbase} '
            f'{self._map_frame} -> {self._base_frame}')

    def _on_speed(self, msg):
        self._speed_ms = msg.data / self._erpm_k

    def _on_servo(self, msg):
        # Invert path_follower_node.py's servo = steering_gain*steering + steering_offset
        self._steering_angle = (msg.data - self._so) / self._sg

    def _step(self):
        yaw_rate = (self._speed_ms / self._wheelbase) * math.tan(self._steering_angle)
        self._x += self._speed_ms * math.cos(self._yaw) * self._dt
        self._y += self._speed_ms * math.sin(self._yaw) * self._dt
        self._yaw += yaw_rate * self._dt

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self._yaw / 2.0)
        t.transform.rotation.w = math.cos(self._yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)


def main(argv=None):
    rclpy.init(args=argv)
    node = KinematicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

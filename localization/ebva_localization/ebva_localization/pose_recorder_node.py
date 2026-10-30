#!/usr/bin/env python3
"""Publish the robot's pose in the map frame for bag recording.

Looks up the TF map -> base_frame at a fixed rate and republishes it as a
PoseStamped on /robot_pose. Run this during the reference lap so the pose
can be recorded into a rosbag and later processed by extract_path.py.

Usage:
  ros2 run ebva_localization pose_recorder
  ros2 bag record -o ref_lap /robot_pose
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers the transform for geometry_msgs types


class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'zed_camera_link')

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        rate_hz = self.get_parameter('rate_hz').value

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self._pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'pose_recorder: {self._map_frame} -> {self._base_frame} @ {rate_hz} Hz')

    def _tick(self):
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self._pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class BaseLinkPosePublisher(Node):

    def __init__(self):
        super().__init__('base_link_pose_publisher')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('publish_rate', 20.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/base_link_pose',
            10
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self.timer_callback
        )

        self.get_logger().info(
            f"Publishing {self.source_frame} → {self.target_frame} as /base_link_pose"
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )

            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = self.source_frame

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            pose.pose.orientation = transform.transform.rotation

            self.pose_pub.publish(pose)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass


def main():
    rclpy.init()
    node = BaseLinkPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


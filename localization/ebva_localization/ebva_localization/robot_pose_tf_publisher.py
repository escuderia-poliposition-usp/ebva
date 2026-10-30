#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RobotPoseTFPublisher(Node):

    def __init__(self):
        super().__init__('robot_pose_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/pose_source/filtered',
            self.odom_callback,
            10
        )

        self.get_logger().info("robot_pose TF publisher started")

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp

        # TF parent/child
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = "robot_pose"

        # Copy pose
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        # publish TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

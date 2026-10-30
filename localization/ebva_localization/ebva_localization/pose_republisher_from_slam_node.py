#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from copy import deepcopy
from .pose_values import pose_covariance

class SlamPoseRepublisher(Node):
    def __init__(self):
        super().__init__('slam_pose_republisher')
        self.declare_parameter('input_topic', '/pose_stamped')
        self.declare_parameter('output_topic', '/slam_pose')
        self.declare_parameter('rate_hz', 2.0)
        # default covariance (6x6 row-major) - tune this before fusing
        self.declare_parameter('covariance', pose_covariance()) # place variances in diag slots

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        cov_param = self.get_parameter('covariance').get_parameter_value().double_array_value
        # ensure length 36
        self.covariance = pose_covariance(cov_param)

        self.last_pose = None
        self.sub = self.create_subscription(PoseStamped, self.input_topic, self.pose_cb, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        # timer to republish last pose at a fixed rate
        period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(period, self.timer_cb)
        self.get_logger().info(f'Republishing {self.input_topic} -> {self.output_topic} at {self.rate_hz} Hz')

    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def timer_cb(self):
        if self.last_pose is None:
            return
        out = PoseWithCovarianceStamped()
        out.header.stamp = deepcopy(self.last_pose.header.stamp)
        out.header.frame_id = self.last_pose.header.frame_id  # likely 'map'
        out.pose.pose = deepcopy(self.last_pose.pose)
        out.pose.covariance = self.covariance
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


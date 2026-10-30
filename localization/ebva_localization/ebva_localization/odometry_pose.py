"""Opt-in pose projection from an explicitly declared Odometry state port."""
from copy import deepcopy
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .state_contract import PoseGate, PosePolicy


class OdometryPose(Node):
    def __init__(self, **kwargs):
        super().__init__("odometry_pose", **kwargs)
        self.declare_parameter("input_topic", "state_odometry")
        self.declare_parameter("output_topic", "state_pose")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("child_frame", "")
        self.declare_parameter("max_age_sec", 0.0)
        self.declare_parameter("future_tolerance_sec", 0.0)
        self.declare_parameter("quaternion_norm_tolerance", 1e-6)
        self.declare_parameter("covariance_tolerance", 1e-10)
        self.policy = PosePolicy(**{
            name: self.get_parameter(name).value
            for name in PosePolicy.__dataclass_fields__
        })
        self.gate = PoseGate(self.policy)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("output_topic").value, qos)
        self.status_publisher = self.create_publisher(String, "~/status", qos)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter("input_topic").value, self.on_odometry, qos)
        self.accepted_count = 0
        self.rejected_count = 0
        self.get_logger().info(
            f"Pose of {self.policy.child_frame} in {self.policy.parent_frame}; "
            "no transform or TF publication; acquisition stamps/covariance copied")

    def on_odometry(self, message):
        reason = self.gate.accept(message, self.get_clock().now().nanoseconds)
        if reason is None:
            output = PoseWithCovarianceStamped()
            output.header = deepcopy(message.header)
            output.pose = deepcopy(message.pose)
            self.publisher.publish(output)
            self.accepted_count += 1
        else:
            self.rejected_count += 1
        status = String()
        status.data = json.dumps({
            "accepted": reason is None,
            "reason": reason or "measurement_accepted",
            "accepted_count": self.accepted_count,
            "rejected_count": self.rejected_count,
            "parent_frame": self.policy.parent_frame,
            "child_frame": self.policy.child_frame,
            "clock_reset_required": self.gate.clock_reset_required,
        }, sort_keys=True)
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OdometryPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

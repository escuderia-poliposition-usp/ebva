import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class CmdVelToAckermannNode(Node):
    """Converts Nav2's /cmd_vel (Twist) into AckermannDriveStamped for ackermann_to_vesc.

    Nav2's controller server always publishes Twist (linear.x, angular.z) regardless of
    which controller plugin is running, even for a car-like robot. This node applies the
    bicycle model to recover a steering angle from that Twist:

        steering_angle = atan2(wheelbase * angular_z, linear_x)

    atan2 (rather than atan) keeps the output well-defined at linear_x == 0 instead of
    dividing by zero; the result is then clamped to the physical steering limit.
    """

    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_node')

        self.declare_parameter('wheelbase', 0.26)          # meters — from f1tenth_ws/vesc.yaml (measured)
        self.declare_parameter('max_steering_angle', 0.4)  # radians — MEASURE on the real car
        self.declare_parameter('frame_id', 'zed_camera_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ackermann_cmd_topic', 'ackermann_cmd')

        self._wheelbase = self.get_parameter('wheelbase').value
        self._max_steering_angle = self.get_parameter('max_steering_angle').value
        self._frame_id = self.get_parameter('frame_id').value

        self._sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._cmd_vel_callback,
            10,
        )
        self._pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('ackermann_cmd_topic').value,
            10,
        )

    def _cmd_vel_callback(self, msg: Twist):
        steering_angle = math.atan2(self._wheelbase * msg.angular.z, msg.linear.x)
        steering_angle = max(-self._max_steering_angle,
                              min(self._max_steering_angle, steering_angle))

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id
        out.drive.speed = msg.linear.x
        out.drive.steering_angle = steering_angle

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

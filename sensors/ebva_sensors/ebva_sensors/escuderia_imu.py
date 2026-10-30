#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuFrameFixer(Node):
    def __init__(self):
        super().__init__('imu_frame_fixer')

        # Parameters
        self.declare_parameter("input_topic", "/sensors/imu/raw")
        self.declare_parameter("output_topic", "/escuderia/imu")
        self.declare_parameter("frame_id", "imu_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.frame_id = self.get_parameter("frame_id").value

        # Subscriber
        self.sub = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10
        )

        # Publisher
        self.pub = self.create_publisher(
            Imu,
            output_topic,
            10
        )

        self.get_logger().info(
            f"IMU Frame Fixer running:\n"
            f"  input:  {input_topic}\n"
            f"  output: {output_topic}\n"
            f"  frame_id set to: '{self.frame_id}'"
        )

    def imu_callback(self, msg: Imu):
        new_msg = Imu()
        new_msg.header = msg.header
        new_msg.header.frame_id = self.frame_id

        new_msg.orientation = msg.orientation #parte nova tirar qualquer ocisa
        new_msg.angular_velocity = msg.angular_velocity #parte nova tirar qualquer ocisa
        new_msg.linear_acceleration = msg.linear_acceleration #parte nova tirar qualquer ocisa

            # ✅ Injeta covariâncias não-zero
        new_msg.orientation_covariance = [
            0.05, 0.0,  0.0,
            0.0,  0.05, 0.0,
            0.0,  0.0,  0.2
        ]
        new_msg.angular_velocity_covariance = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.02
        ]
        new_msg.linear_acceleration_covariance = [
            0.04, 0.0,  0.0,
            0.0,  0.04, 0.0,
            0.0,  0.0,  0.04
        ]

        # Publish it
        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

        # Copy the message
      #  new_msg = Imu()
      ##  new_msg = msg  # reuse the same message object

        # Fix the frame id
       # new_msg.header.frame_id = self.frame_id

#!/usr/bin/env python3
"""Phase 3 — VESC IMU unit converter (g, deg/s) -> SI (m/s^2, rad/s).

The VESC driver publishes `/sensors/imu/raw` as a sensor_msgs/Imu, but the fields
are NOT in SI units (confirmed in vesc_packet.cpp): linear_acceleration is in g
(with gravity, uncompensated) and angular_velocity is in deg/s. It also leaves
header.frame_id empty and every covariance at zero.

robot_localization's ekf_node requires strict SI (m/s^2, rad/s), a non-empty
frame_id it can transform from, and non-zero covariances (a zero covariance reads
as "infinitely certain" and makes the filter trust the sensor blindly). So the raw
topic cannot be fed to the EKF directly -- this node republishes a corrected copy.

For Phase 3 the EKF fuses ONLY this IMU's yaw-rate (angular_velocity.z); the
acceleration is converted too (so the topic is correct for anyone else) but is not
fused. The firmware's fused orientation quaternion is passed through untouched.

Publishes `/sensors/imu/si` (sensor_msgs/Imu). See scripts/imu_odometry.py for the
same ×9.806 / ×(pi/180) conversion applied inline there.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class VescImuToSi(Node):
    def __init__(self):
        super().__init__('vesc_imu_to_si')

        self.declare_parameter('input_topic', '/sensors/imu/raw')
        self.declare_parameter('output_topic', '/sensors/imu/si')
        # Frame the converted IMU is expressed in. Default to the EKF's
        # base_link_frame so no extra static TF is needed; for a pure yaw-rate
        # fusion the small IMU-vs-base mount offset is immaterial.
        self.declare_parameter('frame_id', 'zed_camera_link')
        self.declare_parameter('accel_to_ms2', 9.806)      # g -> m/s^2
        # Diagonal measurement covariances (non-zero so the EKF doesn't treat the
        # readings as perfect). Only the gyro one actually matters for Phase 3.
        self.declare_parameter('angular_velocity_variance', 0.01)     # (rad/s)^2
        self.declare_parameter('linear_acceleration_variance', 0.25)  # (m/s^2)^2

        self._frame_id = self.get_parameter('frame_id').value
        self._accel_to_ms2 = self.get_parameter('accel_to_ms2').value
        self._deg_to_rad = math.pi / 180.0
        self._gyro_var = self.get_parameter('angular_velocity_variance').value
        self._accel_var = self.get_parameter('linear_acceleration_variance').value

        self._pub = self.create_publisher(
            Imu, self.get_parameter('output_topic').value, 10)
        self._sub = self.create_subscription(
            Imu, self.get_parameter('input_topic').value, self._callback, 10)

    def _callback(self, msg: Imu):
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id

        # deg/s -> rad/s
        out.angular_velocity.x = msg.angular_velocity.x * self._deg_to_rad
        out.angular_velocity.y = msg.angular_velocity.y * self._deg_to_rad
        out.angular_velocity.z = msg.angular_velocity.z * self._deg_to_rad

        # g -> m/s^2 (still carries gravity; not fused in Phase 3)
        out.linear_acceleration.x = msg.linear_acceleration.x * self._accel_to_ms2
        out.linear_acceleration.y = msg.linear_acceleration.y * self._accel_to_ms2
        out.linear_acceleration.z = msg.linear_acceleration.z * self._accel_to_ms2

        # Pass the firmware-fused orientation through unchanged.
        out.orientation = msg.orientation

        # Non-zero diagonal covariances (row-major 3x3). We do not fuse the
        # orientation, so flag it unavailable per the sensor_msgs/Imu convention.
        out.orientation_covariance = [-1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0]
        out.angular_velocity_covariance = [self._gyro_var, 0.0, 0.0,
                                           0.0, self._gyro_var, 0.0,
                                           0.0, 0.0, self._gyro_var]
        out.linear_acceleration_covariance = [self._accel_var, 0.0, 0.0,
                                              0.0, self._accel_var, 0.0,
                                              0.0, 0.0, self._accel_var]

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VescImuToSi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

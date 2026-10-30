#!/usr/bin/env python3
"""Phase 2b — IMU-only dead-reckoning odometry (the deliberately weak prior).

Phase 2b feeds slam_toolbox the *worst possible* motion prior: odometry derived
purely from the car's IMU (`/sensors/imu/raw`, the VESC's onboard MPU). It exists
to show that slam_toolbox's scan matching + loop closure "equalizes" the map
regardless of how bad the prior is (the 2a/2b/2c comparison). This is NOT a fusion
phase — that's Phase 3 (robot_localization EKF).

How the odometry is built (simple strapdown dead reckoning):
  - Heading comes straight from the VESC's onboard fused `orientation` quaternion
    (the firmware already runs a complementary filter), so we do NOT integrate the
    gyro. Heading is therefore stable; the drift is isolated to translation.
  - Position is double-integrated from `linear_acceleration`: rotate the body-frame
    specific force into the world frame with the quaternion, subtract the estimated
    gravity+bias vector, then integrate twice. This drifts quadratically — exactly
    the point.

Two things that MUST happen around the double-integration or it diverges in ~1 s:
  - Unit conversion: `/sensors/imu/raw` is NOT in SI units. The VESC publishes
    acceleration in g and angular velocity in deg/s (vesc_packet.cpp). We convert
    accel to m/s^2 here (gyro is unused since heading comes from the quaternion).
  - Startup bias/gravity calibration: while the car sits still at startup we average
    the world-frame acceleration to estimate the constant gravity+bias vector that
    gets subtracted every step. Integration is held at zero until this completes.

Publishes `nav_msgs/msg/Odometry` on `/odom` and broadcasts the `odom -> base_link`
TF, which slam_toolbox consumes as its motion prior (base_frame: base_link).
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def _quat_to_rot(q):
    """3x3 rotation matrix (body -> world) from a geometry_msgs Quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')

        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # /sensors/imu/raw acceleration is in g (vesc_packet.cpp). Convert to m/s^2.
        self.declare_parameter('accel_to_ms2', 9.806)
        # Number of stationary samples averaged at startup to estimate gravity+bias.
        self.declare_parameter('calib_samples', 200)
        # Zero-velocity update: zero the velocity when the car is judged still, to
        # curb runaway. ON by default so the prior stays locally consistent enough for
        # slam_toolbox to keep scan-match lock (the "good map" config). Turn OFF to
        # expose the honest quadratic drift of the worst-possible prior.
        self.declare_parameter('zupt_enable', True)
        self.declare_parameter('zupt_accel_thresh', 0.2)   # m/s^2 (world horizontal)
        self.declare_parameter('zupt_gyro_thresh', 0.05)   # rad/s (gyro_z magnitude)
        # Hard cap on integrated speed (m/s). Insurance against velocity runaway between
        # ZUPT events, so the per-keyframe odom seed stays inside slam_toolbox's scan
        # search window. A physical bound, not a leak. Set huge (e.g. 1e9) to disable.
        self.declare_parameter('max_speed', 4.0)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.accel_to_ms2 = float(self.get_parameter('accel_to_ms2').value)
        self.calib_samples = int(self.get_parameter('calib_samples').value)
        self.zupt_enable = bool(self.get_parameter('zupt_enable').value)
        self.zupt_accel_thresh = float(self.get_parameter('zupt_accel_thresh').value)
        self.zupt_gyro_thresh = float(self.get_parameter('zupt_gyro_thresh').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        # Integration state.
        self.pos = np.zeros(2)        # x, y in odom_frame
        self.vel = np.zeros(2)        # vx, vy in odom_frame
        self.last_stamp = None
        self.yaw = 0.0
        # Heading at calibration end; subtracted so odom starts at identity (ROS convention).
        self.yaw0 = None

        # Gravity+bias calibration (world frame).
        self.grav_bias = np.zeros(3)
        self.calib_accum = np.zeros(3)
        self.calib_count = 0
        self.calibrated = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)

        self.get_logger().info("imu_topic   : %s" % self.imu_topic)
        self.get_logger().info("odom_topic  : %s" % self.odom_topic)
        self.get_logger().info("frames      : %s -> %s" % (self.odom_frame, self.base_frame))
        self.get_logger().info("accel_to_ms2: %.4f" % self.accel_to_ms2)
        self.get_logger().info("zupt_enable : %s" % self.zupt_enable)
        self.get_logger().info("max_speed   : %.2f m/s" % self.max_speed)
        self.get_logger().info(
            "Calibrating gravity+bias over %d stationary samples — keep the car STILL."
            % self.calib_samples)

    def _on_imu(self, msg):
        R = _quat_to_rot(msg.orientation)
        a_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]) * self.accel_to_ms2
        a_world = R @ a_body

        # --- Calibration phase: estimate the constant gravity+bias vector ---
        if not self.calibrated:
            self.calib_accum += a_world
            self.calib_count += 1
            if self.calib_count >= self.calib_samples:
                self.grav_bias = self.calib_accum / self.calib_count
                self.yaw0 = _yaw_from_quat(msg.orientation)
                self.calibrated = True
                self.last_stamp = msg.header.stamp
                self.get_logger().info(
                    "Calibration done. gravity+bias (world) = [%.3f, %.3f, %.3f] m/s^2"
                    % tuple(self.grav_bias))
            # Publish the identity pose (pos/vel still zero) so `odom -> base_link` exists
            # from the first sample and slam_toolbox doesn't drop the opening scans.
            self._publish(msg.header.stamp)
            return

        # dt from consecutive IMU stamps.
        stamp = msg.header.stamp
        if self.last_stamp is not None:
            dt = ((stamp.sec - self.last_stamp.sec)
                  + (stamp.nanosec - self.last_stamp.nanosec) * 1e-9)
        else:
            dt = 0.0
        self.last_stamp = stamp
        if dt <= 0.0 or dt > 1.0:
            # First sample, clock jump, or stale: skip integration this step.
            self._publish(stamp)
            return

        # Heading straight from the onboard quaternion (no gyro integration), re-based
        # to the startup heading so odom begins at identity.
        self.yaw = math.atan2(math.sin(_yaw_from_quat(msg.orientation) - self.yaw0),
                              math.cos(_yaw_from_quat(msg.orientation) - self.yaw0))

        # Remove gravity+bias; keep horizontal motion only (2D map).
        a_lin = a_world - self.grav_bias
        # Re-base into the odom frame (rotated by -yaw0) so the integrated position
        # stays consistent with the rebased heading reported below.
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        a_xy = np.array([c0 * a_lin[0] + s0 * a_lin[1],
                         -s0 * a_lin[0] + c0 * a_lin[1]])

        if (self.zupt_enable
                and np.linalg.norm(a_xy) < self.zupt_accel_thresh
                and abs(msg.angular_velocity.z * math.pi / 180.0) < self.zupt_gyro_thresh):
            self.vel[:] = 0.0
        else:
            self.vel += a_xy * dt
            # Cap integrated speed so a runaway velocity can't throw the per-keyframe
            # odom seed outside slam_toolbox's scan-match search window.
            speed = np.linalg.norm(self.vel)
            if speed > self.max_speed:
                self.vel *= self.max_speed / speed
            self.pos += self.vel * dt

        self._publish(stamp)

    def _publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.pos[0])
        odom.pose.pose.position.y = float(self.pos[1])
        odom.pose.pose.orientation = _yaw_to_quat(self.yaw)
        # Body-frame linear velocity (rotate world velocity back by yaw).
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        odom.twist.twist.linear.x = float(c * self.vel[0] + s * self.vel[1])
        odom.twist.twist.linear.y = float(-s * self.vel[0] + c * self.vel[1])
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.pos[0])
            t.transform.translation.y = float(self.pos[1])
            t.transform.rotation = _yaw_to_quat(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main(argv=None):
    rclpy.init(args=argv)
    node = ImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Opt-in SI boundary for the preserved research IMU translation prior.

No driver units, extrinsics, physical uncertainty or firmware behavior are
inferred here. See docs/imu-prior-contract.md for mandatory composition choices.
"""
from copy import deepcopy
from dataclasses import fields, MISSING
import json
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .imu_prior import ImuPrior, ImuPriorPolicy, ImuSample, quaternion_rotation
from .state_contract import stamp_nanoseconds


def _quat_to_rot(q):
    """Retained mathematical helper; the new input gate rejects null quaternions."""
    return quaternion_rotation((q.x, q.y, q.z, q.w))


def _yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw*0.5)
    q.z = math.sin(yaw*0.5)
    return q


class ImuOdometry(Node):
    def __init__(self, **kwargs):
        super().__init__("imu_odometry", **kwargs)
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        self.declare_parameter("input_topic", "imu_si")
        self.declare_parameter("output_topic", "imu_prior_odometry")
        empty = {
            "input_frame": "", "parent_frame": "", "child_frame": "",
            "orientation_reference_frame": "", "input_acceleration_units": "",
            "input_angular_velocity_units": "", "acceleration_content": "",
            "calibration_stationary_asserted": False, "calib_samples": 200,
            "pose_covariance": [0.0]*36, "twist_covariance": [0.0]*36,
            "covariance_provenance": ""}
        values = {}
        for field in fields(ImuPriorPolicy):
            default = empty.get(field.name, field.default if field.default is not MISSING else 0.0)
            self.declare_parameter(field.name, default)
            values[field.name] = self.get_parameter(field.name).value
        self.policy = ImuPriorPolicy(**values)
        self.prior = ImuPrior(self.policy)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Odometry, self.get_parameter("output_topic").value, qos)
        self.status_publisher = self.create_publisher(String, "~/status", qos)
        self.subscription = self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self.on_imu, qos)
        self.accepted_count = 0
        self.rejected_count = 0

    def on_imu(self, msg):
        try:
            stamp_ns = stamp_nanoseconds(msg.header.stamp)
            covariances = (msg.orientation_covariance, msg.linear_acceleration_covariance,
                           msg.angular_velocity_covariance)
            if not all(np.isfinite(v).all() for v in covariances):
                raise ValueError("nonfinite_input_covariance")
            sample = ImuSample(
                stamp_ns, msg.header.frame_id,
                (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
                (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z),
                (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                msg.orientation_covariance[0] != -1.,
                msg.linear_acceleration_covariance[0] != -1.,
                msg.angular_velocity_covariance[0] != -1.)
            output, reason = self.prior.process(sample, self.get_clock().now().nanoseconds)
        except ValueError as error:
            output, reason = self.prior._reject(str(error))
        if output is not None:
            odom = Odometry()
            odom.header.stamp = deepcopy(msg.header.stamp)
            odom.header.frame_id = self.policy.parent_frame
            odom.child_frame_id = self.policy.child_frame
            odom.pose.pose.position.x, odom.pose.pose.position.y = output["position"]
            odom.pose.pose.orientation = _yaw_to_quat(output["yaw"])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y = output["body_velocity"]
            # Angular twist remains unsupported (zero placeholder); do not claim it
            # as a measured/estimated capability in a consuming profile.
            odom.pose.covariance = list(self.policy.pose_covariance)
            odom.twist.covariance = list(self.policy.twist_covariance)
            self.publisher.publish(odom)
            self.accepted_count += 1
        elif reason not in ("calibrating", "calibration_complete"):
            self.rejected_count += 1
        status = self.prior.status(reason)
        status.update(published=output is not None, accepted_count=self.accepted_count,
                      rejected_count=self.rejected_count)
        self.status_publisher.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuOdometry()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()

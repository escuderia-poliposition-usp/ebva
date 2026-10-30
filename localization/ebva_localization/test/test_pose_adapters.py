"""Data contracts without constructing nodes or opening any device."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ebva_localization.pose_values import pose_covariance
from ebva_localization.pose_republisher_from_slam_node import SlamPoseRepublisher
from ebva_localization.robot_pose_tf_publisher import RobotPoseTFPublisher
from ebva_localization.imu_odometry import _quat_to_rot, _yaw_to_quat


class PoseAdapterTests(unittest.TestCase):
    def test_default_covariance_is_diagonal(self):
        matrix = np.array(pose_covariance()).reshape(6, 6)
        np.testing.assert_array_equal(matrix, np.eye(6) * 0.5)

    def test_invalid_covariance_is_rejected(self):
        for value in ([0.0], [0.0] * 37, [math.nan] * 36):
            with self.assertRaises(ValueError):
                pose_covariance(value)

    def test_republishing_preserves_measurement_time_and_does_not_alias_pose(self):
        source = PoseStamped()
        source.header.stamp.sec, source.header.stamp.nanosec = 23, 45
        source.header.frame_id = "odom"
        source.pose.position.x = 1.25
        fake = SimpleNamespace(last_pose=source, covariance=pose_covariance(), pub=Mock())
        SlamPoseRepublisher.timer_cb(fake)
        out = fake.pub.publish.call_args.args[0]
        self.assertEqual(out.header, source.header)
        self.assertEqual(out.pose.pose, source.pose)
        out.pose.pose.position.x = 99.0
        out.header.stamp.sec = 99
        self.assertEqual(source.pose.position.x, 1.25)
        self.assertEqual(source.header.stamp.sec, 23)

    def test_odom_tf_uses_received_parent_frame(self):
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.header.stamp.sec = 7
        msg.pose.pose.position.x = 3.0
        fake = SimpleNamespace(tf_broadcaster=Mock())
        RobotPoseTFPublisher.odom_callback(fake, msg)
        out = fake.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(out.header, msg.header)
        self.assertEqual(out.child_frame_id, "robot_pose")
        self.assertEqual(out.transform.translation.x, 3.0)

    def test_inertial_prior_rotation_preserves_geometry(self):
        q = _yaw_to_quat(math.pi / 2)
        np.testing.assert_allclose(_quat_to_rot(q) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        q.w *= 4
        q.z *= 4
        np.testing.assert_allclose(_quat_to_rot(q) @ [1, 0, 0], [0, 1, 0], atol=1e-12)

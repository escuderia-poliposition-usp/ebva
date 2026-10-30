"""Directed state-port checks using real ROS messages and unchanged values."""
from copy import deepcopy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from nav_msgs.msg import Odometry
from ebva_localization.odometry_pose import OdometryPose
from ebva_localization.state_contract import PoseGate, PosePolicy, covariance_reason


def policy():
    # Explicit software fixture; not a vehicle profile.
    return PosePolicy("fixture_map", "fixture_base", 0.2, 0.02)


def odometry(stamp_ns=10_000_000_000):
    message = Odometry()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.header.frame_id = "fixture_map"
    message.child_frame_id = "fixture_base"
    message.pose.pose.position.x = 1.25
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance = (np.diag([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])).ravel().tolist()
    return message


def fake_node():
    fake = SimpleNamespace(gate=PoseGate(policy()), policy=policy(),
                           publisher=Mock(), status_publisher=Mock(),
                           accepted_count=0, rejected_count=0,
                           now_ns=10_000_000_000)
    fake.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=fake.now_ns))
    return fake


class StateContractTests(unittest.TestCase):
    def test_requires_declared_frames_and_time_policy(self):
        for values in (("", "base", .2, .02), ("map", "", .2, .02),
                       ("map", "map", .2, .02), ("map", "base", 0., .02),
                       ("map", "base", math.nan, .02),
                       ("map", "base", .2, -1.),
                       (" map", "base", .2, .02)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                PosePolicy(*values)

    def test_positive_and_negative_position_and_rotation_preserved(self):
        fake = fake_node()
        message = odometry()
        message.pose.pose.position.x = -3.25
        message.pose.pose.position.y = 4.5
        message.pose.pose.orientation.z = math.sin(-0.7)
        message.pose.pose.orientation.w = math.cos(-0.7)
        before = deepcopy(message)
        OdometryPose.on_odometry(fake, message)
        output = fake.publisher.publish.call_args.args[0]
        self.assertEqual(output.header, message.header)
        self.assertEqual(output.pose, message.pose)
        self.assertEqual(before, message)
        output.pose.pose.position.x = 99.0
        output.header.stamp.sec = 99
        self.assertEqual(message, before)

    def test_rejects_wrong_frames_without_relabeling(self):
        for attribute in ("parent", "child"):
            message = odometry()
            if attribute == "parent":
                message.header.frame_id = "odom"
            else:
                message.child_frame_id = "camera"
            self.assertEqual(PoseGate(policy()).accept(message, 10_000_000_000),
                             attribute + "_frame_mismatch")

    def test_zero_covariance_is_preserved_without_asserting_uncertainty(self):
        fake = fake_node()
        message = odometry()
        message.pose.covariance = [0.] * 36
        OdometryPose.on_odometry(fake, message)
        self.assertEqual(list(fake.publisher.publish.call_args.args[0].pose.covariance), [0.] * 36)

    def test_correlated_psd_covariance_is_copied_exactly(self):
        fake = fake_node()
        message = odometry()
        matrix = np.eye(6)
        matrix[0, 1] = matrix[1, 0] = .25
        message.pose.covariance = matrix.ravel().tolist()
        OdometryPose.on_odometry(fake, message)
        np.testing.assert_array_equal(fake.publisher.publish.call_args.args[0].pose.covariance, message.pose.covariance)

    def test_covariance_invalid_nonfinite_asymmetric_and_indefinite(self):
        matrix = np.eye(6)
        matrix[0, 1] = 2.
        self.assertEqual(covariance_reason(matrix.ravel(), 1e-10), "asymmetric_covariance")
        matrix[1, 0] = 2.
        self.assertEqual(covariance_reason(matrix.ravel(), 1e-10), "indefinite_covariance")
        self.assertEqual(covariance_reason([math.inf] * 36, 1e-10), "invalid_covariance")
        self.assertEqual(covariance_reason([0.] * 35, 1e-10), "invalid_covariance")

    def test_covariance_tolerance_does_not_modify_accepted_values(self):
        fake = fake_node()
        message = odometry()
        message.pose.covariance[1] = 5e-11
        OdometryPose.on_odometry(fake, message)
        np.testing.assert_array_equal(fake.publisher.publish.call_args.args[0].pose.covariance, message.pose.covariance)

    def test_rejects_nonfinite_position_and_orientation(self):
        for value in (math.nan, math.inf, -math.inf):
            for field in ("position", "orientation"):
                message = odometry()
                setattr(getattr(message.pose.pose, field), "x", value)
                self.assertEqual(PoseGate(policy()).accept(message, 10_000_000_000), "nonfinite_pose")

    def test_rejects_zero_or_scaled_quaternion_without_normalizing(self):
        for w in (0., 2.):
            message = odometry()
            message.pose.pose.orientation.w = w
            self.assertEqual(PoseGate(policy()).accept(message, 10_000_000_000), "nonunit_quaternion")
            self.assertEqual(message.pose.pose.orientation.w, w)

    def test_age_boundary_and_future_tolerance(self):
        self.assertIsNone(PoseGate(policy()).accept(odometry(), 10_199_999_999))
        self.assertEqual(PoseGate(policy()).accept(odometry(), 10_200_000_000), "measurement_expired")
        self.assertIsNone(PoseGate(policy()).accept(odometry(10_020_000_000), 10_000_000_000))
        self.assertEqual(PoseGate(policy()).accept(odometry(10_020_000_001), 10_000_000_000),
                         "measurement_in_future")

    def test_duplicate_and_out_of_order_do_not_publish_or_move_cursor(self):
        fake = fake_node()
        for stamp in (10_000_000_000, 10_000_000_000, 9_990_000_000):
            OdometryPose.on_odometry(fake, odometry(stamp))
        self.assertEqual(fake.publisher.publish.call_count, 1)
        self.assertEqual(fake.gate.last_stamp_ns, 10_000_000_000)
        self.assertEqual(fake.rejected_count, 2)

    def test_invalid_does_not_consume_stamp_and_recovery_uses_new_valid_sample(self):
        fake = fake_node()
        message = odometry()
        message.pose.pose.orientation.w = 0.
        OdometryPose.on_odometry(fake, message)
        self.assertIsNone(fake.gate.last_stamp_ns)
        OdometryPose.on_odometry(fake, odometry())
        self.assertEqual(fake.publisher.publish.call_count, 1)

    def test_nil_stamp_and_unavailable_clock(self):
        self.assertEqual(PoseGate(policy()).accept(odometry(0), 10_000_000_000), "invalid_stamp")
        self.assertEqual(PoseGate(policy()).accept(odometry(), 0), "clock_unavailable")

    def test_clock_regression_requires_recreation_not_silent_reset(self):
        gate = PoseGate(policy())
        self.assertIsNone(gate.accept(odometry(), 10_000_000_000))
        self.assertEqual(gate.accept(odometry(8_000_000_000), 8_000_000_000), "clock_reset_required")
        self.assertEqual(gate.accept(odometry(11_000_000_000), 11_000_000_000), "clock_reset_required")
        self.assertIsNone(PoseGate(policy()).accept(odometry(8_000_000_000), 8_000_000_000))

    def test_projection_does_not_read_twist_or_fabricate_velocity(self):
        fake = fake_node()
        message = odometry()
        message.twist.twist.linear.x = math.nan
        # This port projects pose only; full Odometry consumers validate twist themselves.
        OdometryPose.on_odometry(fake, message)
        self.assertEqual(fake.publisher.publish.call_count, 1)

    def test_rejection_status_exposes_reason_and_counts(self):
        import json
        fake = fake_node()
        message = odometry()
        message.child_frame_id = "camera"
        OdometryPose.on_odometry(fake, message)
        state = json.loads(fake.status_publisher.publish.call_args.args[0].data)
        self.assertFalse(state["accepted"])
        self.assertEqual(state["reason"], "child_frame_mismatch")
        self.assertEqual(state["accepted_count"], 0)
        self.assertEqual(state["rejected_count"], 1)
        self.assertEqual(fake.publisher.publish.call_count, 0)

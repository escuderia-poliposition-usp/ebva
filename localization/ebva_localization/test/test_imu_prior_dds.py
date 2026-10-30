"""Actual DDS input, no timer/TF, silence and recovery of the installed prior."""
import json
import time
import unittest
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from ebva_localization.imu_odometry import ImuOdometry
from test_imu_prior import fixture_values, message


class ImuPriorDdsTests(unittest.TestCase):
    def test_invalid_configuration_destroys_the_partially_created_ros_node(self):
        context = Context(); rclpy.init(context=context)
        try:
            values = fixture_values(); values["calibration_stationary_asserted"] = False
            with patch.object(ImuOdometry, "destroy_node", autospec=True,
                              side_effect=Node.destroy_node) as destroyed:
                with self.assertRaises(ValueError):
                    ImuOdometry(context=context, namespace="/invalid_imu_prior_fixture",
                                parameter_overrides=[Parameter(k, value=v) for k, v in values.items()])
                self.assertEqual(destroyed.call_count, 1)
        finally:
            rclpy.shutdown(context=context)

    def test_calibration_rejection_silence_and_recovery_over_dds(self):
        context = Context(); rclpy.init(context=context)
        prior = observer = None
        executor = SingleThreadedExecutor(context=context)
        try:
            prior = ImuOdometry(context=context, namespace="/imu_prior_fixture",
                               parameter_overrides=[Parameter(k, value=v) for k, v in fixture_values().items()])
            observer = Node("observer", namespace="/imu_prior_fixture", context=context)
            outputs, statuses = [], []
            observer.create_subscription(Odometry, "imu_prior_odometry", outputs.append, 10)
            observer.create_subscription(String, "imu_odometry/status",
                                         lambda m: statuses.append(json.loads(m.data)), 10)
            publisher = observer.create_publisher(Imu, "imu_si", 10)
            executor.add_node(prior); executor.add_node(observer)

            def wait_for(predicate, timeout=4.):
                until = time.monotonic()+timeout
                while not predicate() and time.monotonic() < until:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(predicate())

            def publish(mutate=None):
                source = message(prior.get_clock().now().nanoseconds)
                if mutate:
                    mutate(source)
                count = len(statuses)
                publisher.publish(source)
                wait_for(lambda: len(statuses) > count)
                return source

            wait_for(lambda: publisher.get_subscription_count() == 1
                     and prior.publisher.get_subscription_count() == 1
                     and prior.status_publisher.get_subscription_count() == 1)
            for _ in range(4):
                publish()
            self.assertEqual(len(outputs), 0)
            self.assertEqual(statuses[-1]["reason"], "calibration_complete")
            source = publish(lambda m: setattr(m.linear_acceleration, "x", .5))
            wait_for(lambda: len(outputs) == 1)
            self.assertEqual(outputs[-1].header.stamp, source.header.stamp)
            self.assertEqual(outputs[-1].header.frame_id, "fixture_prior_odom")
            publish(lambda m: setattr(m.orientation, "w", 0.))
            self.assertEqual(statuses[-1]["reason"], "nonunit_quaternion")
            self.assertEqual(len(outputs), 1)
            publish()
            self.assertEqual(statuses[-1]["reason"], "integration_rebased_after_gap")
            publish()
            wait_for(lambda: len(outputs) == 2)
            previous_status = len(statuses)
            until = time.monotonic()+.45
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.02)
            self.assertEqual(len(statuses), previous_status)
            self.assertEqual(len(outputs), 2)
            publish()
            self.assertEqual(statuses[-1]["reason"], "integration_rebased_after_gap")
            self.assertEqual(len(outputs), 2)
            source = publish()
            wait_for(lambda: len(outputs) == 3)
            self.assertEqual(outputs[-1].header.stamp, source.header.stamp)
            self.assertFalse(statuses[-1]["uncertainty_estimated"])
            self.assertEqual(len(list(prior.timers)), 0)
            self.assertFalse(any(p.topic_name in ("/tf", "/tf_static") for p in prior.publishers))
        finally:
            executor.shutdown()
            if prior is not None: prior.destroy_node()
            if observer is not None: observer.destroy_node()
            rclpy.shutdown(context=context)

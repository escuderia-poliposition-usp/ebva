"""Software-only DDS check of the installed opt-in node and silence behavior."""
import json
import time
import unittest

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from ebva_localization.odometry_pose import OdometryPose
from test_state_contract import odometry


class OdometryPoseDdsTests(unittest.TestCase):
    def test_validity_rejection_silence_and_recovery_over_dds(self):
        context = Context()
        rclpy.init(context=context)
        adapter = None
        observer = None
        executor = SingleThreadedExecutor(context=context)
        try:
            adapter = OdometryPose(
                context=context, namespace="/localization_contract_fixture",
                parameter_overrides=[
                    Parameter("parent_frame", value="fixture_map"),
                    Parameter("child_frame", value="fixture_base"),
                    Parameter("max_age_sec", value=0.4),
                    Parameter("future_tolerance_sec", value=0.02),
                ])
            observer = Node("observer", namespace="/localization_contract_fixture", context=context)
            outputs = []
            statuses = []
            observer.create_subscription(PoseWithCovarianceStamped, "state_pose", outputs.append, 10)
            observer.create_subscription(String, "odometry_pose/status",
                                         lambda msg: statuses.append(json.loads(msg.data)), 10)
            publisher = observer.create_publisher(Odometry, "state_odometry", 10)
            executor.add_node(adapter)
            executor.add_node(observer)

            def wait_for(predicate, timeout=3.0):
                deadline = time.monotonic() + timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                self.assertTrue(predicate())

            def spin_for(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)

            wait_for(lambda: publisher.get_subscription_count() == 1
                     and adapter.publisher.get_subscription_count() == 1
                     and adapter.status_publisher.get_subscription_count() == 1)
            first = odometry(adapter.get_clock().now().nanoseconds)
            publisher.publish(first)
            wait_for(lambda: len(outputs) == 1 and len(statuses) == 1)
            self.assertEqual(outputs[0].header, first.header)
            self.assertEqual(outputs[0].pose, first.pose)

            publisher.publish(first)
            wait_for(lambda: len(statuses) == 2)
            self.assertEqual(statuses[-1]["reason"], "measurement_not_newer")
            self.assertEqual(len(outputs), 1)
            # No callback or timer publishes a cached pose during input silence.
            spin_for(0.45)
            self.assertEqual(len(outputs), 1)
            self.assertEqual(len(statuses), 2)
            stale = odometry(adapter.get_clock().now().nanoseconds - 800_000_000)
            publisher.publish(stale)
            wait_for(lambda: len(statuses) == 3)
            self.assertEqual(statuses[-1]["reason"], "measurement_expired")
            wrong = odometry(adapter.get_clock().now().nanoseconds)
            wrong.child_frame_id = "camera"
            publisher.publish(wrong)
            wait_for(lambda: len(statuses) == 4)
            self.assertEqual(statuses[-1]["reason"], "child_frame_mismatch")
            fresh = odometry(adapter.get_clock().now().nanoseconds)
            publisher.publish(fresh)
            wait_for(lambda: len(outputs) == 2 and len(statuses) == 5)
            self.assertEqual(outputs[-1].header, fresh.header)
            self.assertEqual(outputs[-1].pose, fresh.pose)
            self.assertEqual(statuses[-1]["accepted_count"], 2)
            self.assertEqual(statuses[-1]["rejected_count"], 3)
            self.assertEqual(len(list(adapter.timers)), 0)
        finally:
            executor.shutdown()
            if adapter is not None:
                adapter.destroy_node()
            if observer is not None:
                observer.destroy_node()
            rclpy.shutdown(context=context)

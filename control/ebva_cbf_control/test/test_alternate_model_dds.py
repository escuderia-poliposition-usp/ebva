"""Software-only typed yaw_rate node, no drivers or actuator topic publishers."""
import json
import time
import unittest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
import pytest
from ebva_cbf_control.alternate_model_node import CbfYawRateMarginNode, CbfSteeringTangentNode
from alternate_model_fixtures import FRAME, PARENT, scan, velocity, nominal


@pytest.mark.parametrize("node_cls", [CbfYawRateMarginNode, CbfSteeringTangentNode])
def test_alternate_typed_pair_gate_silence_and_fresh_recovery(node_cls):
        self = unittest.TestCase()
        model = node_cls.PORT.MODEL
        make_nominal = lambda stamp, acceleration=.1, lateral=.05: nominal(model, stamp, acceleration, lateral)
        context = Context()
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        node = observer = None
        try:
            node = node_cls(context=context, namespace="/cbf_ros_fixture", parameter_overrides=[
                Parameter("control_frame", value=FRAME), Parameter("velocity_parent_frame", value=PARENT),
                Parameter("model_reference_point", value="rear_axle_center"),
                Parameter("scan_geometry", value="coincident_aligned"),
                Parameter("timeout_s", value=.5), Parameter("max_skew_s", value=.2)])
            observer = Node("observer", namespace="/cbf_ros_fixture", context=context)
            outputs, statuses = [], []
            observer.create_subscription(node.PORT.COMMAND_TYPE, "control/autonomy/cbf_"+model, outputs.append, 10)
            observer.create_subscription(String, "cbf_"+model+"/status", lambda m: statuses.append(json.loads(m.data)), 10)
            scans = observer.create_publisher(LaserScan, "perception/scan", 10)
            speeds = observer.create_publisher(Odometry, "state/odometry", 10)
            nominals = observer.create_publisher(node.PORT.NOMINAL_TYPE, "control/autonomy/nominal", 10)
            executor.add_node(node)
            executor.add_node(observer)
            def wait_for(predicate, timeout=4.):
                deadline = time.monotonic()+timeout
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                self.assertTrue(predicate())
            def send(publisher, message, source):
                count = len(statuses)
                publisher.publish(message)
                wait_for(lambda: len(statuses)>count and statuses[-1]["source"] == source)
            wait_for(lambda: all(p.get_subscription_count() == 1 for p in (scans, speeds, nominals))
                and node.publisher.get_subscription_count() == 1 and node.status_publisher.get_subscription_count() == 1)
            now = node.get_clock().now().nanoseconds
            first_scan, first_speed, first_nominal = scan(now), velocity(now), make_nominal(now)
            send(scans, first_scan, "scan")
            send(speeds, first_speed, "velocity")
            send(nominals, first_nominal, "nominal")
            wait_for(lambda: len(outputs) == 1)
            self.assertEqual(outputs[-1].header, first_nominal.header)
            self.assertEqual(outputs[-1].scan_header, first_scan.header)
            self.assertEqual(outputs[-1].velocity_header, first_speed.header)
            self.assertEqual(outputs[-1].velocity_child_frame_id, FRAME)
            send(nominals, first_nominal, "nominal")
            self.assertEqual(statuses[-1]["reason"], "nominal_stamp_rejected")
            self.assertEqual(len(outputs), 1)
            # Input silence publishes neither cached commands nor timer status.
            count = len(statuses)
            deadline = time.monotonic()+.55
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.01)
            self.assertEqual(len(outputs), 1)
            self.assertEqual(len(statuses), count)
            now = node.get_clock().now().nanoseconds
            send(nominals, make_nominal(now), "nominal")
            self.assertEqual(statuses[-1]["reason"], "scan_unavailable_or_expired")
            now = node.get_clock().now().nanoseconds
            send(scans, scan(now), "scan")
            send(speeds, velocity(now), "velocity")
            send(nominals, make_nominal(now), "nominal")
            wait_for(lambda: len(outputs) == 2)
            now = node.get_clock().now().nanoseconds
            send(scans, scan(now, .1), "scan")
            send(speeds, velocity(now, 4.), "velocity")
            send(nominals, make_nominal(now, .3, .12), "nominal")
            self.assertEqual(statuses[-1]["reason"], "infeasible_box_bound")
            self.assertEqual(len(outputs), 2)
            self.assertEqual(len(list(node.timers)), 0)
        finally:
            executor.shutdown()
            if node is not None: node.destroy_node()
            if observer is not None: observer.destroy_node()
            rclpy.shutdown(context=context)

"""DDS/tf2 software fixture: declaration, observed endpoints, geometry and silence."""
import json
from pathlib import Path
import tempfile
import time
import unittest

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from ebva_localization.state_tf_audit import StateTfAudit
from tf_test_helpers import profile, transform


class TfAuditDdsTests(unittest.TestCase):
    def test_local_tf_and_live_endpoint_policy(self):
        self.run_fixture(False)

    def test_global_and_local_tf_are_composed_without_frame_renaming(self):
        self.run_fixture(True)

    def run_fixture(self, global_frame):
        namespace = "/global_tf_contract_fixture" if global_frame else "/local_tf_contract_fixture"
        context = Context()
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        nodes = []
        with tempfile.TemporaryDirectory(prefix="ebva-state-tf-") as temporary:
            path = Path(temporary) / "declaration.json"
            declaration = profile(global_frame, namespace)
            if global_frame:
                # Humble Python has no per-message GID. One broadcaster owns
                # both dynamic edges in this qualified inference fixture.
                declaration["edges"][0]["publisher"] = namespace + "/local_estimator"
            path.write_text(json.dumps(declaration))
            try:
                extrinsics = Node("extrinsics", namespace=namespace, context=context)
                nodes.append(extrinsics)
                qos_static = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
                static_pub = extrinsics.create_publisher(TFMessage, namespace+"/tf_static", qos_static)
                # Published before the auditor and its state providers are ready.
                static_pub.publish(TFMessage(transforms=[
                    transform("fixture_rear_axle_flu", "fixture_laser", 0, .25)]))
                auditor = StateTfAudit(context=context, namespace=namespace, parameter_overrides=[
                    Parameter("profile_path", value=str(path)),
                    Parameter("tf_topic", value=namespace+"/tf"),
                    Parameter("tf_static_topic", value=namespace+"/tf_static"),
                    Parameter("audit_period_sec", value=.025)])
                nodes.append(auditor)
                observer = Node("observer", namespace=namespace, context=context,
                    cli_args=["--ros-args", "-r", "/tf:="+namespace+"/tf",
                              "-r", "/tf_static:="+namespace+"/tf_static"],
                    use_global_arguments=False)
                nodes.append(observer)
                statuses = []
                observer.create_subscription(String, "state_tf_audit/status",
                    lambda msg: statuses.append(json.loads(msg.data)), 100)
                buffer = Buffer()
                listener = TransformListener(buffer, observer, spin_thread=False)
                for node in nodes:
                    executor.add_node(node)

                def wait_for(predicate, timeout=5.):
                    deadline = time.monotonic() + timeout
                    while not predicate() and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.01)
                    self.assertTrue(predicate(), statuses[-1] if statuses else "No observed status")

                def spin_for(seconds):
                    deadline = time.monotonic()+seconds
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.01)

                wait_for(lambda: statuses and statuses[-1]["pending_discovery_samples"] >= 1)
                self.assertFalse(statuses[-1]["discovery_ready"])
                self.assertEqual(statuses[-1]["rejections"], {})

                local = Node("local_estimator", namespace=namespace, context=context)
                nodes.append(local)
                executor.add_node(local)
                local_pub = local.create_publisher(TFMessage, namespace+"/tf", 100)
                wait_for(lambda: statuses and statuses[-1]["discovery_ready"]
                         and statuses[-1]["pending_discovery_samples"] == 0)
                wait_for(lambda: local_pub.get_subscription_count() >= 2)

                def publish_dynamic():
                    stamp = local.get_clock().now().nanoseconds
                    transforms = [transform(stamp_ns=stamp, x=1.)]
                    if global_frame:
                        transforms.append(transform("fixture_map", "fixture_odom", stamp, 10.))
                    local_pub.publish(TFMessage(transforms=transforms))

                publish_dynamic()
                wait_for(lambda: statuses[-1]["observations_valid"])
                target = "fixture_map" if global_frame else "fixture_odom"
                wait_for(lambda: buffer.can_transform(target, "fixture_laser", Time()))
                result = buffer.lookup_transform(target, "fixture_laser", Time())
                self.assertAlmostEqual(result.transform.translation.x, 11.25 if global_frame else 1.25)
                self.assertEqual(result.header.frame_id, target)
                self.assertEqual(result.child_frame_id, "fixture_laser")
                self.assertFalse(statuses[-1]["authority_authentication"])
                self.assertFalse(statuses[-1]["per_message_gid_available"])
                self.assertFalse(statuses[-1]["sample_authority_verified"])
                self.assertEqual(statuses[-1]["identity_per_sample"], "unavailable")
                self.assertEqual(statuses[-1]["attribution"], "unique_endpoint_inference")
                self.assertEqual(statuses[-1]["state_validation_scope"], "declarations_only")

                # tf2 still caches the transform; monotonic audit validity expires.
                spin_for(.30)
                wait_for(lambda: any(i["reason"] == "transform_expired" for i in statuses[-1]["issues"]))
                self.assertTrue(buffer.can_transform(target, "fixture_laser", Time()))
                self.assertFalse(statuses[-1]["observations_valid"])
                publish_dynamic()
                wait_for(lambda: statuses[-1]["observations_valid"])

                # Two publishing endpoints with the same FQN are ambiguous too.
                extra = local.create_publisher(TFMessage, namespace+"/tf", 100)
                wait_for(lambda: any(i["reason"] == "authority_ambiguous" for i in statuses[-1]["issues"]))
                self.assertTrue(any(i["reason"] == "ambiguous_stream" for i in statuses[-1]["issues"]))
                local.destroy_publisher(extra)
                wait_for(lambda: statuses[-1]["discovery_ready"])

                rogue = Node("another_authority", namespace=namespace, context=context)
                nodes.append(rogue)
                executor.add_node(rogue)
                rogue_pub = rogue.create_publisher(TFMessage, namespace+"/tf", 100)
                wait_for(lambda: any(i["reason"] == "ambiguous_stream" for i in statuses[-1]["issues"]))
                self.assertFalse(statuses[-1]["observations_valid"])
                rogue.destroy_publisher(rogue_pub)
                wait_for(lambda: statuses[-1]["discovery_ready"])

                # A retained static transform is not qualified after its endpoint goes away.
                extrinsics.destroy_publisher(static_pub)
                wait_for(lambda: any(i["reason"] == "authority_missing" for i in statuses[-1]["issues"]))
                self.assertFalse(statuses[-1]["observations_valid"])
                self.assertTrue(buffer.can_transform(target, "fixture_laser", Time()))
                listener.unregister()
            finally:
                executor.shutdown()
                for node in reversed(nodes):
                    node.destroy_node()
                rclpy.try_shutdown(context=context)

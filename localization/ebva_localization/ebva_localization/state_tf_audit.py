"""Opt-in observer of TF declarations, DDS publishing endpoints and source age."""
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .tf_contract import TfAudit


def gid_key(gid):
    return bytes(gid).hex()


def publisher_fqn(endpoint):
    return endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name


class StateTfAudit(Node):
    """Publish status only; never relay, change or remove a transform."""
    def __init__(self, **kwargs):
        super().__init__("state_tf_audit", **kwargs)
        path = self.declare_parameter("profile_path", "").value
        if not path:
            raise ValueError("profile_path must select an explicit declaration")
        self.audit = TfAudit(json.loads(Path(path).read_text(encoding="utf-8")))
        self.topics = {
            "dynamic": self.declare_parameter("tf_topic", "/tf").value,
            "static": self.declare_parameter("tf_static_topic", "/tf_static").value,
        }
        if not all(isinstance(topic, str) and topic for topic in self.topics.values()) or len(set(self.topics.values())) != 2:
            raise ValueError("Distinct nonempty dynamic/static TF topics required")
        period = self.declare_parameter("audit_period_sec", 0.05).value
        if not isinstance(period, (int, float)) or not 0 < period <= 1:
            raise ValueError("audit_period_sec must be positive and at most one second")
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.pending = deque(maxlen=64)
        self.tf_subscriptions = []
        for kind, topic in self.topics.items():
            qos = QoSProfile(
                depth=100 if kind == "dynamic" else 100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE if kind == "dynamic"
                           else DurabilityPolicy.TRANSIENT_LOCAL)
            def bind_kind(selected_kind):
                def receive(message):
                    self.on_tf(message, selected_kind)
                return receive
            self.tf_subscriptions.append(self.create_subscription(
                TFMessage, topic, bind_kind(kind), qos))
        self.audit_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(period, self.tick, clock=self.audit_clock)

    def refresh_graph(self):
        self.audit.update_graph({
            kind: self._endpoints(topic) for kind, topic in self.topics.items()
        })

    def _endpoints(self, topic):
        result = {}
        for endpoint in self.get_publishers_info_by_topic(topic):
            result.setdefault(publisher_fqn(endpoint), set()).add(gid_key(endpoint.endpoint_gid))
        return result

    def stream_gid(self, kind):
        gids = [gid for values in self.audit.endpoints[kind].values() for gid in values]
        return gids[0] if len(gids) == 1 else None

    def stream_issues(self):
        issues = []
        selected_kinds = {edge["kind"] for edge in self.audit.edges.values()}
        for kind in selected_kinds:
            count = sum(len(values) for values in self.audit.endpoints[kind].values())
            if count > 1:
                issues.append({"kind": kind, "reason": "ambiguous_stream",
                               "publishing_endpoints": count})
        return issues

    def on_tf(self, message, kind):
        # Humble rclpy callbacks provide no per-message GID. Infer only from
        # a unique endpoint TOTAL on this topic; never pick one expected FQN
        # while ignoring another publisher on the same stream.
        self.refresh_graph()
        now = self.get_clock().now().nanoseconds
        mono = time.monotonic_ns()
        count = sum(len(values) for values in self.audit.endpoints[kind].values())
        if count > 1:
            self.audit.reject("ambiguous_stream")
            self.publish_status()
            return
        gid = self.stream_gid(kind)
        for transform in message.transforms:
            if gid is None:
                reason = "discovery_not_ready"
            else:
                reason = self.audit.observe(transform, kind, gid, now, mono)
            if reason == "discovery_not_ready":
                if len(self.pending) == self.pending.maxlen:
                    self.audit.reject("discovery_queue_overflow")
                self.pending.append((transform, kind, gid, now, mono))
        self.publish_status()

    def tick(self):
        self.refresh_graph()
        if not self.audit._graph_issues() and not self.stream_issues():
            pending, self.pending = self.pending, deque(maxlen=64)
            for transform, kind, gid, ros_received, mono_received in pending:
                if gid is None or self.stream_gid(kind) != gid:
                    self.audit.reject("pending_endpoint_unknown_or_changed")
                    continue
                self.audit.observe(transform, kind, gid,
                    self.get_clock().now().nanoseconds, time.monotonic_ns(),
                    received_at=(ros_received, mono_received))
        self.publish_status()

    def publish_status(self):
        message = String()
        report = self.audit.report(self.get_clock().now().nanoseconds, time.monotonic_ns())
        stream_issues = self.stream_issues()
        report["issues"].extend(stream_issues)
        report["discovery_ready"] = report["discovery_ready"] and not stream_issues
        report["observations_valid"] = report["observations_valid"] and not stream_issues
        report["pending_discovery_samples"] = len(self.pending)
        report["per_message_gid_available"] = False
        report["identity_per_sample"] = "unavailable"
        report["sample_authority_verified"] = False
        report["attribution"] = "unique_endpoint_inference"
        report["observations_scope"] = "tf_content_time_and_unique_topic_endpoint_inference"
        message.data = json.dumps(report, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateTfAudit()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

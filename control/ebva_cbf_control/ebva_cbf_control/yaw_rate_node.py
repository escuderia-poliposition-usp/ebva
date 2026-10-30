"""Opt-in stamped yaw_rate CBF node; no drivers or actuator publishers."""
import json
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ebva_control_msgs.msg import AccelerationYawRateStamped, CbfYawRateCommand
from .yaw_rate_port import YawRatePort


class CbfYawRateNode(Node):
    def __init__(self, **kwargs):
        super().__init__("cbf_yaw_rate", **kwargs)
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.port = YawRatePort(
            parameter("control_frame", ""), parameter("velocity_parent_frame", ""),
            parameter("model_reference_point", ""), parameter("scan_geometry", ""),
            parameter("timeout_s", .2), parameter("future_tolerance_s", .02),
            parameter("max_skew_s", .08))
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        scan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(CbfYawRateCommand,
            parameter("command_topic", "control/autonomy/cbf_yaw_rate"), reliable)
        self.status_publisher = self.create_publisher(String, "~/status", reliable)
        self.scan_subscription = self.create_subscription(LaserScan,
            parameter("scan_topic", "perception/scan"), self.on_scan, scan_qos)
        self.velocity_subscription = self.create_subscription(Odometry,
            parameter("velocity_topic", "state/odometry"), self.on_velocity, reliable)
        self.nominal_subscription = self.create_subscription(AccelerationYawRateStamped,
            parameter("nominal_topic", "control/autonomy/nominal"), self.on_nominal, reliable)
        self.command_count = 0

    def _status(self, source, reason, kernel=None):
        status = String()
        status.data = json.dumps(dict(source=source, accepted=reason is None,
            reason=reason or "accepted", command_count=self.command_count,
            clock_reset_required=self.port.clock_reset_required,
            max_constraint_violation=(kernel.max_constraint_violation if kernel else None)),
            sort_keys=True, allow_nan=False)
        self.status_publisher.publish(status)

    def on_scan(self, message):
        reason = self.port.observe_scan(message, self.get_clock().now().nanoseconds, time.monotonic())
        self._status("scan", reason)

    def on_velocity(self, message):
        reason = self.port.observe_velocity(message, self.get_clock().now().nanoseconds, time.monotonic())
        self._status("velocity", reason)

    def on_nominal(self, message):
        output, decision = self.port.nominal(message, self.get_clock().now().nanoseconds, time.monotonic())
        if output is not None:
            self.publisher.publish(output)
            self.command_count += 1
        self._status("nominal", None if output is not None else decision.status, decision.kernel)


def main(args=None):
    node, stopping, previous = None, False, {}
    def interrupt(signum, frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            raise KeyboardInterrupt
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt)
        try:
            rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
            node = CbfYawRateNode()
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
        except KeyboardInterrupt:
            pass
        finally:
            stopping = True
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown()
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)

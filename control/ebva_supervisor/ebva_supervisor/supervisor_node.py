"""ROS classifier input and normalized effort output; no actuator transport."""
import copy
from dataclasses import fields, replace
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ebva_control_msgs.msg import TrackClassificationStamped, DriveEffort

from .core import Classification, Settings, Supervisor


def qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


class SupervisorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("supervisor", **kwargs)
        self.input_topic = self.declare_parameter("input_topic", "classification/input").value
        self.output_topic = self.declare_parameter("output_topic", "control/autonomy/effort").value
        if not all(isinstance(v, str) and v for v in (self.input_topic, self.output_topic)):
            raise ValueError("Explicit classification and effort topics required")
        defaults = Settings()
        settings = Settings(**{field.name: self.declare_parameter(
            field.name, getattr(defaults, field.name)).value for field in fields(Settings)})
        self.core = Supervisor(settings)
        self.header = None
        self.pub = self.create_publisher(DriveEffort, self.output_topic, qos())
        self.status = self.create_publisher(DiagnosticArray, "control/supervisor_status", qos())
        self.create_subscription(TrackClassificationStamped, self.input_topic, self.receive, qos())
        self.add_on_set_parameters_callback(self.configure)
        self.timer = self.create_timer(1 / settings.control_rate_hz, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def configure(self, parameters):
        allowed = {field.name for field in fields(Settings)}
        values = {p.name: p.value for p in parameters if p.name in allowed}
        if any(p.name in ("input_topic", "output_topic") and
               p.value != getattr(self, p.name) for p in parameters):
            return SetParametersResult(successful=False, reason="Topic changes require restart")
        try:
            settings = replace(self.core.settings, **values)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.core.configure(settings)
        return SetParametersResult(successful=True)

    def receive(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        value = Classification(stamp, message.header.frame_id, message.valid,
            message.track_type, message.position, message.track_probability,
            message.track_entropy, message.position_probability,
            message.position_entropy, message.curvature_feature)
        if self.core.observe(value, self.get_clock().now().nanoseconds, time.monotonic()):
            self.header = copy.deepcopy(message.header)
        else:
            self.header = None

    def tick(self):
        result = self.core.command(self.get_clock().now().nanoseconds, time.monotonic())
        if result is not None:
            message = DriveEffort()
            message.header = copy.deepcopy(self.header)
            message.contract = result.contract
            message.longitudinal, message.steering = result.longitudinal, result.steering
            self.pub.publish(message)
        state = DiagnosticStatus()
        state.name = "ebva_supervisor"
        state.level = DiagnosticStatus.OK if result is not None else DiagnosticStatus.WARN
        state.message = self.core.reason
        state.values = [KeyValue(key="method", value=self.core.settings.method),
                        KeyValue(key="selected_method", value=self.core.selected_method or ""),
                        KeyValue(key="physical_qualification", value="false")]
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        report.status = [state]
        self.status.publish(report)


def main(args=None):
    stopping = False
    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        for signum in previous:
            signal.signal(signum, request_stop)
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = SupervisorNode()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()

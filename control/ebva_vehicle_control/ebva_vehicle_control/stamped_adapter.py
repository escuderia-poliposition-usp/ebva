"""Shared ROS stamp/frame/fixture policy for evidenced command adapters."""
import copy
import math
from dataclasses import fields
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ebva_control_msgs.msg import ActuatorCommand
from .nodes import parameter, qos, source_age, valid_age, spin


class StampedFixtureAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__(self.node_name, **kwargs)
        try:
            self.purpose = parameter(self, "mapping_purpose", "")
            self.calibration_id = parameter(self, "calibration_id", "")
            self.expected_frame = parameter(self, "expected_frame", "")
            self.timeout = parameter(self, "source_timeout_s", .2)
            if self.purpose != "software_fixture" or not self.calibration_id or not self.expected_frame:
                raise ValueError("Explicit software_fixture purpose, calibration_id and expected_frame required")
            if not math.isfinite(self.timeout) or self.timeout <= 0:
                raise ValueError("Invalid source timeout")
            values = {f.name:self.declare_parameter(f.name, Parameter.Type.DOUBLE).value
                      for f in fields(self.mapping_class)}
            self.mapping = self.mapping_class(**values)
            self.last_stamp_ns = None
            self.pub = self.create_publisher(ActuatorCommand, "control/autonomy/actuators", qos())
            self.status = self.create_publisher(DiagnosticArray, self.status_topic, qos())
            self.create_subscription(self.input_type, self.input_topic, self.receive, qos())
            if self.diagnostic_heartbeat:
                self.create_timer(.1, self.report_clock, clock=Clock(clock_type=ClockType.STEADY_TIME))
        except Exception:
            self.destroy_node()
            raise

    def receive(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        reason = "mapped"
        if not self.accept_contract(message):
            reason = "contract_mismatch"
        elif message.header.frame_id != self.expected_frame:
            reason = "frame_mismatch"
        elif stamp <= 0 or not valid_age(source_age(self, message.header), self.timeout):
            reason = "stale_or_future"
        elif self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
            reason = "duplicate_or_reordered_stamp"
        elif not self.sources_valid(message):
            reason = "secondary_input_invalid_or_expired"
        else:
            try:
                command = self.mapping.convert(*self.values(message))
            except ValueError as error:
                reason = self.conversion_error_reason(error)
            else:
                selected = ActuatorCommand()
                selected.header = copy.deepcopy(self.output_header(message))  # Never renew source age.
                selected.erpm, selected.servo = command.erpm, command.servo
                self.pub.publish(selected)
                self.last_stamp_ns = stamp
        report = DiagnosticArray()
        report.header = copy.deepcopy(message.header)
        state = DiagnosticStatus()
        state.name, state.hardware_id, state.message = "ebva_"+self.node_name, self.calibration_id, reason
        state.level = DiagnosticStatus.OK if reason == "mapped" else DiagnosticStatus.WARN
        state.values = [KeyValue(key="mapping_purpose", value=self.purpose),
                        KeyValue(key="expected_frame", value=self.expected_frame)]
        report.status = [state]
        self.status.publish(report)

    def report_clock(self):
        # Diagnostic heartbeat only; never renew a command lease.
        report=DiagnosticArray();report.header.stamp=self.get_clock().now().to_msg()
        self.status.publish(report)

    diagnostic_heartbeat = False

    def accept_contract(self, message):
        return True

    def conversion_error_reason(self, error):
        return "nonfinite_command"

    def sources_valid(self, message):
        return True

    def output_header(self, message):
        return message.header

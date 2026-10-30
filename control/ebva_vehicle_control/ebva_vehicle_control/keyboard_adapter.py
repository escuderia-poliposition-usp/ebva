"""Stamped key_teleop input through RC authority; never publishes final commands."""
import copy
import math
import time
from dataclasses import fields
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import SetBool
from ebva_control_msgs.msg import ActuatorCommand, SelectedControl

from .core import Mode
from .keyboard_mapping import KeyboardSettings, load_vehicle
from .nodes import parameter, qos, source_age, valid_age, unique_publisher, spin


class KeyboardAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__("keyboard_adapter", **kwargs)
        self.settings = KeyboardSettings(**{
            field.name: parameter(self, field.name, field.default) for field in fields(KeyboardSettings)})
        default = str(Path(get_package_share_directory("ebva_vehicle_control")) /
                      "config/vehicle/manual_core.yaml")
        self.vehicle, self.policy = load_vehicle(parameter(self, "vehicle_params_file", default))
        self.pub = self.create_publisher(ActuatorCommand, "control/autonomy/actuators", qos())
        self.status = self.create_publisher(DiagnosticArray, "control/keyboard_status", qos())
        self.create_subscription(TwistStamped, self.settings.input_topic, self.receive, qos())
        self.create_subscription(SelectedControl, "control/selected", self.receive_selection, qos())
        self.create_subscription(DiagnosticArray, "control/backend_status", self.receive_backend, qos())
        self.client = self.create_client(SetBool, "control/set_autonomy_enabled")
        self.enable_pending = self.settings.enable_external_on_startup
        self.future = None
        self.last_input = None
        self.last_stamp = None
        self.selection = None
        self.backend_enabled = False
        self.backend_seen = None
        self.reason = "waiting_for_keyboard"
        self.counter = 0
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_selection(self, message):
        self.selection = copy.deepcopy(message)

    def receive_backend(self, message):
        for status in message.status:
            if status.name == "ebva_vesc_backend":
                values = {item.key: item.value for item in status.values}
                self.backend_enabled = values.get("enabled") == "True"
                self.backend_seen = time.monotonic()

    def input_unique(self):
        return unique_publisher(self, self.settings.input_topic, self.settings.expected_publisher)

    def receive(self, message):
        self.reason = "invalid_keyboard_source_or_stamp"
        self.last_input = None
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if not (self.input_unique() and message.header.frame_id == self.settings.expected_frame
                and stamp > 0 and valid_age(source_age(self, message.header), self.settings.source_timeout_s)
                and (self.last_stamp is None or stamp > self.last_stamp)):
            return
        twist = message.twist
        unused = (twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y)
        if any(not math.isfinite(value) or value != 0.0 for value in unused):
            self.reason = "unsupported_keyboard_axes"
            return
        try:
            value = self.settings.convert(twist.linear.x, twist.angular.z, self.vehicle)
        except ValueError:
            self.reason = "invalid_keyboard_axes"
            return
        out = ActuatorCommand()
        out.header = copy.deepcopy(message.header)  # Preserve the original lease.
        out.erpm, out.servo = value.erpm, value.servo
        self.pub.publish(out)
        self.last_input = copy.deepcopy(message)
        self.last_stamp = stamp
        self.reason = "keyboard_command"

    def ready_to_enable(self):
        key, selected = self.last_input, self.selection
        if key is None or selected is None or not self.input_unique():
            return False
        if not (valid_age(source_age(self, key.header), self.settings.source_timeout_s)
                and key.twist.linear.x == 0.0 and key.twist.angular.z == 0.0):
            return False
        if not (self.backend_enabled and self.backend_seen is not None
                and 0 <= time.monotonic() - self.backend_seen < self.settings.source_timeout_s):
            return False
        rc = selected.rc
        return (selected.mode == Mode.MANUAL and rc.valid and rc.rf_state in (0, 1)
                and valid_age(source_age(self, selected.header), self.policy.rc_timeout)
                and valid_age(source_age(self, rc.header), self.policy.rc_timeout)
                and all(math.isfinite(v) and 900 <= v <= 2100
                        for v in (rc.throttle_us, rc.steering_us))
                and max(abs(rc.throttle_us - self.policy.center),
                        abs(rc.steering_us - self.policy.center)) <= self.policy.release)

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                response = self.future.result()
                if response is not None and response.success:
                    self.enable_pending = False
                    self.get_logger().info("Keyboard selected; physical RC retains priority")
            finally:
                self.future = None
        if (self.enable_pending and self.future is None and self.ready_to_enable()
                and self.client.service_is_ready()):
            self.future = self.client.call_async(SetBool.Request(data=True))
        self.counter += 1
        if self.counter % 5 == 0:
            reason = self.reason
            if self.last_input is not None and not valid_age(
                    source_age(self, self.last_input.header), self.settings.source_timeout_s):
                reason = "keyboard_expired"
            elif self.enable_pending:
                reason = "waiting_for_neutral_keyboard_rc_and_enabled_backend"
            status = DiagnosticStatus()
            status.name = "ebva_keyboard_adapter"
            status.level, status.message = DiagnosticStatus.OK, reason
            status.values = [KeyValue(key="startup_enable_pending", value=str(self.enable_pending))]
            report = DiagnosticArray()
            report.header.stamp = self.get_clock().now().to_msg()
            report.status = [status]
            self.status.publish(report)

    def destroy_node(self):
        self.enable_pending = False
        if self.future is not None:
            self.future.cancel()
        return super().destroy_node()


def main(args=None):
    spin(KeyboardAdapter, args)

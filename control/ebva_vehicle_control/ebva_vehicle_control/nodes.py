"""ROS I/O for acquisition, authority and bounded VESC backend.

Import is safe: only RcAcquisition construction opens a serial device.
"""
import copy
import math
import signal
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from ebva_control_msgs.msg import RcInput, ActuatorCommand, SelectedControl

from .core import Authority, AuthorityPolicy, RcSample, Actuation, Mode, Selection, Backend, Mapping
from .framing import RcFramer


def qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def parameter(node, name, default):
    return node.declare_parameter(name, default).value


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def source_age(node, header):
    return node.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(header.stamp)


def valid_age(age, timeout):
    return math.isfinite(age) and -0.02 <= age < timeout


def policy(node):
    return AuthorityPolicy(
        rc_timeout=parameter(node, "rc_timeout_s", 0.2),
        autonomy_timeout=parameter(node, "autonomy_timeout_s", 0.2),
        center=parameter(node, "rc_center_us", 1500.0),
        enter=parameter(node, "override_enter_us", 100.0),
        release=parameter(node, "override_release_us", 80.0))


def sample(message, valid=True):
    return RcSample(message.throttle_us, message.steering_us,
                    bool(message.valid and valid), message.rf_state, message.reason)


def unique_publisher(node, topic, expected=None):
    peers = node.get_publishers_info_by_topic(topic)
    return len(peers) == 1 and (expected is None or peers[0].node_name == expected)


class RcAcquisition(Node):
    """Own the serial port, publish raw frames/invalidity, never actuator commands."""
    def __init__(self, **kwargs):
        super().__init__("rc_acquisition", **kwargs)
        import serial
        self.timeout = parameter(self, "rc_timeout_s", 0.2)
        self.port_name = parameter(self, "serial_port", "/dev/arduino_rc")
        baud = parameter(self, "baud_rate", 115200)
        self.framer = RcFramer()
        self.pub = self.create_publisher(RcInput, "rc/input", qos())
        self.sequence = 0
        self.last_event = time.monotonic()
        # Kernel advisory exclusivity, plus host ownership check before the bench run.
        self.port = serial.Serial(self.port_name, baud, timeout=0, exclusive=True)
        self.port.reset_input_buffer()
        self.timer = self.create_timer(.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_frame(self, valid, throttle, steering, reason):
        message = RcInput()
        message.header.stamp = self.get_clock().now().to_msg()
        self.sequence += 1
        message.sequence = self.sequence
        message.valid, message.throttle_us, message.steering_us = valid, throttle, steering
        message.rf_state, message.reason = RcInput.RF_UNKNOWN, reason
        self.pub.publish(message)

    def tick(self):
        now = time.monotonic()
        try:
            waiting = self.port.in_waiting if self.port else 0
            if waiting > 4096:
                self.port.reset_input_buffer()
                self.framer = RcFramer()
                self.last_event = now
                self.publish_frame(False, 0.0, 0.0, "serial_backlog_discarded")
            elif waiting:
                frames = self.framer.feed(self.port.read(waiting))
                if frames:
                    # Only latest complete event: old queued RC cannot replay motions.
                    self.publish_frame(*frames[-1])
                    self.last_event = now
        except OSError:
            if self.port:
                self.port.close()
                self.port = None
            self.publish_frame(False, 0.0, 0.0, "serial_error")
            self.last_event = now
        if now-self.last_event >= self.timeout:
            self.publish_frame(False, 0.0, 0.0, "serial_timeout")
            self.last_event = now

    def destroy_node(self):
        if self.port:
            self.port.close()
            self.port = None
        return super().destroy_node()


class CommandAuthority(Node):
    def __init__(self, **kwargs):
        super().__init__("command_authority", **kwargs)
        self.core = Authority(policy(self))
        self.rc_publisher_name = parameter(self, "rc_publisher_name", "rc_acquisition")
        if not self.rc_publisher_name or not self.rc_publisher_name.replace("_", "a").isalnum():
            raise ValueError("Expected explicit RC publisher node name")
        self.graph_check = parameter(self, "enforce_graph", True)
        self.rc_message, self.auto_message = RcInput(), ActuatorCommand()
        self.sequence = 0
        self.pub = self.create_publisher(SelectedControl, "control/selected", qos())
        self.status_pub = self.create_publisher(DiagnosticArray, "control/status", qos())
        self.create_subscription(RcInput, "rc/input", self.receive_rc, qos())
        self.create_subscription(ActuatorCommand, "control/autonomy/actuators", self.receive_auto, qos())
        self.create_service(SetBool, "control/set_autonomy_enabled", self.enable_autonomy)
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_rc(self, message):
        age = source_age(self, message.header)
        self.rc_message = copy.deepcopy(message)
        self.core.receive_rc(sample(message, valid_age(age, self.core.policy.rc_timeout)),
                             time.monotonic()-max(0.0, age) if math.isfinite(age) else time.monotonic())

    def receive_auto(self, message):
        age = source_age(self, message.header)
        self.auto_message = copy.deepcopy(message)
        command = Actuation(message.erpm, message.servo)
        if not valid_age(age, self.core.policy.autonomy_timeout):
            command = Actuation(float("nan"), 0.5)
        self.core.receive_autonomy(command, time.monotonic()-max(0.0, age) if math.isfinite(age) else time.monotonic())

    def enable_autonomy(self, request, response):
        self.core.autonomy_enabled = request.data
        response.success = True
        response.message = "Autonomy enabled; valid RC/command still required" if request.data else "Autonomy disabled; manual RC remains available"
        return response

    def tick(self):
        if self.graph_check:
            if not unique_publisher(self, "rc/input", self.rc_publisher_name):
                self.core.rc = RcSample(reason="rc_publisher_conflict")
            if len(self.get_publishers_info_by_topic("control/autonomy/actuators")) > 1:
                self.core.auto_valid = False
        selected = self.core.decide(time.monotonic())
        message = SelectedControl()
        message.header.stamp = self.get_clock().now().to_msg()
        self.sequence += 1
        message.sequence, message.mode, message.reason = self.sequence, int(selected.mode), selected.reason
        message.rc, message.autonomy = copy.deepcopy(self.rc_message), copy.deepcopy(self.auto_message)
        # Propagate invalidation, not only the last incoming raw valid flag.
        message.rc.valid = selected.rc.valid
        self.pub.publish(message)
        if self.sequence % 5 == 0:
            status = DiagnosticStatus()
            status.name, status.hardware_id = "ebva_command_authority", "rc_serial_rf_unknown"
            status.level, status.message = DiagnosticStatus.OK, selected.reason
            status.values = [KeyValue(key=key, value=str(value)) for key, value in {
                "mode": selected.mode.name, "rc_age_s": selected.rc_age,
                "autonomy_age_s": selected.autonomy_age, "autonomy_enabled": self.core.autonomy_enabled,
                "rc_valid": selected.rc.valid, "rf_state": selected.rc.rf_state}.items()]
            report = DiagnosticArray()
            report.header = message.header
            report.status = [status]
            self.status_pub.publish(report)


class VescBackend(Node):
    def __init__(self, **kwargs):
        super().__init__("vesc_backend", **kwargs)
        selected_policy = policy(self)
        mapping = Mapping(**{name: parameter(self, name, default)
                             for name, default in Mapping().__dict__.items()})
        self.core = Backend(mapping, parameter(self, "backend_timeout_s", 0.2), selected_policy)
        self.graph_check = parameter(self, "enforce_graph", True)
        self.auto_enable_on_startup = parameter(self, "auto_enable_on_startup", False)
        self.startup_enable_pending = self.auto_enable_on_startup
        self.motor = self.create_publisher(Float64, "commands/motor/speed", qos())
        self.servo = self.create_publisher(Float64, "commands/servo/position", qos())
        self.status_pub = self.create_publisher(DiagnosticArray, "control/backend_status", qos())
        self.create_subscription(SelectedControl, "control/selected", self.receive, qos())
        self.create_service(SetBool, "control/set_actuation_enabled", self.enable)
        self.counter = 0
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def graph_ok(self):
        return (not self.graph_check or (
            unique_publisher(self, "commands/motor/speed", "vesc_backend")
            and unique_publisher(self, "commands/servo/position", "vesc_backend")
            and unique_publisher(self, "control/selected", "command_authority")))

    def receive(self, message):
        age = source_age(self, message.header)
        if not valid_age(age, self.core.timeout):
            self.core.selection, self.core.received = None, None
            return
        rc_age, auto_age = source_age(self, message.rc.header), source_age(self, message.autonomy.header)
        rc_ok = valid_age(rc_age, self.core.policy.rc_timeout) and message.rc.rf_state in (0, 1)
        try:
            mode = Mode(message.mode)
        except ValueError:
            mode = Mode.STOP
        selection = Selection(mode, message.reason, sample(message.rc, rc_ok),
                              Actuation(message.autonomy.erpm, message.autonomy.servo),
                              max(0.0, rc_age-age), max(0.0, auto_age-age))
        self.core.receive(selection, time.monotonic()-max(0.0, age))

    def enable(self, request, response):
        # An explicit operator request takes precedence over startup automation.
        self.startup_enable_pending = False
        if request.data and not self.graph_ok():
            response.success, response.message = False, "Publisher ownership conflict/not discovered"
        else:
            response.success = self.core.enable(request.data, time.monotonic())
            response.message = "Enabled" if response.success and request.data else (
                "Disabled" if response.success else "Fresh neutral RC required before enabling")
        return response

    def tick(self):
        graph_ok = self.graph_ok()
        now = time.monotonic()
        if not graph_ok:
            self.core.enabled = False
        elif self.startup_enable_pending:
            selected = self.core.selection
            # Startup can release manual control only after fresh neutral RC.
            # Successful startup is consumed once; faults never auto-rearm it.
            if (selected is not None and selected.mode == Mode.MANUAL
                    and self.core.enable(True, now)):
                self.startup_enable_pending = False
                self.get_logger().info("Manual actuation enabled after fresh neutral RC")
        command, reason = self.core.output(now)
        if self.startup_enable_pending:
            reason = ("startup_waiting_for_neutral_rc" if graph_ok
                      else "startup_waiting_for_graph")
        self.motor.publish(Float64(data=command.erpm))
        self.servo.publish(Float64(data=command.servo))
        self.counter += 1
        if self.counter % 5 == 0:
            status = DiagnosticStatus()
            status.name, status.hardware_id = "ebva_vesc_backend", "vesc"
            status.level, status.message = DiagnosticStatus.OK, reason
            status.values = [KeyValue(key=key, value=str(value)) for key, value in {
                "enabled": self.core.enabled, "erpm": command.erpm, "servo": command.servo,
                "auto_enable_on_startup": self.auto_enable_on_startup,
                "startup_enable_pending": self.startup_enable_pending}.items()]
            report = DiagnosticArray()
            report.header.stamp = self.get_clock().now().to_msg()
            report.status = [status]
            self.status_pub.publish(report)

    def stop(self):
        self.startup_enable_pending = False
        self.core.enabled = False
        if self.context.ok():
            self.motor.publish(Float64(data=0.0))


def spin(factory, args=None):
    """Process entry point: stop/destroy before shutting down the ROS context.

    Process signals only request a stop: raising from a Python handler during
    a native take_message call can cross the binding boundary as another error.
    Bounded waits service silent subscribers without interrupting native calls.
    Callback failures remain failures, including RuntimeError after a stop request.
    """
    node = None
    previous = {}
    stopping = False
    def interrupt(signum, frame):
        nonlocal stopping
        stopping = True
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.getsignal(signum)
            signal.signal(signum, interrupt)
        try:
            rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
            node = factory()
            while not stopping and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
        except KeyboardInterrupt:
            pass
        finally:
            stopping = True  # Repeated process signals must not interrupt cleanup.
            try:
                if node is not None:
                    try:
                        if isinstance(node, VescBackend):
                            node.stop()
                    finally:
                        node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def rc_main(args=None):
    spin(RcAcquisition, args)


def authority_main(args=None):
    spin(CommandAuthority, args)


def backend_main(args=None):
    spin(VescBackend, args)

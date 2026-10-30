"""Real DDS/service transitions with synthetic inputs; no serial/VESC devices."""
import time
from unittest.mock import patch, Mock

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from ebva_control_msgs.msg import RcInput, ActuatorCommand, SelectedControl
from ebva_vehicle_control.nodes import (
    RcAcquisition, CommandAuthority, VescBackend, qos, valid_age, source_age)


class Rig:
    def __init__(self, enforce_graph=False, rc_provider="rc_acquisition",
                 auto_enable_on_startup=False):
        self.context = Context()
        rclpy.init(context=self.context)
        params = [Parameter("enforce_graph", value=enforce_graph),
                  Parameter("rc_publisher_name",value=rc_provider)]
        self.authority = CommandAuthority(context=self.context, parameter_overrides=params)
        self.backend = VescBackend(context=self.context, parameter_overrides=[
            *params, Parameter("auto_enable_on_startup", value=auto_enable_on_startup)])
        self.source = Node(rc_provider, context=self.context)
        self.auto_source = Node("test_autonomy", context=self.context)
        self.extra_nodes = []
        self.executor = SingleThreadedExecutor(context=self.context)
        for node in (self.authority, self.backend, self.source, self.auto_source):
            self.executor.add_node(node)
        self.rc_pub = self.source.create_publisher(RcInput, "rc/input", qos())
        self.auto_pub = self.auto_source.create_publisher(ActuatorCommand, "control/autonomy/actuators", qos())
        self.motor, self.servo, self.selected = [], [], []
        self.source.create_subscription(Float64, "commands/motor/speed", lambda m: self.motor.append(m.data), qos())
        self.source.create_subscription(Float64, "commands/servo/position", lambda m: self.servo.append(m.data), qos())
        self.source.create_subscription(SelectedControl, "control/selected", lambda m: self.selected.append(m.mode), qos())
        self.sequence = 0

    def pump(self, duration=.35, throttle=1500., steering=1500., auto=True, valid=True,
             stale=False, stale_rc=False):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            message = RcInput()
            if not stale_rc:
                message.header.stamp = self.source.get_clock().now().to_msg()
            self.sequence += 1
            message.sequence, message.valid = self.sequence, valid
            message.throttle_us, message.steering_us = throttle, steering
            self.rc_pub.publish(message)
            if auto:
                command = ActuatorCommand()
                if not stale:
                    command.header.stamp = self.source.get_clock().now().to_msg()
                command.erpm, command.servo = 300., .53
                self.auto_pub.publish(command)
            # Drain callbacks across all nodes, not one callback per publication cycle.
            # Otherwise the fixture itself creates an ever-growing DDS backlog.
            for _ in range(16):
                self.executor.spin_once(timeout_sec=0)
            time.sleep(.01)

    def service(self, name, value):
        client = self.source.create_client(SetBool, name)
        try:
            assert client.wait_for_service(timeout_sec=2.)
            future = client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic()+2.
            while not future.done() and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=.01)
            assert future.done()
            return future.result()
        finally:
            self.source.destroy_client(client)

    def wait_graph(self, topic, count, **inputs):
        deadline = time.monotonic()+3.
        while len(self.backend.get_publishers_info_by_topic(topic)) != count:
            assert time.monotonic() < deadline, (topic, count)
            self.pump(.03, **inputs)
        self.pump(.12, **inputs)

    def arm(self):
        self.wait_graph("rc/input", 1)
        self.wait_graph("control/selected", 1)
        self.pump(.2)
        assert self.service("control/set_autonomy_enabled", True).success
        self.pump(.12)
        assert self.service("control/set_actuation_enabled", True).success
        self.pump(.12)
        assert self.motor[-1] == 300. and self.selected[-1] == SelectedControl.AUTONOMY

    def add_writer(self, message_type, topic):
        node = Node("conflicting_source", context=self.context)
        self.executor.add_node(node)
        self.extra_nodes.append(node)
        publisher = node.create_publisher(message_type, topic, qos())
        return node, publisher

    def remove_writer(self, node):
        self.executor.remove_node(node)
        node.destroy_node()
        self.extra_nodes.remove(node)

    def close(self):
        self.backend.stop()
        self.executor.shutdown()
        for node in (self.authority, self.backend, self.source, self.auto_source, *self.extra_nodes):
            if node is not None:
                node.destroy_node()
        self.context.shutdown()


def test_dds_manual_priority_neutral_resume_and_stale_source_stops():
    rig = Rig()
    try:
        rig.pump(.6)
        assert rig.motor[-1] == 0.
        assert rig.service("control/set_autonomy_enabled", True).success
        rig.pump(.1)
        assert rig.service("control/set_actuation_enabled", True).success
        rig.pump(.15)
        assert rig.selected[-1] == SelectedControl.AUTONOMY
        assert rig.motor[-1] == 300.

        rig.pump(.15, throttle=1505., steering=1700.)
        assert rig.selected[-1] == SelectedControl.MANUAL
        assert rig.motor[-1] == 0.  # no autonomous throttle leaks into steering override
        assert rig.servo[-1] == .6

        rig.pump(.15)
        assert rig.selected[-1] == SelectedControl.AUTONOMY
        assert rig.motor[-1] == 300.

        rig.pump(.35, auto=False)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.pump(.15, auto=True)
        assert rig.motor[-1] == 300.

        rig.pump(.15, valid=False)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.pump(.15)
        assert rig.motor[-1] == 300.

        rig.pump(.15, stale=True)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        assert rig.service("control/set_actuation_enabled", False).success
        rig.pump(.15)
        assert rig.motor[-1] == 0.
    finally:
        rig.close()


def test_ros_source_age_and_serial_construction_is_explicit():
    context = Context()
    rclpy.init(context=context)
    fake = Mock()
    fake.in_waiting = 0
    fake.read.return_value = b""
    with patch("serial.Serial", return_value=fake) as serial:
        node = RcAcquisition(context=context)
        try:
            assert serial.call_args.kwargs["exclusive"] is True
            assert serial.call_args.kwargs["timeout"] == 0
            assert valid_age(0.1, .2) and not valid_age(.3, .2)
            assert not valid_age(-1., .2)
            message = RcInput()
            assert source_age(node, message.header) > 1.
        finally:
            node.destroy_node()
            context.shutdown()
    fake.close.assert_called_once()


@pytest.mark.parametrize("topic,message_type", [
    ("commands/motor/speed", Float64), ("control/selected", SelectedControl)])
def test_dds_graph_competing_final_route_disarms_and_requires_rearming(topic, message_type):
    rig = Rig(enforce_graph=True)
    try:
        rig.arm()
        node, publisher = rig.add_writer(message_type, topic)
        rig.wait_graph(topic, 2)
        assert rig.motor[-1] == 0.
        assert not rig.backend.core.enabled
        assert not rig.service("control/set_actuation_enabled", True).success
        rig.remove_writer(node)
        rig.wait_graph(topic, 1)
        assert rig.motor[-1] == 0. and not rig.backend.core.enabled
        assert rig.service("control/set_actuation_enabled", True).success
        rig.pump(.15)
        assert rig.motor[-1] == 300.
    finally:
        rig.close()


def test_dds_graph_duplicate_autonomy_blocks_auto_and_preserves_manual_priority():
    rig = Rig(enforce_graph=True)
    try:
        rig.arm()
        node, publisher = rig.add_writer(ActuatorCommand, "control/autonomy/actuators")
        rig.wait_graph("control/autonomy/actuators", 2)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        assert rig.backend.core.enabled
        rig.pump(.2, throttle=1700., steering=1700.)
        assert rig.selected[-1] == SelectedControl.MANUAL
        assert rig.motor[-1] == -600. and rig.servo[-1] == .6
        rig.remove_writer(node)
        rig.wait_graph("control/autonomy/actuators", 1, throttle=1700., steering=1700.)
        rig.pump(.2)
        assert rig.selected[-1] == SelectedControl.AUTONOMY and rig.motor[-1] == 300.
    finally:
        rig.close()


def test_dds_graph_duplicate_rc_blocks_auto_and_manual_until_unique_fresh_rc():
    rig = Rig(enforce_graph=True)
    try:
        rig.arm()
        node, publisher = rig.add_writer(RcInput, "rc/input")
        rig.wait_graph("rc/input", 2)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.pump(.2, throttle=1700., steering=1700.)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.remove_writer(node)
        rig.wait_graph("rc/input", 1)
        rig.pump(.2)
        assert rig.selected[-1] == SelectedControl.AUTONOMY and rig.motor[-1] == 300.
    finally:
        rig.close()


def test_dds_graph_authority_disappearance_disarms_and_new_instance_starts_disabled():
    rig = Rig(enforce_graph=True)
    try:
        rig.arm()
        rig.executor.remove_node(rig.authority)
        rig.authority.destroy_node()
        rig.authority = None
        rig.wait_graph("control/selected", 0)
        assert rig.motor[-1] == 0. and not rig.backend.core.enabled
        assert not rig.service("control/set_actuation_enabled", True).success

        rig.authority = CommandAuthority(context=rig.context,
            parameter_overrides=[Parameter("enforce_graph", value=True)])
        rig.executor.add_node(rig.authority)
        rig.wait_graph("control/selected", 1)
        assert not rig.authority.core.autonomy_enabled
        # Valid neutral RC remains the manual source while autonomy is off.
        # A restarted authority still cannot rearm the disabled backend.
        assert rig.selected[-1] == SelectedControl.MANUAL and rig.motor[-1] == 0.
        assert not rig.backend.core.enabled
        assert rig.service("control/set_autonomy_enabled", True).success
        rig.pump(.15)
        assert rig.motor[-1] == 0.
        assert rig.service("control/set_actuation_enabled", True).success
        rig.pump(.15)
        assert rig.motor[-1] == 300.
    finally:
        rig.close()


def test_dds_explicit_replay_rc_provider_can_satisfy_ownership():
    rig=Rig(enforce_graph=True,rc_provider="rosbag2_player")
    try:
        rig.arm()
        rig.pump(.15,throttle=1700.,steering=1700.)
        assert rig.selected[-1] == SelectedControl.MANUAL and rig.motor[-1] == -600.
    finally:rig.close()


def test_dds_auto_start_waits_for_valid_fresh_neutral_then_passes_manual():
    rig = Rig(enforce_graph=True, auto_enable_on_startup=True)
    try:
        rig.pump(.6, throttle=1700.)
        assert not rig.backend.core.enabled and rig.backend.startup_enable_pending
        assert rig.motor[-1] == 0.
        rig.pump(.2, valid=False)
        assert not rig.backend.core.enabled
        rig.pump(.2, stale_rc=True)
        assert not rig.backend.core.enabled
        rig.pump(.2, steering=1700.)
        assert not rig.backend.core.enabled
        rig.pump(.3)
        assert rig.backend.core.enabled and not rig.backend.startup_enable_pending
        assert not rig.authority.core.autonomy_enabled
        assert rig.selected[-1] == SelectedControl.MANUAL and rig.motor[-1] == 0.
        rig.pump(.2, throttle=1700., steering=1700.)
        assert rig.motor[-1] == -600. and rig.servo[-1] == .6
        rig.pump(.25, valid=False)
        assert rig.motor[-1] == 0.
    finally:
        rig.close()


def test_dds_explicit_disable_cancels_pending_auto_start():
    rig = Rig(enforce_graph=True, auto_enable_on_startup=True)
    try:
        rig.pump(.6, throttle=1700.)
        assert rig.backend.startup_enable_pending and not rig.backend.core.enabled
        assert rig.service("control/set_actuation_enabled", False).success
        rig.pump(.2)
        assert not rig.backend.startup_enable_pending and not rig.backend.core.enabled
        rig.pump(.2, throttle=1700.)
        assert rig.motor[-1] == 0.
        rig.pump(.2)
        assert rig.service("control/set_actuation_enabled", True).success
        rig.pump(.2, throttle=1700.)
        assert rig.motor[-1] == -600.
        assert rig.service("control/set_actuation_enabled", False).success
        rig.pump(.2)
        assert not rig.backend.core.enabled and rig.motor[-1] == 0.
    finally:
        rig.close()


def test_dds_auto_start_waits_for_unique_graph_and_never_rearms_after_fault():
    rig = Rig(enforce_graph=True, auto_enable_on_startup=True)
    node, publisher = rig.add_writer(Float64, "commands/motor/speed")
    try:
        rig.wait_graph("commands/motor/speed", 2)
        assert not rig.backend.core.enabled and rig.backend.startup_enable_pending
        rig.remove_writer(node)
        rig.wait_graph("commands/motor/speed", 1)
        rig.pump(.3)
        assert rig.backend.core.enabled and not rig.backend.startup_enable_pending
        node, publisher = rig.add_writer(Float64, "commands/motor/speed")
        rig.wait_graph("commands/motor/speed", 2)
        assert not rig.backend.core.enabled
        rig.remove_writer(node)
        rig.wait_graph("commands/motor/speed", 1)
        rig.pump(.2)
        assert not rig.backend.core.enabled and not rig.backend.startup_enable_pending
        rig.pump(.2, throttle=1700.)
        assert rig.motor[-1] == 0.
    finally:
        rig.close()


class KeyboardRig(Rig):
    """Vendor-compatible input plus real authority/backend; no device nodes."""
    def __init__(self, tmp_path, auto_enable=True):
        import yaml
        from ebva_vehicle_control.core import Mapping
        from ebva_vehicle_control.keyboard_adapter import KeyboardAdapter
        from geometry_msgs.msg import TwistStamped
        super().__init__(enforce_graph=True, auto_enable_on_startup=auto_enable)
        self.auto_source.destroy_publisher(self.auto_pub)
        self.auto_pub = None
        mapping = {**Mapping().__dict__, "rc_timeout_s": .2, "autonomy_timeout_s": .2,
                   "rc_center_us": 1500., "override_enter_us": 100., "override_release_us": 80.}
        path = tmp_path / "vehicle.yaml"
        path.write_text(yaml.safe_dump({"vesc_backend": {"ros__parameters": mapping}}))
        self.keyboard = Node("key_teleop", context=self.context)
        self.adapter = KeyboardAdapter(context=self.context, parameter_overrides=[
            Parameter("vehicle_params_file", value=str(path))])
        for node in (self.keyboard, self.adapter):
            self.executor.add_node(node)
            self.extra_nodes.append(node)
        self.key_pub = self.keyboard.create_publisher(TwistStamped, "key_vel", qos())

    def pump(self, duration=.35, linear=0., angular=0., key=True, **inputs):
        from geometry_msgs.msg import TwistStamped
        inputs.pop("auto", None)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if key:
                message = TwistStamped()
                message.header.stamp = self.keyboard.get_clock().now().to_msg()
                message.header.frame_id = "key_teleop"
                message.twist.linear.x, message.twist.angular.z = linear, angular
                self.key_pub.publish(message)
            super().pump(.015, auto=False, **inputs)


def test_dds_keyboard_rc_priority_resume_and_input_loss(tmp_path):
    rig = KeyboardRig(tmp_path)
    try:
        # Held keyboard input must not select external control during startup.
        rig.pump(.8, linear=.8)
        assert rig.backend.core.enabled
        assert not rig.authority.core.autonomy_enabled and rig.adapter.enable_pending
        assert rig.motor[-1] == 0.
        rig.pump(.4)
        assert rig.authority.core.autonomy_enabled and not rig.adapter.enable_pending
        rig.pump(.2, linear=.8, angular=1.)
        assert rig.selected[-1] == SelectedControl.AUTONOMY
        assert rig.motor[-1] == 480. and rig.servo[-1] == .4
        rig.pump(.2, linear=.8, throttle=1700., steering=1700.)
        assert rig.selected[-1] == SelectedControl.MANUAL
        assert rig.motor[-1] == -600. and rig.servo[-1] == .6
        rig.pump(.2, linear=.8)
        assert rig.selected[-1] == SelectedControl.AUTONOMY and rig.motor[-1] == 480.
        rig.pump(.35, key=False)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.pump(.2, linear=.8)
        assert rig.motor[-1] == 480.
        rig.pump(.35, linear=.8, valid=False)
        assert rig.selected[-1] == SelectedControl.STOP and rig.motor[-1] == 0.
        rig.pump(.2)
        assert rig.service("control/set_autonomy_enabled", False).success
        rig.pump(.3, linear=.8)
        assert not rig.authority.core.autonomy_enabled and rig.motor[-1] == 0.
    finally:
        rig.close()


def test_dds_keyboard_does_not_enable_actuation_without_vehicle_policy(tmp_path):
    rig = KeyboardRig(tmp_path, auto_enable=False)
    try:
        rig.pump(.8)
        assert not rig.backend.core.enabled and not rig.authority.core.autonomy_enabled
        assert rig.service("control/set_actuation_enabled", True).success
        rig.pump(.4)
        assert rig.authority.core.autonomy_enabled
        rig.pump(.2, linear=.5)
        assert rig.motor[-1] == 300.
        assert rig.service("control/set_actuation_enabled", False).success
        rig.pump(.3)
        assert not rig.backend.core.enabled and rig.motor[-1] == 0.
    finally:
        rig.close()


def test_dds_keyboard_duplicate_source_stops_keyboard_but_rc_still_wins(tmp_path):
    from geometry_msgs.msg import TwistStamped
    rig = KeyboardRig(tmp_path)
    try:
        rig.pump(1.)
        rig.pump(.2, linear=.8)
        assert rig.motor[-1] == 480.
        node, publisher = rig.add_writer(TwistStamped, "key_vel")
        rig.wait_graph("key_vel", 2)
        rig.pump(.35, linear=.8)
        assert rig.motor[-1] == 0.
        rig.pump(.2, linear=.8, throttle=1700.)
        assert rig.motor[-1] == -600.
        rig.remove_writer(node)
        rig.wait_graph("key_vel", 1)
        rig.pump(.2, linear=.8)
        assert rig.motor[-1] == 480.
    finally:
        rig.close()

"""Finite real-DDS supervisor/authority exercise with synthetic inputs only."""
import argparse
import copy
import json
import os
from pathlib import Path
import time


class Rig:
    def __init__(self):
        import rclpy
        import yaml
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from ebva_control_msgs.msg import (
            TrackClassificationStamped, DriveEffort, ActuatorCommand, RcInput, SelectedControl)
        from std_msgs.msg import Float64
        from ebva_bringup.profiles import inspect_profile
        from ebva_supervisor.supervisor_node import SupervisorNode
        from ebva_vehicle_control.supervisor_effort_node import SupervisorEffortAdapter
        from ebva_vehicle_control.nodes import CommandAuthority, VescBackend, qos
        from .resources import profile_path

        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes = []
        self.source = None
        self.backend = None
        self.profile = inspect_profile(profile_path("supervisor_software"))
        if self.profile["launches"]:
            raise ValueError("Supervisor software fixture cannot include launches")
        expected = {"supervisor": SupervisorNode,
                    "supervisor_effort_adapter": SupervisorEffortAdapter,
                    "command_authority": CommandAuthority, "vesc_backend": VescBackend}
        if {item["name"] for item in self.profile["nodes"]} != set(expected):
            raise ValueError("Unexpected supervisor fixture components")
        self.namespace = self.profile["namespace"]
        try:
            for item in self.profile["nodes"]:
                name = item["name"]
                if item["executable"] not in {
                        "supervisor_node", "supervisor_effort_adapter", "command_authority", "vesc_backend"}:
                    raise ValueError("Only supervisor software components are allowed")
                params = {}
                for filename in item["parameters"]:
                    document = yaml.safe_load(Path(filename).read_text())
                    key = self.namespace.rstrip("/") + "/" + name
                    params.update(document[key]["ros__parameters"])
                node = expected[name](context=self.context, namespace=self.namespace,
                    parameter_overrides=[Parameter(key, value=value) for key, value in params.items()])
                self.nodes.append(node)
                self.executor.add_node(node)
                setattr(self, name if name != "vesc_backend" else "backend", node)
            self.source = Node("supervisor_fixture", namespace=self.namespace, context=self.context)
            self.nodes.append(self.source)
            self.executor.add_node(self.source)
            self.class_pub = self.source.create_publisher(
                TrackClassificationStamped, "classification/input", qos())
            self.rc_pub = self.source.create_publisher(RcInput, "rc/input", qos())
            self.records = {"effort": [], "actuators": [], "selection": [], "motor": [], "servo": []}
            for kind, message_type, topic in [
                ("effort", DriveEffort, "control/autonomy/effort"),
                ("actuators", ActuatorCommand, "control/autonomy/actuators"),
                ("selection", SelectedControl, "control/selected"),
                ("motor", Float64, "commands/motor/speed"),
                ("servo", Float64, "commands/servo/position")]:
                self.source.create_subscription(message_type, topic,
                    lambda msg, kind=kind: self.records[kind].append(
                        (time.monotonic(), copy.deepcopy(msg))), qos())
            self.sent_stamps = set()
            self.sequence = 0
            self.held_classification = None
        except BaseException:
            self.close()
            raise

    def pump(self, duration=.25, *, method_track="straight", position="middle",
             classifier="fresh", rc="fresh", throttle=1500., steering=1500.):
        from ebva_control_msgs.msg import TrackClassificationStamped, RcInput
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            stamp = self.source.get_clock().now().to_msg()
            if classifier != "none":
                message = TrackClassificationStamped()
                message.header.stamp = copy.deepcopy(stamp)
                message.header.frame_id = "supervisor_fixture_model"
                message.valid = classifier != "invalid"
                message.track_type, message.position = method_track, position
                message.track_probability, message.position_probability = 1., 1.
                message.track_entropy, message.position_entropy, message.curvature_feature = 0., 0., 0.
                if classifier == "future":
                    message.header.stamp.sec += 1
                elif classifier == "bad_probability":
                    message.track_probability = 2.
                elif classifier == "duplicate":
                    if self.held_classification is None:
                        self.held_classification = copy.deepcopy(message)
                    message = copy.deepcopy(self.held_classification)
                self.sent_stamps.add(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)
                self.class_pub.publish(message)
            if rc != "none":
                message = RcInput()
                message.header.stamp = copy.deepcopy(stamp)
                message.header.frame_id = "synthetic_rc"
                self.sequence += 1
                message.sequence, message.valid = self.sequence, rc != "invalid"
                message.rf_state = RcInput.RF_UNKNOWN
                message.throttle_us, message.steering_us = throttle, steering
                message.reason = "supervisor_software_fixture"
                self.rc_pub.publish(message)
            for _ in range(24):
                self.executor.spin_once(timeout_sec=0)
            time.sleep(.008)

    def service(self, name, value):
        from std_srvs.srv import SetBool
        client = self.source.create_client(SetBool, name)
        try:
            if not client.wait_for_service(timeout_sec=2.):
                raise RuntimeError("Service unavailable: " + name)
            future = client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic() + 2.
            while not future.done() and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=.005)
            if not future.done():
                raise RuntimeError("Service timed out: " + name)
            return future.result().success
        finally:
            self.source.destroy_client(client)

    def method(self, name):
        from rclpy.parameter import Parameter
        results = self.supervisor.set_parameters([Parameter("method", value=name)])
        if not all(result.successful for result in results):
            raise RuntimeError("Cannot select supervisor method " + name)

    def latest(self, kind):
        if not self.records[kind]:
            raise RuntimeError("No observations for " + kind)
        return self.records[kind][-1][1]

    def close(self):
        clean = True
        if self.backend is not None:
            try:
                self.backend.stop()
            except Exception:
                clean = False
        if hasattr(self, "executor"):
            self.executor.shutdown()
        for node in reversed(self.nodes):
            try:
                node.destroy_node()
            except Exception:
                clean = False
        self.nodes = []
        if self.context.ok():
            self.context.shutdown()
        return clean


def run(output):
    from ebva_control_msgs.msg import SelectedControl
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    result = {"passed": False, "kind": "synthetic_real_dds_supervisor_pipeline",
              "devices_started": [], "physical_qualification": False,
              "domain_id": os.environ.get("ROS_DOMAIN_ID"), "scenarios": {}}
    rig = None
    def check(condition, label):
        if not condition:
            raise RuntimeError(label)
        result["scenarios"][label] = True
    def approx(actual, expected):
        return abs(actual - expected) < 1e-4
    try:
        rig = Rig()
        result["profile"] = rig.profile
        rig.pump(.9)
        check(not rig.backend.core.enabled and not rig.command_authority.core.autonomy_enabled,
              "startup_remains_disarmed_and_manual")
        check(rig.latest("motor").data == 0., "startup_zero_output")
        check(rig.service("control/set_actuation_enabled", True), "explicit_neutral_arm")
        rig.pump(.1)
        check(rig.service("control/set_autonomy_enabled", True), "explicit_autonomy_enable")
        rig.pump(.35)
        check(rig.latest("selection").mode == SelectedControl.AUTONOMY
              and approx(rig.latest("motor").data, 100.), "supervisor_reaches_authority_and_backend")
        for mode, track, position, longitudinal, steering in [
            ("pi_straight", "straight", "middle", 1., 0.),
            ("p_curve", "left_curve", "middle", .6, -.5),
            ("p_intersection", "intersection", "left", .25, .8),
            ("wall_recovery", "walls", "middle", -.5, 0.),
            ("straight_rule", "straight", "middle", 1., 0.)]:
            rig.method(mode)
            rig.pump(.25, method_track=track, position=position)
            command = rig.latest("effort")
            check(command.contract == "supervisor_" + mode + "_v1"
                  and approx(command.longitudinal, longitudinal)
                  and approx(command.steering, steering), "fixed_" + mode)
        rig.method("straight_rule")
        rig.pump(.25, method_track="left_curve")
        check(rig.latest("effort").longitudinal == 0., "straight_rule_other_class_zero")
        rig.method("supervised")
        rig.pump(.25, method_track="right_curve")
        check(rig.latest("effort").contract == "supervisor_p_curve_v1", "automatic_classification_selection")
        rig.pump(.25, method_track="walls")
        check(rig.latest("effort").contract == "supervisor_wall_recovery_v1"
              and rig.latest("effort").longitudinal < 0., "automatic_wall_recovery")
        rig.pump(.3)
        rig.pump(.25, throttle=1700., steering=1700.)
        check(rig.latest("selection").mode == SelectedControl.MANUAL
              and approx(rig.latest("motor").data, -600.)
              and approx(rig.latest("servo").data, .6), "rc_priority_whole_pair")
        rig.pump(.25)
        check(rig.latest("selection").mode == SelectedControl.AUTONOMY
              and approx(rig.latest("motor").data, 100.), "valid_neutral_autonomy_resume")
        rig.pump(.3, rc="invalid")
        check(rig.latest("selection").mode == SelectedControl.STOP and rig.latest("motor").data == 0.,
              "invalid_rc_blocks_autonomy")
        rig.pump(.25)
        rig.pump(.4, rc="none")
        check(rig.latest("selection").mode == SelectedControl.STOP and rig.latest("motor").data == 0.,
              "rc_silence_blocks_autonomy")
        rig.pump(.3)
        check(approx(rig.latest("motor").data, 100.), "fresh_rc_recovery")
        for invalid in ("invalid", "future", "bad_probability", "duplicate", "none"):
            rig.pump(.25)
            rig.held_classification = None
            started = time.monotonic()
            rig.pump(.5, classifier=invalid)
            check(rig.latest("selection").mode == SelectedControl.STOP
                  and rig.latest("motor").data == 0., "classifier_" + invalid + "_stops")
            check(not any(at > started + .3 for at, _ in rig.records["effort"]),
                  "classifier_" + invalid + "_does_not_renew_output")
        rig.pump(.3)
        check(approx(rig.latest("motor").data, 100.), "fresh_classification_recovery")
        check(all(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                  in rig.sent_stamps and message.header.frame_id == "supervisor_fixture_model"
                  for _, message in rig.records["effort"]), "original_classifier_headers_preserved")
        check(all(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec in rig.sent_stamps
                  and m.header.frame_id == "supervisor_fixture_model"
                  for _, m in rig.records["actuators"]), "adapter_preserves_effort_lease")
        check(rig.service("control/set_actuation_enabled", False), "explicit_disarm")
        rig.pump(.2)
        check(rig.latest("motor").data == 0. and not rig.backend.core.enabled,
              "final_disarmed_zero")
        result["observations"] = {key: len(rows) for key, rows in rig.records.items()}
        result["passed"] = True
    except Exception as error:
        result["error"] = repr(error)
    finally:
        if rig is not None:
            result["clean_shutdown"] = rig.close()
            if not result["clean_shutdown"]:
                result["passed"] = False
        else:
            result["clean_shutdown"] = False
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        parser.error("Use ROS_LOCALHOST_ONLY=1 and an explicit isolated ROS_DOMAIN_ID")
    result = run(args.output)
    print(json.dumps(result, indent=2), flush=True)
    return int(not result["passed"])


if __name__ == "__main__":
    raise SystemExit(main())

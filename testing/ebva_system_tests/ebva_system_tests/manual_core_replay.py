"""Finite rosbag2 exercise of the selected manual core, always without devices."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from .manual_fixture import NAMESPACE, inspect_profile

ORIGIN = 7000.
DURATION = 7.5


def make_bag(output):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from ebva_control_msgs.msg import RcInput, ActuatorCommand
    from std_msgs.msg import Empty
    events = []
    prefix = "/" + NAMESPACE
    topics = {prefix + "/rc/input": "ebva_control_msgs/msg/RcInput",
              prefix + "/control/autonomy/actuators": "ebva_control_msgs/msg/ActuatorCommand",
              prefix + "/replay_tick": "std_msgs/msg/Empty"}
    for index in range(round(DURATION / .02) + 1):
        t = round(index * .02, 6)
        stamp = round((ORIGIN + t) * 1e9)
        if t < 6.8 and not 4.6 <= t < 5.2:
            rc = RcInput()
            rc.header.stamp.sec, rc.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            rc.header.frame_id = "synthetic_rc"
            rc.sequence, rc.valid, rc.rf_state = index + 1, True, RcInput.RF_UNKNOWN
            manual = .8 <= t < 1.2 or 1.8 <= t < 2.4 or 3.6 <= t < 4.2
            rc.throttle_us = 1700. if manual else 1500.
            rc.steering_us = 1700. if manual else 1500.
            rc.reason = "manual_core_software_fixture"
            events.append((stamp, 0, prefix + "/rc/input", rc))
        if t < 6.8 and not 5.7 <= t < 6.3:
            auto = ActuatorCommand()
            auto.header.stamp.sec, auto.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            auto.header.frame_id = "software_fixture_only"
            auto.erpm, auto.servo = 400., .45
            events.append((stamp, 1, prefix + "/control/autonomy/actuators", auto))
    # Advance /clock through the final loss without refreshing RC or autonomy.
    events.append((round((ORIGIN + DURATION) * 1e9), 2, prefix + "/replay_tick", Empty()))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output / "input_bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, kind in topics.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format="cdr"))
    for stamp, _, topic, message in sorted(events, key=lambda item: (item[0], item[1])):
        writer.write(topic, serialize_message(message), stamp)
    del writer
    return {"kind": "synthetic_manual_core_no_vehicle_calibration", "messages": len(events), "topics": topics}


def verify(events):
    def rows(kind, lo, hi):
        values = [event for event in events if event["kind"] == kind and lo <= event["t_s"] < hi]
        if not values:
            raise RuntimeError(f"No {kind} observations in {lo}..{hi}")
        return values
    def motor(lo, hi, value):
        if not all(abs(event["erpm"] - value) < 1e-8 for event in rows("motor", lo, hi)):
            raise RuntimeError(f"Unexpected motor value in {lo}..{hi}")
    def selection(lo, hi, mode):
        if not all(event["mode"] == mode for event in rows("selection", lo, hi)):
            raise RuntimeError(f"Unexpected authority mode in {lo}..{hi}")
    def servo(lo, hi, value):
        if not all(abs(event["servo"] - value) < 1e-8 for event in rows("servo", lo, hi)):
            raise RuntimeError(f"Mixed manual/autonomous pair in {lo}..{hi}")
    motor(.3, 1.2, 0.)
    selection(.3, .65, 1)
    if not all(event["reason"] == "manual_only" for event in rows("selection", .3, .65)):
        raise RuntimeError("Autonomy was not disabled at startup")
    selection(.95, 1.15, 1)
    motor(2., 2.25, -600.); servo(2., 2.25, .6); selection(2., 2.25, 1)
    motor(2.6, 2.9, 0.); servo(2.6, 2.9, .5); selection(2.6, 2.9, 1)
    motor(3.3, 3.5, 400.); servo(3.3, 3.5, .45); selection(3.3, 3.5, 2)
    motor(3.8, 4.05, -600.); servo(3.8, 4.05, .6); selection(3.8, 4.05, 1)
    motor(4.35, 4.5, 400.); servo(4.35, 4.5, .45); selection(4.35, 4.5, 2)
    for lo, hi in ((4.95, 5.1), (6.05, 6.2), (7.1, 7.45)):
        motor(lo, hi, 0.); selection(lo, hi, 0)
    for lo, hi in ((5.35, 5.55), (6.45, 6.65)):
        motor(lo, hi, 400.); selection(lo, hi, 2)
    statuses = rows("backend_status", .3, 1.2)
    if not all(event["enabled"] == "False" for event in statuses):
        raise RuntimeError("Backend did not start disabled")
    return {"startup_actuation_disabled_despite_manual_input": True,
            "startup_autonomy_disabled_despite_fresh_command": True,
            "explicit_neutral_arming": True, "manual_whole_pair": True,
            "manual_neutral_zero_motor_and_center_servo": True, "explicit_autonomy_enable": True,
            "rc_priority": True, "automatic_valid_neutral_resume": True,
            "rc_silence_stop": True, "autonomy_silence_stop": True,
            "fresh_input_recovery": True, "final_silence_stop": True}


def run(output):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float64
    from std_srvs.srv import SetBool
    from rosbag2_interfaces.srv import Resume, PlayNext
    from ebva_control_msgs.msg import RcInput, ActuatorCommand, SelectedControl
    report, _, _ = inspect_profile()
    result = {"passed": False, "kind": "synthetic_rosbag2_replay", "physical_qualification": False,
              "profile": report, "devices_started": [], "image": os.environ.get("VALIDATION_IMAGE", ""),
              "domain_id": os.environ.get("ROS_DOMAIN_ID"), "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY")}
    result["bag"] = make_bag(output)
    events, children, logs = [], [], []
    trace = (output / "trace.jsonl").open("w")
    rclpy.init()
    node = Node("manual_core_observer", namespace=NAMESPACE,
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    def record(kind, message, values):
        item = {"kind": kind, "t_s": node.get_clock().now().nanoseconds / 1e9 - ORIGIN, **values}
        if hasattr(message, "header"):
            item["stamp_ns"] = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        events.append(item); trace.write(json.dumps(item) + "\n")
    def status(message):
        if message.status:
            values = {item.key: item.value for item in message.status[0].values}
            record("backend_status", message, {"enabled": values.get("enabled"), "reason": message.status[0].message})
    subscriptions = [(RcInput, "rc/input", lambda m: record("rc", m, {"valid": m.valid})),
                     (ActuatorCommand, "control/autonomy/actuators", lambda m: record("autonomy", m, {})),
                     (SelectedControl, "control/selected", lambda m: record("selection", m, {"mode": m.mode, "reason": m.reason})),
                     (Float64, "commands/motor/speed", lambda m: record("motor", m, {"erpm": m.data})),
                     (Float64, "commands/servo/position", lambda m: record("servo", m, {"servo": m.data})),
                     (DiagnosticArray, "control/backend_status", status)]
    for kind, topic, callback in subscriptions:
        node.create_subscription(kind, topic, callback, 10)
    def pump_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            if predicate():
                return
            if any(child.poll() is not None for child in children):
                raise RuntimeError("Replay process exited early")
        raise RuntimeError("Replay readiness deadline exceeded")
    def service(name, value, timeout=1.):
        client = node.create_client(SetBool, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return False
            future = client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.005)
            return future.done() and future.result().success
        finally:
            node.destroy_client(client)
    armed, autonomy = False, False
    commands = [["ros2", "launch", "ebva_bringup", "bringup.launch.py", "profile:=" + report["profile_file"]],
                ["ros2", "bag", "play", str(output / "input_bag"), "--clock", "100", "--start-paused", "--disable-keyboard-controls"]]
    result["commands"] = commands
    try:
        for index, command in enumerate(commands):
            log = (output / ("launch.log" if index == 0 else "player.log")).open("w"); logs.append(log)
            children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        resume = node.create_client(Resume, "/rosbag2_player/resume")
        step = node.create_client(PlayNext, "/rosbag2_player/play_next")
        try:
            def graph_ready():
                names = {info.node_name for info in node.get_subscriptions_info_by_topic("/clock")}
                return ({"command_authority", "vesc_backend", "manual_core_observer"} <= names
                        and resume.service_is_ready() and step.service_is_ready()
                        and node.count_publishers("rc/input") == 1
                        and node.count_publishers("control/autonomy/actuators") == 1
                        and node.count_subscribers("control/selected") >= 2
                        and node.count_publishers("commands/motor/speed") == 1)
            pump_until(graph_ready, 15.)
            settled = time.monotonic() + .3
            while time.monotonic() < settled:
                rclpy.spin_once(node, timeout_sec=.005)
            future = step.call_async(PlayNext.Request())
            pump_until(future.done, 3.)
            if not future.result().success:
                raise RuntimeError("Cannot prime replay clock")
            pump_until(lambda: any(event["kind"] == "backend_status" and event.get("stamp_ns", 0) >= round(ORIGIN * 1e9)
                                  for event in events), 5.)
            future = resume.call_async(Resume.Request()); pump_until(future.done, 3.)
        finally:
            node.destroy_client(step); node.destroy_client(resume)
        deadline = time.monotonic() + 16.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
            t = node.get_clock().now().nanoseconds / 1e9 - ORIGIN
            if not armed and 1.4 <= t < 1.75:
                armed = service("control/set_actuation_enabled", True)
            if not autonomy and 3.05 <= t < 3.3:
                autonomy = service("control/set_autonomy_enabled", True)
            if children[0].poll() is not None:
                raise RuntimeError("Manual core exited during replay")
            if children[1].poll() is not None:
                if children[1].returncode != 0:
                    raise RuntimeError("Bag player failed")
                end = time.monotonic() + .25
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.005)
                break
        else:
            raise RuntimeError("Finite replay deadline exceeded")
        if not armed or not autonomy:
            raise RuntimeError("Explicit neutral arming/autonomy steps were not accepted")
        result["scenarios"] = verify(events)
        result["passed"] = True
    except Exception as error:
        result["error"] = repr(error)
    finally:
        result["cleanup_disarmed"] = service("control/set_actuation_enabled", False)
        service("control/set_autonomy_enabled", False)
        forced = False
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=7.)
                except subprocess.TimeoutExpired:
                    forced = True; os.killpg(child.pid, signal.SIGKILL); child.wait()
        for log in logs:
            log.close()
        launch_text = (output / "launch.log").read_text() if (output / "launch.log").exists() else ""
        result["child_returncodes"] = [child.returncode for child in children]
        result["clean_process_shutdown"] = (not forced and len(children) == 2
            and all(child.returncode == 0 for child in children)
            and launch_text.count("process has finished cleanly") >= 2
            and "process has died" not in launch_text and "Traceback (most recent call last)" not in launch_text)
        if not result["cleanup_disarmed"] or not result["clean_process_shutdown"]:
            result["passed"] = False
            result.setdefault("error", "Manual core did not disarm/exit cleanly")
        trace.close(); node.destroy_node(); rclpy.try_shutdown()
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    result = run(args.output.resolve())
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

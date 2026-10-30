"""Finite rosbag2 CSV/Frenet → Ackermann → RC authority replay, without devices."""
import argparse
import json
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .csv_frenet_software import inspect_profile, NAMESPACE


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import RcInput
    prefix = "/" + NAMESPACE
    types = {prefix+"/state/odometry":"nav_msgs/msg/Odometry", prefix+"/rc/input":"ebva_control_msgs/msg/RcInput"}
    events = []
    origin = round(fixture["origin_ros_s"] * 1e9)
    for i in range(round(fixture["duration_s"] / fixture["odom_period_s"]) + 1):
        t = round(i * fixture["odom_period_s"], 6)
        phase = next((p for p in fixture["phases"] if p["start"] <= t < p["end"]), None)
        if phase is None or phase["mode"] == "silence":
            continue
        ns = origin + round(t * 1e9)
        stamp = ns - round(.8e9) if phase["mode"] == "stale" else ns
        msg = Odometry()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.header.frame_id = "fixture_map"
        msg.child_frame_id = "wrong_body" if phase["mode"] == "wrong_body" else "fixture_rear_axle_flu"
        msg.pose.pose.position.x = .6 * t
        msg.pose.pose.orientation.w = 1.
        msg.twist.twist.linear.x = .6
        events.append((ns, prefix+"/state/odometry", msg))
    for i in range(round(fixture["duration_s"] / fixture["rc_period_s"]) + 1):
        t = round(i * fixture["rc_period_s"], 6)
        ns = origin + round(t * 1e9)
        rc = RcInput()
        rc.header.stamp.sec, rc.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        rc.header.frame_id = "fixture_rc"
        rc.sequence = i + 1
        rc.valid = True
        rc.rf_state = RcInput.RF_UNKNOWN
        rc.reason = "synthetic_replay"
        rc.throttle_us = 1500.
        lo, hi = fixture["manual_rc_interval_s"]
        rc.steering_us = 1700. if lo <= t < hi else 1500.
        events.append((ns, prefix+"/rc/input", rc))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, type_name in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format="cdr"))
    for ns, topic, message in sorted(events, key=lambda event:(event[0], event[1])):
        writer.write(topic, serialize_message(message), ns)
    del writer
    return {"kind":fixture["kind"], "messages":len(events), "topics":types}


def subscriptions(record):
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    return [(AckermannDriveStamped, "control/autonomy/ackermann", lambda m:record("ackermann", m,
                {"speed":m.drive.speed, "steering":m.drive.steering_angle})),
            (Odometry, "state/odometry", lambda m:record("odometry", m, {"body":m.child_frame_id}))]


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind, lo, hi):
        found = [event for event in events if event["kind"] == kind and lo <= event["t_s"] < hi]
        if not found:
            raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return found
    commands = rows("ackermann", 1., 5.)
    if not all(.4-1e-6 <= event["speed"] <= .5+1e-6 and abs(event["steering"]) <= .4+1e-6 for event in commands):
        raise RuntimeError("Command left the explicit model reference domain")
    if not any(event["steering"] > .01 for event in commands) or not any(event["steering"] < -.01 for event in commands):
        raise RuntimeError("Synthetic reference did not exercise both steering signs")
    if not all(event["mode"] == SelectedControl.MANUAL for event in rows("selection", 3.75, 3.95)):
        raise RuntimeError("RC did not take full priority")
    if not all(event["erpm"] == 0 for event in rows("motor", 3.75, 3.95)) or not all(
            abs(event["servo"]-.6) < 1e-6 for event in rows("servo", 3.75, 3.95)):
        raise RuntimeError("Manual pair mixed with autonomous commands")
    if not all(event["mode"] == SelectedControl.AUTONOMY for event in rows("selection", 4.2, 4.5)):
        raise RuntimeError("Valid neutral RC did not resume autonomy")
    for lo, hi in [(5.35, 5.55), (5.85, 6.15), (7.5, 7.9)]:
        if not all(event["erpm"] == 0 for event in rows("motor", lo, hi)):
            raise RuntimeError("Wrong body, stale odometry or silence did not expire the command")
        if not all(event["mode"] == SelectedControl.STOP for event in rows("selection", lo, hi)):
            raise RuntimeError("Expiry did not reach the authority")
    if not all(event["erpm"] > 0 for event in rows("motor", 6.55, 6.9)):
        raise RuntimeError("Fresh valid input did not recover")
    odometry = {event["stamp_ns"]:event for event in events if event["kind"] == "odometry"
                and event["body"] == "fixture_rear_axle_flu"}
    source = {event["stamp_ns"]:event for event in events if event["kind"] == "ackermann"}
    for stamp, command in source.items():
        if stamp not in odometry or command["frame"] != "fixture_rear_axle_flu" or odometry[stamp]["frame"] != "fixture_map":
            raise RuntimeError("Measurement stamp or explicit command model frame lost")
    matched = 0
    for event in (event for event in events if event["kind"] == "actuator"):
        command = source.get(event["stamp_ns"])
        if command is None:
            continue
        if event["frame"] != command["frame"] or abs(event["erpm"]-100*command["speed"]) > 1e-5 or abs(
                event["servo"]-(.5-.2*command["steering"])) > 1e-6:
            raise RuntimeError("Existing Ackermann fixture mapping changed")
        matched += 1
    if matched < 80:
        raise RuntimeError("Insufficient matched source conversions")
    return {"model_reference_domain":True, "both_steering_signs":True, "original_measurement_stamp":True,
        "explicit_command_body":True, "existing_ackermann_adapter_reused":True, "rc_priority":True,
        "neutral_resume":True, "wrong_body_expiry":True, "stale_input_expiry":True,
        "silence_expiry":True, "fresh_input_recovery":True, "matched_conversions":matched}


def run(output):
    from .software_replay import run_campaign
    fixture = json.loads((share()/"data/csv_frenet_replay.json").read_text())
    controller = {"launch":"csv_frenet_software.launch.py", "subscriptions":subscriptions,
        "expected_subscribers":{"state/odometry":{"csv_frenet_pp", "control_replay_observer"},
            "control/autonomy/ackermann":{"ackermann_adapter", "control_replay_observer"}},
        "clock_kinds":["selection", "backend_status"]}
    return run_campaign(output, inspect_profile(), fixture, make_bag, verify, controller)


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

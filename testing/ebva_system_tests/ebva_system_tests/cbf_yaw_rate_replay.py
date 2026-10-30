"""Finite synthetic three-source CBF yaw-rate rosbag2 campaign, no actuators."""
import argparse
import json
import math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .cbf_yaw_rate_software import NAMESPACE, inspect_profile


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import AccelerationYawRateStamped, RcInput
    prefix = "/" + NAMESPACE
    origin = round(fixture["origin_ros_s"] * 1e9)
    events = []
    types = {prefix+"/perception/scan": "sensor_msgs/msg/LaserScan",
             prefix+"/state/odometry": "nav_msgs/msg/Odometry",
             prefix+"/control/autonomy/nominal": "ebva_control_msgs/msg/AccelerationYawRateStamped",
             prefix+"/rc/input": "ebva_control_msgs/msg/RcInput"}
    def stamped(message, ns, frame):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        message.header.frame_id = frame
        return message
    for index in range(round(fixture["duration_s"]/fixture["period_s"])+1):
        t = round(index*fixture["period_s"], 6)
        ns = origin+round(t*1e9)
        phase = next((p for p in fixture["phases"] if p["start"] <= t < p["end"]), None)
        if phase is None or phase["mode"] == "silence":
            continue
        mode = phase["mode"]
        # Both acquisition timelines strictly increase, while the oldest source
        # alternates between body-frame scan and parent-frame velocity header.
        scan_offset, velocity_offset = ((10_000_000, 30_000_000) if index % 2 else (30_000_000, 10_000_000))
        if mode != "scan_silence":
            scan = stamped(LaserScan(), ns-scan_offset, fixture["body_frame"])
            scan.angle_min, scan.angle_max = -math.pi/2, math.pi/2
            scan.angle_increment = math.pi/180
            scan.range_min, scan.range_max = .05, 10.
            scan.scan_time = fixture["period_s"]
            scan.ranges = [.1 if mode == "infeasible" else 8.] * 181
            if mode == "bad_scan":
                scan.ranges[90] = float("nan")
            events.append((ns, 0, prefix+"/perception/scan", scan))
        if mode != "velocity_silence":
            velocity_stamp = ns-800_000_000 if mode == "stale_velocity" else ns-velocity_offset
            state = stamped(Odometry(), velocity_stamp, fixture["velocity_parent_frame"])
            state.child_frame_id = fixture["body_frame"]
            state.pose.pose.orientation.w = 1.
            state.twist.twist.linear.x = 4. if mode == "infeasible" else 1.
            events.append((ns, 1, prefix+"/state/odometry", state))
        if mode != "nominal_silence":
            nominal = stamped(AccelerationYawRateStamped(), ns, fixture["body_frame"])
            nominal.acceleration_mps2 = phase["acceleration_mps2"]
            nominal.yaw_rate_rps = phase["yaw_rate_rps"]
            events.append((ns, 2, prefix+"/control/autonomy/nominal", nominal))
    for index in range(round(fixture["duration_s"]/fixture["rc_period_s"])+1):
        t = round(index*fixture["rc_period_s"], 6)
        ns = origin+round(t*1e9)
        rc = stamped(RcInput(), ns, "fixture_rc")
        rc.sequence, rc.valid, rc.rf_state = index+1, True, RcInput.RF_UNKNOWN
        rc.reason, rc.throttle_us = "synthetic_replay", 1500.
        lo, hi = fixture["manual_rc_interval_s"]
        rc.steering_us = 1700. if lo <= t < hi else 1500.
        events.append((ns, 3, prefix+"/rc/input", rc))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    for topic, name in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=name, serialization_format="cdr"))
    for ns, _, topic, message in sorted(events, key=lambda e: (e[0], e[1])):
        writer.write(topic, serialize_message(message), ns)
    del writer
    return {"kind": fixture["kind"], "messages": len(events), "topics": types}


def subscriptions(record):
    from ebva_control_msgs.msg import CbfYawRateCommand, AccelerationYawRateStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String
    def ns(header):
        return header.stamp.sec*1_000_000_000+header.stamp.nanosec
    def command(message):
        record("cbf", message, {"acceleration": message.acceleration_mps2, "omega": message.yaw_rate_rps,
            "speed": message.longitudinal_velocity_mps, "scan_stamp_ns": ns(message.scan_header),
            "scan_frame": message.scan_header.frame_id, "velocity_stamp_ns": ns(message.velocity_header),
            "velocity_frame": message.velocity_header.frame_id, "velocity_child": message.velocity_child_frame_id})
    def status(message):
        value = json.loads(message.data)
        record("cbf_status", message, {"source": value["source"], "reason": value["reason"]})
    return [(CbfYawRateCommand, "control/autonomy/cbf_yaw_rate", command),
        (LaserScan, "perception/scan", lambda m: record("scan", m, {})),
        (Odometry, "state/odometry", lambda m: record("velocity", m,
            {"speed": m.twist.twist.linear.x, "child": m.child_frame_id})),
        (AccelerationYawRateStamped, "control/autonomy/nominal", lambda m: record("nominal", m,
            {"acceleration": m.acceleration_mps2, "omega": m.yaw_rate_rps})),
        (String, "cbf_yaw_rate/status", status),
        (DiagnosticArray, "control/cbf_adapter_status", lambda m: record("cbf_adapter_status", m,
            {"reason": m.status[0].message if m.status else ""}))]


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    body, parent = "fixture_rear_axle_flu", "fixture_odom"
    def rows(kind, lo, hi):
        result = [e for e in events if e["kind"] == kind and lo <= e["t_s"] < hi]
        if not result:
            raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return result
    for lo, hi, omega in ((.8, 1.3, 0.), (1.8, 2.2, .05), (2.8, 3.2, -.05)):
        if not all(abs(e["acceleration"]-.1) < 1e-10 and abs(e["omega"]-omega) < 1e-10
                   for e in rows("cbf", lo, hi)):
            raise RuntimeError("Open-space yaw_rate pair differs from preserved model")
    if not all(e["mode"] == SelectedControl.MANUAL for e in rows("selection", 3.75, 3.95)):
        raise RuntimeError("RC failed to override CBF pair")
    if not all(e["erpm"] == 0 for e in rows("motor", 3.75, 3.95)) or not all(
            abs(e["servo"]-.6) < 1e-6 for e in rows("servo", 3.75, 3.95)):
        raise RuntimeError("Manual output mixed with CBF autonomy")
    if not all(e["mode"] == SelectedControl.AUTONOMY for e in rows("selection", 4.15, 4.35)):
        raise RuntimeError("Neutral RC failed to resume valid CBF")
    expirations = {"scan": (4.85, 5.05), "velocity": (5.95, 6.15), "nominal": (6.95, 7.15),
                   "infeasible": (7.35, 7.55), "invalid": (7.8, 7.95),
                   "stale_velocity": (8.2, 8.35), "final": (9.45, 9.9)}
    for source, (lo, hi) in expirations.items():
        if not all(e["erpm"] == 0 for e in rows("motor", lo, hi)):
            raise RuntimeError("CBF input/gate did not expire output: " + source)
    for lo, hi in ((5.3, 5.5), (6.35, 6.55), (8.65, 8.9)):
        if not all(e["erpm"] > 0 for e in rows("motor", lo, hi)):
            raise RuntimeError("Fresh three-input CBF recovery failed")
    if not all(abs(e["erpm"]-24.) < 1e-8 for e in rows("motor", 5.3, 5.5)):
        raise RuntimeError("Positive acceleration cap/artificial fixture gain not exercised")
    if not any(e["reason"] == "infeasible_box_bound" for e in rows("cbf_status", 7.25, 7.6)):
        raise RuntimeError("Infeasible CBF result not diagnosed")
    if not any(e["source"] == "scan" and e["reason"] == "scan_rejected" for e in rows("cbf_status", 7.65, 7.95)):
        raise RuntimeError("Invalid scan not diagnosed")
    if not any(e["source"] == "velocity" and e["reason"] == "measurement_expired" for e in rows("cbf_status", 8.05, 8.35)):
        raise RuntimeError("Old velocity source time was refreshed")
    scans = {(e["stamp_ns"], e["frame"]) for e in events if e["kind"] == "scan"}
    velocities = {(e["stamp_ns"], e["frame"], e["child"], e["speed"]) for e in events if e["kind"] == "velocity"}
    nominals = {(e["stamp_ns"], e["frame"]) for e in events if e["kind"] == "nominal"}
    sources = {}
    for event in (e for e in events if e["kind"] == "cbf"):
        if ((event["stamp_ns"], event["frame"]) not in nominals
                or (event["scan_stamp_ns"], event["scan_frame"]) not in scans
                or (event["velocity_stamp_ns"], event["velocity_frame"], event["velocity_child"], event["speed"]) not in velocities
                or event["frame"] != body or event["scan_frame"] != body
                or event["velocity_frame"] != parent or event["velocity_child"] != body):
            raise RuntimeError("CBF original three headers, child or consumed velocity lost")
        stamps = [event["stamp_ns"], event["scan_stamp_ns"], event["velocity_stamp_ns"]]
        if max(stamps)-min(stamps) > 80_000_000:
            raise RuntimeError("CBF three-source skew exceeded declared domain")
        sources.setdefault(min(stamps), []).append(event)
    matched, oldest_frames = 0, set()
    for event in (e for e in events if e["kind"] == "actuator"):
        candidates = sources.get(event["stamp_ns"], [])
        if not candidates:
            raise RuntimeError("Actuator lacks original CBF source provenance")
        found = None
        for source in candidates:
            effective_speed = max(abs(source["speed"]), .1) * (-1 if source["speed"] < 0 else 1)
            angle = max(-math.pi/6, min(math.pi/6, math.atan(.33*source["omega"]/effective_speed)))
            erpm = 100*min(source["acceleration"], .24)
            servo = max(.4, min(.6, .5-.2*angle))
            oldest = min([(source["stamp_ns"], source["frame"]),
                          (source["scan_stamp_ns"], source["scan_frame"]),
                          (source["velocity_stamp_ns"], source["velocity_frame"])], key=lambda pair: pair[0])
            if (abs(event["erpm"]-erpm) < 1e-8 and abs(event["servo"]-servo) < 1e-8
                    and (event["stamp_ns"], event["frame"]) == oldest):
                found = source
                break
        if found is None:
            raise RuntimeError("CBF specific fixture conversion or exact oldest header mismatch")
        matched += 1
        oldest_frames.add(event["frame"])
    if matched < 100 or oldest_frames != {body, parent}:
        raise RuntimeError("Insufficient CBF conversions or oldest velocity-parent coverage")
    return {"straight_and_yaw_signs": True, "rc_priority": True, "neutral_resume": True,
        "scan_silence_expiry": True, "velocity_silence_expiry": True, "nominal_silence_expiry": True,
        "infeasible_gate": True, "invalid_scan": True, "old_velocity": True, "fresh_recovery": True,
        "three_original_headers_and_velocity": True, "oldest_header_exact_including_parent": True,
        "acceleration_cap_and_artificial_mapping": True, "final_silence": True,
        "matched_conversions": matched, "oldest_frames": sorted(oldest_frames)}


def run(output):
    from .software_replay import run_campaign
    fixture = json.loads((share()/"data/cbf_yaw_rate_replay.json").read_text())
    controller = {"launch": "cbf_yaw_rate_software.launch.py", "subscriptions": subscriptions,
        "expected_subscribers": {
            "perception/scan": {"cbf_yaw_rate", "control_replay_observer"},
            "state/odometry": {"cbf_yaw_rate", "control_replay_observer"},
            "control/autonomy/nominal": {"cbf_yaw_rate", "control_replay_observer"},
            "control/autonomy/cbf_yaw_rate": {"cbf_yaw_rate_adapter", "control_replay_observer"},
            "cbf_yaw_rate/status": {"control_replay_observer"},
            "control/cbf_adapter_status": {"control_replay_observer"}},
        "clock_kinds": ["cbf_adapter_status", "selection", "backend_status"]}
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

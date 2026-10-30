"""Finite synthetic rosbag2-player qualification of the opt-in SI research prior."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ORIGIN_NS = 100_000_000_000
NAMESPACE = "/imu_prior_replay_fixture"


def run_campaign(output):
    """No drivers/actuators/TF; caller supplies isolated ROS localhost environment."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from ament_index_python.packages import get_package_share_directory
    import rosbag2_py
    from rosbag2_interfaces.srv import PlayNext, Resume
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ebva_localization.imu_odometry import ImuOdometry
    from ebva_localization.state_contract import stamp_nanoseconds

    output = Path(output)
    if output.exists():
        raise ValueError("replay output must be a new directory")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise ValueError("set ROS_LOCALHOST_ONLY=1 and use an isolated ROS_DOMAIN_ID")
    if not os.environ.get("ROS_DOMAIN_ID"):
        raise ValueError("an explicit isolated ROS_DOMAIN_ID is required")
    output.mkdir(parents=True)
    share = Path(get_package_share_directory("ebva_localization"))
    values = json.loads((share/"config/imu_prior/si_fixture.json").read_text())
    cases = []
    def add(t, change=None, stamp_t=None):
        msg = Imu()
        stamp = ORIGIN_NS+round((t if stamp_t is None else stamp_t)*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = "fixture_base"
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.806
        if change:
            change(msg)
        cases.append((ORIGIN_NS+round(t*1e9), msg))
    for t in (.5, .55, .6, .65):
        add(t)
    add(.7, lambda m: setattr(m.linear_acceleration, "x", .5))
    add(.75, stamp_t=.7)
    add(.8, lambda m: setattr(m.orientation, "w", 0.))
    add(.85)
    add(.9)
    add(.95, lambda m: setattr(m.header, "frame_id", "wrong_body"))
    add(1.)
    add(1.05)
    add(1.4, stamp_t=1.1)
    add(1.45)
    add(1.5)
    add(2.)
    add(2.05)
    add(2.1, lambda m: setattr(m.linear_acceleration, "x", math.nan))
    add(2.15)
    add(2.2)
    writer = rosbag2_py.SequentialWriter()
    options = rosbag2_py.ConverterOptions("", "")
    bag = output/"input_bag"
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), options)
    for name, type_name in ((NAMESPACE+"/imu_si", "sensor_msgs/msg/Imu"),
                            (NAMESPACE+"/marker", "std_msgs/msg/String")):
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=type_name, serialization_format="cdr"))
    bag_events = [(ORIGIN_NS, NAMESPACE+"/marker", String(data="clock_prime")),
                  (ORIGIN_NS+1_750_000_000, NAMESPACE+"/marker", String(data="silence_checkpoint")),
                  (ORIGIN_NS+2_300_000_000, NAMESPACE+"/marker", String(data="finished"))]
    bag_events += [(stamp, NAMESPACE+"/imu_si", msg) for stamp, msg in cases]
    for stamp, topic, msg in sorted(bag_events, key=lambda e: e[0]):
        writer.write(topic, serialize_message(msg), stamp)
    del writer

    result = {
        "passed": False, "kind": "synthetic_rosbag2_player_with_clock",
        "physical_qualification": False, "devices_started": [], "publish_tf": False,
        "input_count": len(cases), "fixture": values,
        "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
        "image": os.environ.get("VALIDATION_IMAGE", ""),
        "uncertainty_estimated": False, "calibration_qualification": "software_fixture"}
    context = Context(); rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    prior = observer = player = None
    log = (output/"player.log").open("x")
    trace = (output/"trace.jsonl").open("x")
    outputs, statuses, markers = [], [], []
    forced = False

    def trace_event(kind, data):
        trace.write(json.dumps({"kind": kind, **data}, sort_keys=True)+"\n")
        trace.flush()

    def wait_for(predicate, timeout=8.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        if not predicate():
            raise RuntimeError("timed out waiting for replay condition")

    try:
        prior = ImuOdometry(
            context=context, namespace=NAMESPACE,
            parameter_overrides=[Parameter(k, value=v) for k, v in values.items()]
            +[Parameter("use_sim_time", value=True)])
        observer = Node("observer", namespace=NAMESPACE, context=context)
        executor.add_node(prior); executor.add_node(observer)
        def odometry(msg):
            outputs.append(msg)
            trace_event("odometry", {"stamp_ns": stamp_nanoseconds(msg.header.stamp),
                                    "position": [msg.pose.pose.position.x, msg.pose.pose.position.y],
                                    "frame": msg.header.frame_id, "body": msg.child_frame_id})
        def status(msg):
            value = json.loads(msg.data); statuses.append(value)
            trace_event("status", {**value, "observed_ros_ns": prior.get_clock().now().nanoseconds})
        def marker(msg):
            value = {"value": msg.data, "outputs": len(outputs), "statuses": len(statuses),
                     "observed_ros_ns": prior.get_clock().now().nanoseconds}
            markers.append(value); trace_event("marker", value)
        observer.create_subscription(Odometry, "imu_prior_odometry", odometry, 10)
        observer.create_subscription(String, "imu_odometry/status", status, 10)
        observer.create_subscription(String, "marker", marker, 10)
        command = ["ros2", "bag", "play", str(bag), "--clock", "100",
                   "--start-paused", "--disable-keyboard-controls"]
        result["command"] = command
        player = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        resume = observer.create_client(Resume, "/rosbag2_player/resume")
        step = observer.create_client(PlayNext, "/rosbag2_player/play_next")
        try:
            wait_for(lambda: resume.service_is_ready() and step.service_is_ready()
                     and prior.count_publishers(NAMESPACE+"/imu_si") == 1
                     and prior.publisher.get_subscription_count() == 1
                     and prior.status_publisher.get_subscription_count() == 1
                     and "/clock" in [s.topic_name for s in prior.subscriptions])
            # Let DDS settle after graph discovery, then step only the clock marker.
            until = time.monotonic()+.3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.005)
            future = step.call_async(PlayNext.Request())
            wait_for(future.done)
            if not future.result().success:
                raise RuntimeError("player could not prime ROS clock")
            wait_for(lambda: prior.get_clock().now().nanoseconds >= ORIGIN_NS)
            future = resume.call_async(Resume.Request())
            wait_for(future.done)
        finally:
            observer.destroy_client(resume); observer.destroy_client(step)
        wait_for(lambda: player.poll() is not None, timeout=15.)
        if player.returncode:
            raise RuntimeError("rosbag2 player failed")
        until = time.monotonic()+.3
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.005)
        expected_stamps = {ORIGIN_NS+round(t*1e9) for t in (.7, .9, 1.05, 1.5, 2.05, 2.2)}
        actual_stamps = [stamp_nanoseconds(msg.header.stamp) for msg in outputs]
        if len(outputs) != 6 or set(actual_stamps) != expected_stamps:
            raise AssertionError("unexpected or refreshed output stamps: "+repr(actual_stamps))
        if len(statuses) != len(cases):
            raise AssertionError("missing status/input messages")
        counts = Counter(s["reason"] for s in statuses)
        expected_rejections = ("measurement_not_newer", "nonunit_quaternion",
                               "input_frame_mismatch", "measurement_expired", "nonfinite_imu")
        if not all(counts[r] == 1 for r in expected_rejections):
            raise AssertionError("missing directed rejection: "+repr(counts))
        if counts["integration_rebased_after_gap"] != 5:
            raise AssertionError("missing invalid/silence recovery baselines")
        checkpoint = [m for m in markers if m["value"] == "silence_checkpoint"]
        if len(checkpoint) != 1 or checkpoint[0]["outputs"] != 4 or checkpoint[0]["statuses"] != 15:
            raise AssertionError("cached state/status appeared during silence")
        for msg in outputs:
            numbers = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                       msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if not all(math.isfinite(v) for v in numbers):
                raise AssertionError("nonfinite output")
            if msg.header.frame_id != values["parent_frame"] or msg.child_frame_id != values["child_frame"]:
                raise AssertionError("frame/body mismatch")
            if list(msg.pose.covariance) != values["pose_covariance"] or list(msg.twist.covariance) != values["twist_covariance"]:
                raise AssertionError("fixture covariance changed")
        if any(s["uncertainty_estimated"] or s["stationarity_verified"] or s["publish_tf"] for s in statuses):
            raise AssertionError("unsupported physical claim")
        result.update(passed=True, output_count=len(outputs), status_count=len(statuses),
                      output_stamps_ns=actual_stamps, reason_counts=dict(counts),
                      silence_checkpoint=checkpoint[0], exact_measurement_stamps=True,
                      scenarios=["calibration_no_output", *expected_rejections,
                                 "input_silence_no_heartbeat", "recovery_baseline",
                                 "finite_output", "explicit_covariance_and_quality"])
    except Exception as error:
        result["error"] = repr(error)
    finally:
        if player is not None and player.poll() is None:
            player.send_signal(signal.SIGINT)
            try: player.wait(timeout=6.)
            except subprocess.TimeoutExpired:
                forced = True; os.killpg(player.pid, signal.SIGKILL); player.wait()
        result["player_returncode"] = player.returncode if player is not None else None
        result["clean_process_shutdown"] = player is not None and player.returncode == 0 and not forced
        executor.shutdown()
        if prior is not None: prior.destroy_node()
        if observer is not None: observer.destroy_node()
        rclpy.shutdown(context=context)
        log.close(); trace.close()
        if not result["clean_process_shutdown"]:
            result["passed"] = False
        (output/"result.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run_campaign(args.output)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

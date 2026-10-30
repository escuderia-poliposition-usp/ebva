"""Finite rosbag2 Image -> LaneBoundaries qualification of all three lane methods."""
import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import cv2
import numpy as np

ORIGIN_NS = 100_000_000_000
NAMESPACE = "/lane_perception_replay_fixture"


def run_campaign(output):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.serialization import serialize_message
    from rosbag2_interfaces.srv import PlayNext, Resume
    from sensor_msgs.msg import Image
    from lane_msgs.msg import LaneBoundaries
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    import rosbag2_py
    from ebva_lane_detection.lane_detection_node import LaneDetectionNode
    from ebva_lane_detection.lane_segmentation import process_frame
    from ebva_lane_detection.segmentation_methods import METHODS, config_for_method
    from ebva_lane_detection.comparison import synthetic_cases

    output = Path(output)
    if output.exists():
        raise ValueError("replay output must be a new directory")
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        raise ValueError("explicit isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 required")
    output.mkdir(parents=True)
    bridge = CvBridge()
    frames = dict(synthetic_cases())
    cases = []
    def add(t, key="straight_road_l40", encoding="bgr8", change=None):
        frame = frames[key]
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif encoding == "mono8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = bridge.cv2_to_imgmsg(frame, encoding=encoding)
        stamp = ORIGIN_NS+round(t*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
        msg.header.frame_id = "fixture_image_optical"
        if change:
            change(msg)
        cases.append((stamp, msg))
    add(.5)
    add(1., key="straight_road_l65", encoding="rgb8")
    add(1.5, change=lambda m: setattr(m, "encoding", "16UC1"))
    add(2., encoding="mono8")
    add(2.5, change=lambda m: setattr(m, "step", 1))
    add(3., key="empty_gray")
    add(3.5, change=lambda m: setattr(m.header.stamp, "nanosec", 1_000_000_000))
    add(4., change=lambda m: setattr(m.header, "frame_id", ""))
    add(4.5, change=lambda m: setattr(m, "height", 1))
    add(5.)
    accepted_times = (.5, 1., 2., 3., 5.)
    accepted_stamps = {ORIGIN_NS+round(t*1e9) for t in accepted_times}
    expected = {method: {} for method in METHODS}
    for stamp, msg in cases:
        if stamp in accepted_stamps:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            for method in METHODS:
                expected[method][stamp] = process_frame(frame, config_for_method(method), return_uv=True)
    writer = rosbag2_py.SequentialWriter()
    bag = output/"input_bag"
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    for name, type_name in ((NAMESPACE+"/image", "sensor_msgs/msg/Image"),
                            (NAMESPACE+"/marker", "std_msgs/msg/String")):
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=type_name, serialization_format="cdr"))
    events = [(ORIGIN_NS, NAMESPACE+"/marker", String(data="clock_prime")),
              (ORIGIN_NS+5_800_000_000, NAMESPACE+"/marker", String(data="silence_checkpoint")),
              (ORIGIN_NS+6_000_000_000, NAMESPACE+"/marker", String(data="finished"))]
    events += [(stamp, NAMESPACE+"/image", msg) for stamp, msg in cases]
    for stamp, topic, msg in sorted(events, key=lambda e:e[0]):
        writer.write(topic, serialize_message(msg), stamp)
    del writer

    result = {"passed": False, "kind": "synthetic_rosbag2_player_with_clock",
              "physical_qualification": False, "devices_started": [], "actuators_started": [],
              "input_count": len(cases), "methods": list(METHODS),
              "domain_id": os.environ.get("ROS_DOMAIN_ID"),
              "localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
              "image": os.environ.get("VALIDATION_IMAGE", ""), "ground_truth_available": False,
              "temporal_coasting": False, "camera_metric_calibration": "synthetic_assumption_only"}
    context = Context(); rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    nodes, observer = [], None
    player = None; forced = False
    log = (output/"player.log").open("x")
    trace = (output/"trace.jsonl").open("x")
    outputs = {m: [] for m in METHODS}; statuses = {m: [] for m in METHODS}
    debug = {m: [] for m in METHODS}; markers = []
    def stamp_ns(msg):
        return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
    def record(kind, method, value):
        trace.write(json.dumps({"kind": kind, "method": method, **value}, sort_keys=True)+"\n")
        trace.flush()
    def wait_for(predicate, timeout=10.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        if not predicate():
            raise RuntimeError("timed out waiting for replay condition")
    try:
        observer = Node("observer", namespace=NAMESPACE, context=context)
        executor.add_node(observer)
        for method in METHODS:
            prefix = NAMESPACE+"/"+method
            remaps = ["--ros-args", "-r", "/lane_detection/lane_boundaries:="+prefix+"/lanes",
                      "-r", "/lane_detection/status:="+prefix+"/status",
                      "-r", "/lane_detection/birdseye_debug:="+prefix+"/debug"]
            node = LaneDetectionNode(context=context, namespace=prefix, cli_args=remaps,
                                     parameter_overrides=[Parameter("image_topic", value=NAMESPACE+"/image"),
                                     Parameter("camera_frame_id", value="fixture_camera_ground_xy_right"),
                                     Parameter("segmentation_method", value=method),
                                     Parameter("use_sim_time", value=True)])
            nodes.append(node); executor.add_node(node)
            def receive(msg, method=method):
                outputs[method].append(msg)
                record("lanes", method, {"stamp_ns": stamp_ns(msg), "frame": msg.header.frame_id,
                                          "left": [[p.x,p.y,p.z] for p in msg.left_lane],
                                          "right": [[p.x,p.y,p.z] for p in msg.right_lane]})
            def status(msg, method=method):
                value = json.loads(msg.data); statuses[method].append(value); record("status", method, value)
            def image(msg, method=method):
                debug[method].append(msg)
            observer.create_subscription(LaneBoundaries, prefix+"/lanes", receive, 10)
            observer.create_subscription(String, prefix+"/status", status, 10)
            observer.create_subscription(Image, prefix+"/debug", image, 10)
        def marker(msg):
            value = {"value": msg.data, "outputs": {m:len(outputs[m]) for m in METHODS},
                     "statuses": {m:len(statuses[m]) for m in METHODS}}
            markers.append(value); record("marker", "all", value)
        observer.create_subscription(String, NAMESPACE+"/marker", marker, 10)
        command = ["ros2", "bag", "play", str(bag), "--clock", "100", "--start-paused",
                   "--disable-keyboard-controls"]
        result["command"] = command
        player = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        resume = observer.create_client(Resume, "/rosbag2_player/resume")
        step = observer.create_client(PlayNext, "/rosbag2_player/play_next")
        try:
            wait_for(lambda: resume.service_is_ready() and step.service_is_ready()
                     and all(n.count_publishers(NAMESPACE+"/image") == 1
                             and n.lane_pub.get_subscription_count() == 1
                             and n.status_pub.get_subscription_count() == 1
                             and n.debug_pub.get_subscription_count() == 1 for n in nodes))
            until = time.monotonic()+.3
            while time.monotonic() < until: executor.spin_once(timeout_sec=.005)
            future = step.call_async(PlayNext.Request()); wait_for(future.done)
            if not future.result().success: raise RuntimeError("failed to prime player clock")
            wait_for(lambda: all(n.get_clock().now().nanoseconds >= ORIGIN_NS for n in nodes))
            future = resume.call_async(Resume.Request()); wait_for(future.done)
        finally:
            observer.destroy_client(resume); observer.destroy_client(step)
        wait_for(lambda: player.poll() is not None, timeout=25.)
        if player.returncode: raise RuntimeError("rosbag2 player failed")
        until = time.monotonic()+.3
        while time.monotonic() < until: executor.spin_once(timeout_sec=.005)
        for method in METHODS:
            actual = outputs[method]
            if len(actual) != 5 or {stamp_ns(msg) for msg in actual} != accepted_stamps:
                raise AssertionError("output stamps/count changed: "+method)
            if len(debug[method]) != 5 or len(statuses[method]) != len(cases):
                raise AssertionError("missing debug/status: "+method)
            for msg, dbg in zip(actual, debug[method]):
                reference = expected[method][stamp_ns(msg)]
                for points, xy in ((msg.left_lane, reference[1]), (msg.right_lane, reference[2])):
                    arr = np.array([[p.x,p.y] for p in points]).reshape(-1,2)
                    np.testing.assert_allclose(arr, np.asarray(xy).reshape(-1,2), atol=1e-10, rtol=0.)
                    if any(p.z != 0. for p in points): raise AssertionError("nonplanar geometry")
                if msg.header.frame_id != "fixture_camera_ground_xy_right":
                    raise AssertionError("camera coordinate frame changed")
                source = next(m for stamp,m in cases if stamp == stamp_ns(msg))
                if dbg.header != source.header:
                    raise AssertionError("debug overlay must preserve original image header, distinct from geometry frame")
                if dbg.encoding != "bgr8" or (dbg.height,dbg.width) != (source.height,source.width):
                    raise AssertionError("debug encoding/dimensions changed")
            empty = next(msg for msg in actual if stamp_ns(msg) == ORIGIN_NS+3_000_000_000)
            if empty.left_lane or empty.right_lane: raise AssertionError("coasted old geometry")
            counts = Counter(s["reason"] for s in statuses[method])
            for reason in ("unsupported_image_encoding", "invalid_image_step", "noncanonical_measurement_stamp",
                           "missing_or_invalid_input_frame", "invalid_image_dimensions"):
                if counts[reason] != 1: raise AssertionError("directed rejection missing: "+reason)
            if counts["accepted"] != 5: raise AssertionError("wrong accepted count")
        checkpoints = [m for m in markers if m["value"] == "silence_checkpoint"]
        if len(checkpoints) != 1 or any(checkpoints[0]["outputs"][m] != 5 for m in METHODS):
            raise AssertionError("missing silence checkpoint or cached heartbeat")
        result.update(passed=True, output_count=sum(map(len, outputs.values())),
                      outputs_per_method={m:len(outputs[m]) for m in METHODS},
                      status_count=sum(map(len,statuses.values())), exact_measurement_stamps=True,
                      exact_geometry_for_shared_input=True, silence_checkpoint=checkpoints[0],
                      scenarios=["bgr8_rgb8_mono8_conversion", "three_preserved_segmentation_methods",
                                 "invalid_encoding_step_stamp_frame_dimensions", "empty_detection_no_coasting",
                                 "image_silence_no_heartbeat", "recovery", "original_stamp_camera_frame",
                                 "debug_bgr8_input_dimensions"])
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
        for node in nodes: node.destroy_node()
        if observer is not None: observer.destroy_node()
        rclpy.shutdown(context=context)
        log.close(); trace.close()
        if not result["clean_process_shutdown"]: result["passed"] = False
        (output/"result.json").write_text(json.dumps(result,indent=2)+"\n")
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

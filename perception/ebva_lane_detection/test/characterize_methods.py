"""Read-only source/data characterization, not a runtime dependency or GT evaluation."""
import argparse
import ast
import hashlib
import importlib.util
import json
import sqlite3
import sys
from pathlib import Path
import cv2
import numpy as np


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--image-topic", help="explicit source Image topic; never infer from topic order")
    args = parser.parse_args()
    from ebva_lane_detection.comparison import synthetic_cases, compare_frame
    from ebva_lane_detection.segmentation_methods import METHODS, config_for_method
    from ebva_lane_detection import lane_segmentation as active
    import matplotlib.pyplot as plt
    plt.show = lambda: None
    sources = {
        "road_mask_v4_80": args.raw_root/"0ff8e9c2b718/lane_perception/lane_segmentation_v4.py",
        "road_mask_v4_50": args.raw_root/"32fd165b98a7/lane_perception/lane_segmentation_v4.py",
        "color_gradient_v3": args.raw_root/"0ff8e9c2b718/lane_perception/lane_segmentation_v3.py"}
    modules = {k: load_module(v, "original_"+k) for k, v in sources.items()}
    result = {"kind": "directed_comparison_not_accuracy_evaluation", "ground_truth_available": False,
              "sources": {k: {"path": str(p), "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                          for k, p in sources.items()}, "synthetic": [], "real": []}
    comparisons = []
    for name, frame in synthetic_cases():
        row = compare_frame(name, frame)
        for method in METHODS:
            cfg = config_for_method(method)
            actual = active.binarize(frame, cfg)
            try:
                original = modules[method].binarize(frame, cfg)
                if not np.array_equal(actual, original):
                    raise AssertionError("binarization drift: "+name+"/"+method)
                comparisons.append({"case": name, "method": method, "equal": True})
            except (TypeError, AttributeError) as error:
                if method != "road_mask_v4_80" or actual.any():
                    raise
                comparisons.append({"case": name, "method": method, "equal": None,
                                    "original_error": str(error), "fixed_empty_mask": True})
            finally:
                plt.close("all")
        result["synthetic"].append(row)
    result["original_binarization_comparisons"] = comparisons
    shared_names = ["build_perspective_transform", "warp_to_birdseye", "compute_scale_factors",
                    "spatial_continuity_filter", "find_histogram_bases", "_normalize", "crawl_lane",
                    "find_lane_candidates", "pixel_to_camera_frame", "select_immediate_lanes",
                    "resample_points", "candidate_to_camera_points", "LaneTracker", "draw_debug_image",
                    "overlay_on_original", "generate_synthetic_frame"]
    def nodes(path):
        return {n.name: n for n in ast.parse(Path(path).read_text()).body
                if isinstance(n, (ast.ClassDef, ast.FunctionDef))}
    current = nodes(active.__file__)
    result["shared_ast_sha256"] = {}
    for name in shared_names:
        dumped = ast.dump(current[name], include_attributes=False)
        for path in sources.values():
            if ast.dump(nodes(path)[name], include_attributes=False) != dumped:
                raise AssertionError("shared geometry changed: "+name)
        result["shared_ast_sha256"][name] = hashlib.sha256(dumped.encode()).hexdigest()
    old = nodes(sources["color_gradient_v3"])["binarize"]
    new = current["binarize_color_gradient_v3"]; new.name = "binarize"
    assert ast.dump(old, include_attributes=False) == ast.dump(new, include_attributes=False)
    result["v3_binarization_ast_preserved"] = True
    if args.bag:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        db = sqlite3.connect("file:"+str(args.bag.resolve())+"?mode=ro", uri=True)
        try:
            if not args.image_topic:
                raise ValueError("--image-topic is required with --bag")
            topic = db.execute("select id,name,type from topics where type = 'sensor_msgs/msg/Image' and name=?",
                               (args.image_topic,)).fetchone()
            if topic is None:
                raise ValueError("bag has no Image topic")
            count = db.execute("select count(*) from messages where topic_id=?", (topic[0],)).fetchone()[0]
            for numerator in (1,2,3):
                offset = (count-1)*numerator//4
                message_id, timestamp, data = db.execute(
                    "select id,timestamp,data from messages where topic_id=? order by timestamp,id limit 1 offset ?",
                    (topic[0], offset)).fetchone()
                msg = deserialize_message(data, Image)
                frame = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
                row = compare_frame("real_bag_quartile_"+str(numerator), frame)
                row["source"] = {"bag": str(args.bag), "topic": topic[1], "message_id": message_id,
                                 "database_timestamp_ns": timestamp, "ordered_offset": offset,
                                 "message_count": count, "serialized_cdr_sha256": hashlib.sha256(data).hexdigest(),
                                 "encoding": msg.encoding, "input_frame": msg.header.frame_id,
                                 "stamp_sec": msg.header.stamp.sec, "stamp_nanosec": msg.header.stamp.nanosec}
                result["real"].append(row)
        finally:
            db.close()
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"synthetic":len(result["synthetic"]), "real":len(result["real"]),
                      "original_comparisons":len(comparisons), "passed":True}))


if __name__ == "__main__":
    main()

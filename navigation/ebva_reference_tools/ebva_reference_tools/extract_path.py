#!/usr/bin/env python3
"""Offline tool: extract a reference path from a rosbag2 recording.

Reads /robot_pose (PoseStamped) from a rosbag2, resamples the trajectory at a
fixed arc-length spacing, applies light Gaussian smoothing, and saves the result
as a JSON array of waypoints: [{x, y, yaw}, ...].

Usage:
  ros2 run ebva_reference_tools extract_path <bag_dir> <output.json> [--ds 0.1] [--smooth 5]

Arguments:
  bag_dir     Directory containing the rosbag2 (the one passed to `ros2 bag record`)
  output      Output JSON file path
  --ds        Arc-length spacing between waypoints in metres (default: 0.1)
  --smooth    Gaussian smoothing window half-width in samples (default: 5, 0=off)
  --topic     PoseStamped topic to read (default: /robot_pose)
"""
import argparse
import json
import math
import sys

import numpy as np


def _quat_to_yaw(q):
    siny_cosp = 2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    cosy_cosp = 1.0 - 2.0 * (q['y'] ** 2 + q['z'] ** 2)
    return math.atan2(siny_cosp, cosy_cosp)


def _read_bag(bag_dir, topic):
    """Return list of (x, y, yaw) from PoseStamped messages in a rosbag2."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from geometry_msgs.msg import PoseStamped
    except ImportError as e:
        sys.exit(f'ERROR: {e}\nRun this script via `ros2 run ebva_reference_tools extract_path`')

    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = list(topic_types.keys())
        sys.exit(f'ERROR: topic {topic!r} not found in bag. Available: {available}')

    poses = []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t != topic:
            continue
        msg = deserialize_message(data, PoseStamped)
        q = msg.pose.orientation
        q_dict = {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}
        poses.append((msg.pose.position.x, msg.pose.position.y, _quat_to_yaw(q_dict)))

    return poses


def _resample(poses, ds):
    """Resample trajectory to uniform arc-length spacing ds (metres)."""
    if len(poses) < 2:
        return poses

    # Build cumulative arc length
    pts = np.array([[p[0], p[1]] for p in poses])
    diffs = np.diff(pts, axis=0)
    seg_len = np.hypot(diffs[:, 0], diffs[:, 1])
    arc = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = arc[-1]

    if total < ds:
        return poses

    # New sample positions
    s_new = np.arange(0.0, total, ds)

    x_new = np.interp(s_new, arc, pts[:, 0])
    y_new = np.interp(s_new, arc, pts[:, 1])

    # Yaw: interpolate via unwrapped angle to handle wrap-around
    yaws = np.array([p[2] for p in poses])
    yaws_unwrapped = np.unwrap(yaws)
    yaw_new = np.interp(s_new, arc, yaws_unwrapped)
    # Re-wrap to (-pi, pi]
    yaw_new = (yaw_new + math.pi) % (2 * math.pi) - math.pi

    return list(zip(x_new.tolist(), y_new.tolist(), yaw_new.tolist()))


def _smooth(poses, half_width):
    """Apply Gaussian smoothing with the given half-width (in samples)."""
    if half_width <= 0 or len(poses) < 3:
        return poses

    try:
        from scipy.ndimage import gaussian_filter1d
        sigma = half_width / 2.0
        arr = np.array(poses)
        arr[:, 0] = gaussian_filter1d(arr[:, 0], sigma)
        arr[:, 1] = gaussian_filter1d(arr[:, 1], sigma)
        yaws_u = np.unwrap(arr[:, 2])
        yaws_u = gaussian_filter1d(yaws_u, sigma)
        arr[:, 2] = (yaws_u + math.pi) % (2 * math.pi) - math.pi
        return [tuple(r) for r in arr.tolist()]
    except ImportError:
        # Fallback: simple moving average without scipy
        w = half_width
        arr = np.array(poses, dtype=np.float64)
        smoothed = arr.copy()
        for i in range(len(arr)):
            lo = max(0, i - w)
            hi = min(len(arr), i + w + 1)
            smoothed[i, :2] = arr[lo:hi, :2].mean(axis=0)
            # yaw: unwrap window
            yaw_win = np.unwrap(arr[lo:hi, 2])
            raw_mean = yaw_win.mean()
            smoothed[i, 2] = (raw_mean + math.pi) % (2 * math.pi) - math.pi
        return [tuple(r) for r in smoothed.tolist()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir', help='rosbag2 directory')
    parser.add_argument('output', help='output JSON file')
    parser.add_argument('--ds', type=float, default=0.1,
                        help='arc-length spacing in metres (default: 0.1)')
    parser.add_argument('--smooth', type=int, default=5,
                        help='Gaussian smoothing half-width in samples (0 = off, default: 5)')
    parser.add_argument('--topic', default='/robot_pose',
                        help='PoseStamped topic to read (default: /robot_pose)')
    args = parser.parse_args()
    if not math.isfinite(args.ds) or args.ds <= 0 or args.smooth < 0:
        parser.error('--ds deve ser finito/positivo e --smooth não-negativo')

    print(f'Reading {args.topic} from {args.bag_dir} ...')
    raw = _read_bag(args.bag_dir, args.topic)
    if not raw:
        sys.exit(f'ERROR: no messages found on {args.topic}')
    print(f'  Read {len(raw)} raw poses')

    resampled = _resample(raw, args.ds)
    print(f'  Resampled to {len(resampled)} waypoints (ds={args.ds} m)')

    smoothed = _smooth(resampled, args.smooth)
    if args.smooth > 0:
        print(f'  Smoothed (half-width={args.smooth} samples)')

    waypoints = [{'x': p[0], 'y': p[1], 'yaw': p[2]} for p in smoothed]

    with open(args.output, 'x') as f:
        json.dump(waypoints, f, indent=2)
    print(f'Saved {len(waypoints)} waypoints to {args.output}')


if __name__ == '__main__':
    main()

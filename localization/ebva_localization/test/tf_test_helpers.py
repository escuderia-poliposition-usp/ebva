"""Explicit software declarations shared by state/TF tests."""
from geometry_msgs.msg import TransformStamped


def profile(global_frame=False, namespace="/tf_contract_fixture"):
    body = "fixture_rear_axle_flu"
    result = {
        "profile_id": "synthetic_state_tf", "local_frame": "fixture_odom",
        "global_frame": "fixture_map" if global_frame else None,
        "body_frame": body, "require_tf": True,
        "static_lifetime": "publisher_present", "future_tolerance_sec": .02,
        "edges": [
            {"parent": "fixture_odom", "child": body, "kind": "dynamic",
             "publisher": namespace + "/local_estimator", "max_age_sec": .25},
            {"parent": body, "child": "fixture_laser", "kind": "static",
             "publisher": namespace + "/extrinsics"},
        ],
        "state_ports": [
            {"name": "local_motion", "publisher": namespace + "/local_estimator",
             "topic": namespace + "/state/local", "type": "nav_msgs/Odometry",
             "role": "local", "frame": "fixture_odom", "body": body,
             "model_point": "software_fixture_rear_axle", "stamp": "measurement",
             "provides": ["pose", "pose_covariance", "longitudinal_velocity"],
             "twist_frame": body, "max_age_sec": .2},
        ],
        "consumers": [
            {"name": "cbf_yaw_rate", "port": "local_motion", "type": "nav_msgs/Odometry",
             "frame": "fixture_odom", "body": body, "model_point": "software_fixture_rear_axle",
             "needs": ["longitudinal_velocity"], "max_age_sec": .2},
        ],
    }
    if global_frame:
        result["edges"].insert(0, {"parent": "fixture_map", "child": "fixture_odom",
            "kind": "dynamic", "publisher": namespace + "/global_estimator", "max_age_sec": .25})
        result["state_ports"].append({
            "name": "global_pose", "publisher": namespace + "/global_estimator",
            "topic": namespace + "/state/global", "type": "geometry_msgs/PoseWithCovarianceStamped",
            "role": "global", "frame": "fixture_map", "body": body,
            "model_point": "software_fixture_rear_axle", "stamp": "measurement",
            "provides": ["pose", "pose_covariance"], "max_age_sec": .2})
        result["consumers"].append({
            "name": "waypoint_pose", "port": "global_pose",
            "type": "geometry_msgs/PoseWithCovarianceStamped",
            "frame": "fixture_map", "body": body, "model_point": "software_fixture_rear_axle",
            "needs": ["pose"], "max_age_sec": .2})
    return result


def transform(parent="fixture_odom", child="fixture_rear_axle_flu", stamp_ns=10_000_000_000, x=1.):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.transform.translation.x = x
    message.transform.rotation.w = 1.
    return message


def graph(declaration):
    result = {"dynamic": {}, "static": {}}
    for edge in declaration["edges"]:
        result[edge["kind"]].setdefault(edge["publisher"], set()).add(edge["publisher"] + "_gid")
    return result

#!/usr/bin/env python3
"""Atomic waypoint effort/angle; no actuator writers or timer renewals."""
import copy
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from diagnostic_msgs.msg import DiagnosticArray
from ebva_control_msgs.msg import LongitudinalEffortSteering
from ..effort_contract import WaypointEffortContract, WAYPOINT_EFFORT_PARAMS, load_effort_waypoints
from ..effort_ros import original_stamp, publish_result
from .core import WaypointEffortCore

# These are the original laws, retained as the public family methods.
def quaternion_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))


def default_reference_path():
    return str(Path(get_package_share_directory('ebva_path_control'))/'data/waypoint_debug/centerline.csv')


class PPDebugController(Node):
    carrot_control = WaypointEffortCore.carrot_control
    stanley_control = WaypointEffortCore.stanley_control
    pure_pursuit_control = WaypointEffortCore.pure_pursuit_control

    def __init__(self, **kwargs):
        super().__init__('waypoint_control', **kwargs)
        params = {name: self.declare_parameter(name, '').value for name in WAYPOINT_EFFORT_PARAMS}
        if not all(params.values()):
            raise ValueError('Explicit waypoint effort reference/body/model contracts required')
        csv_path = self.declare_parameter('csv_path', '').value or default_reference_path()
        self.csv_path = csv_path
        self.contract = WaypointEffortContract(
            load_effort_waypoints(csv_path), **params,
            control_mode=self.declare_parameter('control_mode', 'carrot').value,
            observation_timeout=self.declare_parameter('observation_timeout', .2).value,
            future_slack=self.declare_parameter('future_slack', .02).value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.drive_pub = self.create_publisher(LongitudinalEffortSteering, self.declare_parameter('command_topic','effort_steering_command').value, qos)
        self.status_pub = self.create_publisher(DiagnosticArray, 'control/path_status', qos)
        self.marker_pub = self.create_publisher(Marker, 'control/lookahead', qos)
        self.odom_sub = self.create_subscription(Odometry, self.declare_parameter('odometry_topic','odometry').value, self.odom_callback, qos)

    def odom_callback(self, msg):
        stamp = original_stamp(msg.header)
        if stamp is None:
            self.contract.last_rejection = 'invalid_stamp'
            result = None
        else:
            pose = msg.pose.pose
            result = self.contract.accept(
                stamp=stamp, ros_now=self.get_clock().now().nanoseconds*1.e-9,
                monotonic_now=time.monotonic(), frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id, x=pose.position.x, y=pose.position.y,
                quaternion=(pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w))
        publish_result(self, msg.header, result)
        if result is not None:
            marker = Marker()
            marker.header = copy.deepcopy(msg.header)
            marker.ns = 'waypoint_control'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = map(float, result[2])
            marker.pose.position.z = .2
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .35
            marker.color.r = marker.color.a = 1.
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PPDebugController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

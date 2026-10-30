#!/usr/bin/env python3
"""Atomic model target from an explicitly aligned Cartesian body pose."""
import copy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .contract import WaypointContract, PATH_CONTRACT_PARAMS, load_waypoint_reference


class PurePursuitController(Node):
    def __init__(self, **node_kwargs):
        super().__init__('waypoint_pp', **node_kwargs)
        params = {name: self.declare_parameter(name, '').value
                  for name in ('csv_path', *PATH_CONTRACT_PARAMS)}
        if not all(params.values()):
            raise ValueError('waypoint_pp requires explicit path/frame/body/model contracts')
        timeout = self.declare_parameter('observation_timeout', .2).value
        future = self.declare_parameter('future_slack', .02).value
        pose_topic = self.declare_parameter('pose_topic', 'pose').value
        command_topic = self.declare_parameter('command_topic', 'ackermann_command').value
        if not pose_topic or not command_topic:
            raise ValueError('Nonempty input/output topic names required')
        waypoints = load_waypoint_reference(params.pop('csv_path'))
        self.contract = WaypointContract(waypoints, **params,
                                         observation_timeout=timeout, future_slack=future)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, command_topic, qos)
        self.status_pub = self.create_publisher(DiagnosticArray, 'control/path_status', qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, pose_topic,
                                                  self.pose_callback, qos)

    def pose_callback(self, msg):
        stamp = msg.header.stamp
        canonical = stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
        if not canonical:
            self.contract.last_rejection = 'invalid_stamp'
            command = None
        else:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            command = self.contract.accept(
                stamp=stamp.sec+stamp.nanosec*1.e-9,
                ros_now=self.get_clock().now().nanoseconds*1.e-9,
                monotonic_now=time.monotonic(), frame_id=msg.header.frame_id,
                x=p.x, y=p.y, quaternion=(q.x, q.y, q.z, q.w))
        status = DiagnosticStatus()
        status.name = 'waypoint_pp'
        status.hardware_id = 'software_contract'
        status.level = DiagnosticStatus.OK if command is not None else DiagnosticStatus.WARN
        status.message = self.contract.last_rejection or 'accepted'
        status.values = [KeyValue(key='body_frame', value=self.contract.body_frame_id),
                         KeyValue(key='source_stamp_sec', value=str(stamp.sec)),
                         KeyValue(key='source_stamp_nanosec', value=str(stamp.nanosec))]
        diagnostic = DiagnosticArray()
        diagnostic.header = copy.deepcopy(msg.header)
        diagnostic.status = [status]
        self.status_pub.publish(diagnostic)
        if command is None:
            return
        out = AckermannDriveStamped()
        out.header.stamp = copy.deepcopy(stamp)
        out.header.frame_id = self.contract.body_frame_id
        out.drive.speed, out.drive.steering_angle = command
        self.drive_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PurePursuitController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

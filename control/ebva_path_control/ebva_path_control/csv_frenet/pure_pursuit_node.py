#!/usr/bin/env python3
"""CSV/Frenet -> atomic kinematic target; no direct actuator topic or timer."""
import copy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ebva_path_control.references import load_frenet_reference
from .contract import CsvFrenetContract


class PurePursuitFrenetNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('csv_frenet_pp', **node_kwargs)
        required = ('csv_path', 'reference_frame_id', 'body_frame_id',
                    'model_point', 'reference_contract', 'progress_origin',
                    'progress_direction')
        params = {name: self.declare_parameter(name, '').value for name in required}
        if not all(params.values()):
            raise ValueError('csv_frenet_pp requires explicit CSV/frame/model/progress contract parameters')
        timeout = self.declare_parameter('observation_timeout', .2).value
        slack = self.declare_parameter('future_slack', .02).value
        odom_topic = self.declare_parameter('odometry_topic', 'odometry').value
        command_topic = self.declare_parameter('command_topic', 'ackermann_command').value
        if not odom_topic or not command_topic:
            raise ValueError('Explicit nonempty input/output topic names required')
        s_ref, ey_ref = load_frenet_reference(params.pop('csv_path'))
        self.contract = CsvFrenetContract(s_ref, ey_ref, **params,
                                          observation_timeout=timeout,
                                          future_slack=slack)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, command_topic, qos)
        self.status_pub = self.create_publisher(DiagnosticArray, 'control/path_status', qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v = msg.twist.twist.linear
        stamp = msg.header.stamp
        canonical_stamp = stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000
        if not canonical_stamp:
            self.contract.last_rejection = 'invalid_stamp'
            command = None
        else:
            command = self.contract.accept(
                stamp=stamp.sec+stamp.nanosec*1.e-9,
                ros_now=self.get_clock().now().nanoseconds*1.e-9,
                monotonic_now=time.monotonic(), frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id, x=p.x, y=p.y,
                quaternion=(q.x, q.y, q.z, q.w), vx=v.x, vy=v.y)
        status = DiagnosticStatus()
        status.name = 'csv_frenet_pp'
        status.hardware_id = 'software_contract'
        status.level = DiagnosticStatus.OK if command is not None else DiagnosticStatus.WARN
        status.message = self.contract.last_rejection or 'accepted'
        status.values = [KeyValue(key='progress_m', value=str(self.contract.core.s)),
                         KeyValue(key='body_frame', value=self.contract.body_frame_id),
                         KeyValue(key='source_stamp_s', value=str(
                             stamp.sec+stamp.nanosec*1.e-9) if canonical_stamp else 'invalid'),
                         KeyValue(key='source_stamp_sec', value=str(stamp.sec)),
                         KeyValue(key='source_stamp_nanosec', value=str(stamp.nanosec))]
        diagnostic = DiagnosticArray()
        diagnostic.header = copy.deepcopy(msg.header)
        diagnostic.status = [status]
        self.status_pub.publish(diagnostic)
        if command is None:
            return
        out = AckermannDriveStamped()
        # This is a body-relative command, not a relabelled world pose.
        out.header.stamp = copy.deepcopy(msg.header.stamp)
        out.header.frame_id = self.contract.body_frame_id
        out.drive.speed, out.drive.steering_angle = command
        self.drive_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PurePursuitFrenetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

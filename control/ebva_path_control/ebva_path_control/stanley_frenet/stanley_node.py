#!/usr/bin/env python3
"""Stamped clearance + odometry -> atomic Ackermann, never a timer heartbeat."""
import copy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ebva_control_msgs.msg import FrenetLateralErrorStamped
from .contract import StanleyFrenetContract, STANLEY_CONTRACT_PARAMS


def canonical_stamp(stamp):
    return stamp.sec >= 0 and 0 <= stamp.nanosec < 1_000_000_000


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


class StanleyFrenetNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('stanley_frenet', **node_kwargs)
        try:
            required = {name: self.declare_parameter(name, '').value
                        for name in STANLEY_CONTRACT_PARAMS}
            if not all(required.values()):
                raise ValueError('Explicit Stanley clearance/frame/body/model/reference parameters required')
            self.contract = StanleyFrenetContract(
                **required,
                observation_timeout=self.declare_parameter('observation_timeout', .2).value,
                future_slack=self.declare_parameter('future_slack', .02).value,
                maximum_skew=self.declare_parameter('maximum_skew', .08).value)
            topics = {key: self.declare_parameter(key, value).value for key, value in (
                ('odometry_topic', 'odometry'), ('lateral_error_topic', 'lateral_error'),
                ('command_topic', 'ackermann_command'))}
            if not all(topics.values()):
                raise ValueError('Nonempty input/output topics required')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.drive_pub = self.create_publisher(AckermannDriveStamped, topics['command_topic'], qos)
            self.status_pub = self.create_publisher(DiagnosticArray, 'control/path_status', qos)
            self.odom_sub = self.create_subscription(Odometry, topics['odometry_topic'], self.odom_callback, qos)
            self.error_sub = self.create_subscription(FrenetLateralErrorStamped,
                                                       topics['lateral_error_topic'], self.ey_callback, qos)
            self.odom_header = self.error_header = None
        except Exception:
            self.destroy_node()
            raise

    def _measurement_times(self):
        return self.get_clock().now().nanoseconds*1.e-9, time.monotonic()

    def odom_callback(self, msg):
        ros_now, monotonic_now = self._measurement_times()
        stamp = msg.header.stamp
        if not canonical_stamp(stamp):
            self.contract.last_rejection = 'invalid_odometry_stamp'
            accepted = False
        else:
            q, v = msg.pose.pose.orientation, msg.twist.twist.linear
            accepted = self.contract.observe_odometry(
                stamp=stamp.sec+stamp.nanosec*1.e-9, ros_now=ros_now,
                monotonic_now=monotonic_now, frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id, quaternion=(q.x, q.y, q.z, q.w),
                vx=v.x, vy=v.y)
        if accepted:
            self.odom_header = copy.deepcopy(msg.header)
        self._publish(msg.header, accepted, ros_now, monotonic_now)

    def ey_callback(self, msg):
        ros_now, monotonic_now = self._measurement_times()
        stamp = msg.header.stamp
        if not canonical_stamp(stamp):
            self.contract.last_rejection = 'invalid_lateral_error_stamp'
            accepted = False
        else:
            accepted = self.contract.observe_error(
                stamp=stamp.sec+stamp.nanosec*1.e-9, ros_now=ros_now,
                monotonic_now=monotonic_now, frame_id=msg.header.frame_id,
                reference_id=msg.reference_id, body_frame_id=msg.body_frame_id,
                model_point=msg.model_point, contract_id=msg.contract_id,
                lateral_error_m=msg.lateral_error_m)
        if accepted:
            self.error_header = copy.deepcopy(msg.header)
        self._publish(msg.header, accepted, ros_now, monotonic_now)

    def _publish(self, incoming_header, accepted, ros_now, monotonic_now):
        command = self.contract.command(ros_now, monotonic_now) if accepted else None
        status = DiagnosticStatus()
        status.name = 'stanley_frenet'
        status.hardware_id = 'software_contract'
        status.level = DiagnosticStatus.OK if command is not None else DiagnosticStatus.WARN
        status.message = self.contract.last_rejection or 'accepted'
        values = dict(reference_id=self.contract.reference_id,
                      reference_contract=self.contract.reference_contract,
                      body_frame_id=self.contract.body_frame_id,
                      model_point=self.contract.model_point,
                      signed_target_speed_mps='-0.6', speed_feedback='hypot_vx_vy',
                      heading_reference='fixed_zero_in_reference_frame')
        for name, header in (('odometry', self.odom_header), ('lateral_error', self.error_header)):
            if header is not None:
                values.update({name+'_stamp_sec': str(header.stamp.sec),
                               name+'_stamp_nanosec': str(header.stamp.nanosec),
                               name+'_frame_id': header.frame_id})
        if command is not None:
            # Select in exact integer ROS time, then deepcopy the original stamp.
            oldest_source, oldest = min((('odometry', self.odom_header),
                                         ('lateral_error', self.error_header)),
                                        key=lambda pair: stamp_ns(pair[1]))
            values['oldest_source'] = oldest_source
            output = AckermannDriveStamped()
            output.header.stamp = copy.deepcopy(oldest.stamp)
            output.header.frame_id = self.contract.body_frame_id
            output.drive.speed, output.drive.steering_angle = command
            self.drive_pub.publish(output)
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        diagnostic = DiagnosticArray()
        diagnostic.header = copy.deepcopy(incoming_header)
        diagnostic.status = [status]
        self.status_pub.publish(diagnostic)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StanleyFrenetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

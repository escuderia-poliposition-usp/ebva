"""Stamped scan/steering boundary for the selected FTG/PD method."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan, JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ebva_control_msgs.msg import FtgPdCommand
from ebva_control_contracts import MeasurementLease
from .ftg_pd_core import FtgPdCore, Scan, CONTRACT


def seconds(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9


class FtgPdNode(Node):
    def __init__(self, **kwargs):
        super().__init__("ftg_pd", **kwargs)
        try:
            self.frame=self.declare_parameter("expected_frame", "").value
            self.purpose=self.declare_parameter("feedback_purpose", "").value
            self.joint=self.declare_parameter("feedback_joint", "virtual_steering").value
            self.timeout=self.declare_parameter("source_timeout_s", .2).value
            self.skew=self.declare_parameter("max_input_skew_s", .08).value
            if self.purpose!="software_fixture_angle_rad" or not self.joint or not 0<self.skew<=self.timeout:
                raise ValueError("Explicit software fixture feedback contract required")
            self.scan_lease=MeasurementLease(self.timeout,self.frame)
            self.feedback_lease=MeasurementLease(self.timeout,self.frame)
            self.feedback_header=None;self.feedback=0.;self.last_output_time=None
            self.core=FtgPdCore();self.reason="waiting_for_inputs"
            self.pub=self.create_publisher(FtgPdCommand,"control/autonomy/ftg_pd",10)
            self.status=self.create_publisher(DiagnosticArray,"control/ftg_status",10)
            self.create_subscription(LaserScan,"sensors/scan",self.receive_scan,10)
            self.create_subscription(JointState,"sensors/steering",self.receive_feedback,10)
            self.create_timer(.1,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))
        except Exception:
            self.destroy_node();raise

    def receive_feedback(self, observation):
        if (list(observation.name)!=[self.joint] or len(observation.position)!=1
                or not math.isfinite(observation.position[0])):
            self.reason="invalid_steering_observation";return
        now=self.get_clock().now().nanoseconds*1e-9
        if self.feedback_lease.observe(seconds(observation.header),now,time.monotonic(),observation.header.frame_id):
            self.feedback=float(observation.position[0]);self.feedback_header=copy.deepcopy(observation.header)
        else:self.reason="invalid_steering_time_or_frame"

    @staticmethod
    def valid_scan(scan):
        return (len(scan.ranges)>=3 and all(math.isfinite(v) for v in
                (scan.angle_min,scan.angle_max,scan.angle_increment,scan.range_min,scan.range_max))
                and 0<scan.range_min<scan.range_max and scan.angle_increment>0
                and -math.pi<=scan.angle_min<=-math.radians(95)
                and math.radians(95)<=scan.angle_max<=math.pi
                and abs(scan.angle_max-(scan.angle_min+(len(scan.ranges)-1)*scan.angle_increment))<1e-4
                and all(math.isfinite(v) and scan.range_min<=v<=scan.range_max for v in scan.ranges))

    def receive_scan(self, scan):
        ros_now=self.get_clock().now().nanoseconds*1e-9;now=time.monotonic()
        if not self.valid_scan(scan):self.reason="invalid_scan";return
        if not self.feedback_lease.fresh(ros_now,now):self.reason="feedback_expired";return
        stamp=seconds(scan.header)
        if abs(stamp-self.feedback_lease.stamp)>self.skew:self.reason="input_skew";return
        previous=self.scan_lease.stamp
        if not self.scan_lease.observe(stamp,ros_now,now,scan.header.frame_id):
            self.reason="invalid_scan_time_or_frame";return
        if self.last_output_time is None or now-self.last_output_time>=self.timeout or stamp-previous>=self.timeout:
            self.core.reset()
        pure=Scan(list(scan.ranges),scan.angle_min,scan.angle_max,scan.angle_increment,scan.range_max)
        effort,throttle=self.core.step(pure,self.feedback)
        if not all(math.isfinite(v) and abs(v)<=3.402823e38 for v in (effort,throttle)):
            self.core.reset();self.reason="unrepresentable_controller_output";return
        output=FtgPdCommand();output.header=copy.deepcopy(scan.header)
        output.feedback_header=copy.deepcopy(self.feedback_header);output.contract=CONTRACT
        output.throttle=throttle;output.steering_effort=effort;self.pub.publish(output)
        self.last_output_time=now;self.reason="accepted"

    def report(self):
        report=DiagnosticArray();report.header.stamp=self.get_clock().now().to_msg()
        status=DiagnosticStatus();status.name="ebva_ftg_pd";status.message=self.reason
        status.level=DiagnosticStatus.OK if self.reason=="accepted" else DiagnosticStatus.WARN
        report.status=[status];self.status.publish(report)


def main(args=None):
    rclpy.init(args=args);node=None
    try:node=FtgPdNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

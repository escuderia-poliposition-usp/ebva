"""Guarded, atomic point-sample FRM ROS boundary; no actuator topics."""
import copy
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ebva_control_msgs.msg import DriveEffort
from .point_core import FrmPointCore, CONTRACT, INDICES


class FrmPointNode(Node):
    def __init__(self, **kwargs):
        super().__init__("frm_point_sample", **kwargs)
        self.expected_frame=self.declare_parameter("expected_scan_frame", "").value
        self.timeout=self.declare_parameter("source_timeout_s", .2).value
        if not self.expected_frame or not math.isfinite(self.timeout) or self.timeout<=0:
            self.destroy_node()
            raise ValueError("Explicit scan frame and positive timeout required")
        self.core=FrmPointCore()
        self.last_stamp_ns=self.last_receipt=None
        self.reason="waiting_for_scan"
        self.output=self.create_publisher(DriveEffort,"control/autonomy/effort",10)
        self.status=self.create_publisher(DiagnosticArray,"control/frm_status",10)
        self.create_subscription(LaserScan,"sensors/scan",self.receive,10)
        self.create_timer(.1,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, scan):
        stamp=scan.header.stamp.sec*1_000_000_000+scan.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)*1e-9
        if scan.header.frame_id!=self.expected_frame:
            self.reason="frame_mismatch"
        elif stamp<=0 or not -.02<=age<self.timeout:
            self.reason="stale_or_future"
        elif self.last_stamp_ns is not None and stamp<=self.last_stamp_ns:
            self.reason="duplicate_or_reordered_stamp"
        elif not self.valid_geometry(scan):
            self.reason="invalid_scan_geometry"
        elif not all(math.isfinite(scan.ranges[i]) and scan.range_min<=scan.ranges[i]<=scan.range_max for i in INDICES):
            self.reason="invalid_consumed_range"
        else:
            now=time.monotonic()
            # The preserved recurrence is per sample. Do not differentiate
            # across a missing-input interval as if it were one sample.
            if self.last_receipt is None or now-self.last_receipt>=self.timeout or (stamp-self.last_stamp_ns)*1e-9>=self.timeout:
                self.core.reset()
            steering,longitudinal=self.core.step(scan.ranges)
            command=DriveEffort();command.header=copy.deepcopy(scan.header)
            command.contract=CONTRACT;command.steering=float(steering);command.longitudinal=float(longitudinal)
            self.output.publish(command)
            self.last_stamp_ns,self.last_receipt=stamp,now
            self.reason="accepted"
        # Rejection never renews a command or its timestamp. The authority and
        # backend independently expire the last atomic output.

    @staticmethod
    def valid_geometry(scan):
        n=len(scan.ranges)
        return (n>max(INDICES) and all(math.isfinite(v) for v in
                (scan.angle_min,scan.angle_max,scan.angle_increment,scan.range_min,scan.range_max))
                and 0<=scan.range_min<scan.range_max
                and abs(scan.angle_min+3*math.pi/4)<1e-6
                and abs(scan.angle_increment-math.pi/720)<1e-6
                and abs(scan.angle_max-(scan.angle_min+(n-1)*scan.angle_increment))<1e-6)

    def report(self):
        report=DiagnosticArray();report.header.stamp=self.get_clock().now().to_msg()
        state=DiagnosticStatus();state.name="ebva_frm_point_sample";state.message=self.reason
        state.level=DiagnosticStatus.OK if self.reason=="accepted" else DiagnosticStatus.WARN
        report.status=[state];self.status.publish(report)


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=FrmPointNode();rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

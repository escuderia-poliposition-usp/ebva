"""Physical route adapter; inherited, unqualified calibration emits no commands."""
import copy, math
from dataclasses import fields
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from ebva_control_msgs.msg import ActuatorCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from .ackermann import AckermannMapping
from .nodes import parameter, qos, source_age, valid_age, spin

class NavigationAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__("navigation_adapter", **kwargs)
        self.qualified=parameter(self,"qualified",False)
        self.calibration_id=parameter(self,"calibration_id","")
        self.expected_frame=parameter(self,"expected_frame","")
        self.timeout=parameter(self,"source_timeout_s",.2)
        self.last_stamp=None
        if not self.expected_frame or not math.isfinite(self.timeout) or self.timeout<=0:
            raise ValueError("Explicit frame and finite source timeout required")
        from rclpy.parameter import Parameter
        values={f.name:self.declare_parameter(f.name,Parameter.Type.DOUBLE).value for f in fields(AckermannMapping)}
        self.mapping=None
        if self.qualified:
            if not self.calibration_id:raise ValueError("Qualified mapping requires calibration_id")
            self.mapping=AckermannMapping(**values)
        self.pub=self.create_publisher(ActuatorCommand,"control/autonomy/actuators",qos())
        self.status=self.create_publisher(DiagnosticArray,"control/navigation_adapter_status",qos())
        self.create_subscription(AckermannDriveStamped,"control/autonomy/ackermann",self.receive,qos())
        if not self.qualified:self.get_logger().warning("Autonomous conversion blocked: physical calibration pending")

    def receive(self,msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        reason="calibration_pending"
        if self.mapping is not None:
            reason="invalid_frame_or_stamp"
            if (msg.header.frame_id==self.expected_frame and stamp>0 and
                valid_age(source_age(self,msg.header),self.timeout) and
                (self.last_stamp is None or stamp>self.last_stamp)):
                try:value=self.mapping.convert(msg.drive.speed,msg.drive.steering_angle)
                except ValueError:reason="invalid_command"
                else:
                    out=ActuatorCommand();out.header=copy.deepcopy(msg.header)
                    out.erpm,out.servo=value.erpm,value.servo
                    self.pub.publish(out);self.last_stamp=stamp;reason="mapped"
        report=DiagnosticArray();report.header=copy.deepcopy(msg.header)
        status=DiagnosticStatus();status.name="ebva_navigation_adapter";status.message=reason
        status.level=DiagnosticStatus.OK if reason=="mapped" else DiagnosticStatus.WARN
        report.status=[status];self.status.publish(report)

def main(args=None):
    spin(NavigationAdapter,args)

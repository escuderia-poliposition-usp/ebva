"""ROS output helpers for the two effort / steering-angle consumers."""
import copy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ebva_control_msgs.msg import LongitudinalEffortSteering


def original_stamp(header):
    stamp = header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    return stamp.sec+stamp.nanosec*1.e-9


def publish_result(node, source_header, command):
    status = DiagnosticStatus()
    status.name = node.get_name()
    status.hardware_id = 'software_contract'
    status.level = DiagnosticStatus.OK if command is not None else DiagnosticStatus.WARN
    status.message = node.contract.last_rejection or 'accepted'
    status.values = [KeyValue(key='contract', value=node.contract.contract)]
    report = DiagnosticArray()
    report.header = copy.deepcopy(source_header)
    report.status = [status]
    node.status_pub.publish(report)
    if command is not None:
        out = LongitudinalEffortSteering()
        out.header = copy.deepcopy(source_header)
        out.contract = node.contract.contract
        out.longitudinal_effort = float(command[0])
        out.steering_angle_rad = float(command[1])
        node.drive_pub.publish(out)

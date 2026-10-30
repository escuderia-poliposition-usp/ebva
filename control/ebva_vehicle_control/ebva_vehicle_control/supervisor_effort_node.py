"""Supervisor-specific software fixture adapter; never physical calibration."""
from ebva_control_msgs.msg import DriveEffort
from .supervisor_effort import CONTRACTS, SupervisorEffortMapping
from .stamped_adapter import StampedFixtureAdapter
from .nodes import spin


class SupervisorEffortAdapter(StampedFixtureAdapter):
    node_name = "supervisor_effort_adapter"
    mapping_class = SupervisorEffortMapping
    input_type = DriveEffort
    input_topic = "control/autonomy/effort"
    status_topic = "control/supervisor_effort_status"
    diagnostic_heartbeat = True

    def accept_contract(self, message):
        return message.contract in CONTRACTS

    def values(self, message):
        return message.longitudinal, message.steering


def main(args=None):
    spin(SupervisorEffortAdapter, args)

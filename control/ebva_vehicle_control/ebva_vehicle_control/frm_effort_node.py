"""Specific FRM effort fixture adapter, never a universal throttle conversion."""
from ebva_control_msgs.msg import DriveEffort
from .frm_effort import FrmEffortMapping, CONTRACT
from .stamped_adapter import StampedFixtureAdapter
from .nodes import spin


class FrmEffortAdapter(StampedFixtureAdapter):
    node_name="frm_effort_adapter"
    mapping_class=FrmEffortMapping
    input_type=DriveEffort
    input_topic="control/autonomy/effort"
    status_topic="control/effort_status"
    diagnostic_heartbeat=True

    def accept_contract(self, message):
        return message.contract==CONTRACT

    def values(self, message):
        return message.longitudinal, message.steering


def main(args=None):
    spin(FrmEffortAdapter, args)

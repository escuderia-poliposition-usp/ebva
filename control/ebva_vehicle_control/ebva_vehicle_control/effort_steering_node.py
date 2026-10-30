"""Fixture adapter selected by one evidenced effort/angle producer contract."""
from ebva_control_msgs.msg import LongitudinalEffortSteering
from .effort_steering import EffortSteeringMapping, CONTRACTS
from .stamped_adapter import StampedFixtureAdapter
from .nodes import parameter, spin

class EffortSteeringAdapter(StampedFixtureAdapter):
    node_name = "effort_steering_adapter"
    mapping_class = EffortSteeringMapping
    input_type = LongitudinalEffortSteering
    input_topic = "control/autonomy/effort_steering"
    status_topic = "control/effort_steering_status"
    diagnostic_heartbeat = True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        try:
            self.expected_contract = parameter(self, "expected_contract", "")
            if self.expected_contract not in CONTRACTS:
                raise ValueError("Select an evidenced effort/angle contract")
        except Exception:
            self.destroy_node()
            raise

    def accept_contract(self, message):
        return (message.contract == self.expected_contract
                and message.header.stamp.sec >= 0
                and 0 <= message.header.stamp.nanosec < 1_000_000_000)

    def values(self, message):
        return message.longitudinal_effort, message.steering_angle_rad

def main(args=None):
    spin(EffortSteeringAdapter, args)

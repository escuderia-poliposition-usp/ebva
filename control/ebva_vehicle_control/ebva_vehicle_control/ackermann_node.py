"""Stamped Ackermann fixture adapter: input m/s and rad remain explicit."""
from ackermann_msgs.msg import AckermannDriveStamped
from .ackermann import AckermannMapping
from .stamped_adapter import StampedFixtureAdapter
from .nodes import spin


class AckermannAdapter(StampedFixtureAdapter):
    node_name="ackermann_adapter"
    mapping_class=AckermannMapping
    input_type=AckermannDriveStamped
    input_topic="control/autonomy/ackermann"
    status_topic="control/ackermann_status"

    def values(self, message):
        return message.drive.speed, message.drive.steering_angle


def main(args=None):
    spin(AckermannAdapter, args)

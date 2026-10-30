"""FTG pair adapter preserving the lifetime of the older input measurement."""
import time
from ebva_control_msgs.msg import FtgPdCommand
from ebva_control_contracts import MeasurementLease
from .stamped_adapter import StampedFixtureAdapter
from .ftg_pd_mapping import FtgPdMapping, CONTRACT
from .nodes import spin, stamp_seconds


class FtgPdAdapter(StampedFixtureAdapter):
    node_name="ftg_pd_adapter"
    mapping_class=FtgPdMapping
    input_type=FtgPdCommand
    input_topic="control/autonomy/ftg_pd"
    status_topic="control/ftg_adapter_status"
    diagnostic_heartbeat=True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        try:
            self.skew=self.declare_parameter("max_input_skew_s",.08).value
            if not 0<self.skew<=self.timeout:raise ValueError("Invalid input skew bound")
            self.feedback_lease=MeasurementLease(self.timeout,self.expected_frame)
        except Exception:self.destroy_node();raise

    def accept_contract(self, message):return message.contract==CONTRACT

    def values(self, message):return message.throttle,message.steering_effort

    def sources_valid(self, message):
        h=message.feedback_header;stamp=stamp_seconds(h.stamp)
        if h.frame_id!=self.expected_frame or abs(stamp-stamp_seconds(message.header.stamp))>self.skew:return False
        ros_now=self.get_clock().now().nanoseconds*1e-9;now=time.monotonic()
        if self.feedback_lease.stamp!=stamp:
            if not self.feedback_lease.observe(stamp,ros_now,now,h.frame_id):return False
        return self.feedback_lease.fresh(ros_now,now)

    def output_header(self, message):
        # Both measurements use the same explicitly selected fixture frame.
        # The complete source headers remain in FtgPdCommand for inspection.
        if stamp_seconds(message.feedback_header.stamp)<stamp_seconds(message.header.stamp):return message.feedback_header
        return message.header


def main(args=None):spin(FtgPdAdapter,args)

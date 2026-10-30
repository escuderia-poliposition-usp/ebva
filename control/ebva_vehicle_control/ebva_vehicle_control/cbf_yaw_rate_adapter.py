"""CBF yaw-rate fixture adapter retaining all three source lifetimes."""
import math
import time
from ebva_control_msgs.msg import CbfYawRateCommand
from ebva_control_contracts import MeasurementLease
from .stamped_adapter import StampedFixtureAdapter
from .cbf_yaw_rate_mapping import CbfYawRateMapping
from .nodes import spin


def canonical_seconds(header):
    stamp=header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    ns=stamp.sec*1_000_000_000+stamp.nanosec
    return ns*1e-9 if ns > 0 else None


class CbfYawRateAdapter(StampedFixtureAdapter):
    node_name="cbf_yaw_rate_adapter"
    mapping_class=CbfYawRateMapping
    input_type=CbfYawRateCommand
    input_topic="control/autonomy/cbf_yaw_rate"
    status_topic="control/cbf_adapter_status"
    diagnostic_heartbeat=True

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        try:
            self.velocity_parent_frame=self.declare_parameter("velocity_parent_frame", "").value
            self.skew=self.declare_parameter("max_input_skew_s", .08).value
            if (not self.velocity_parent_frame or self.velocity_parent_frame == self.expected_frame
                    or not math.isfinite(self.skew) or not 0 < self.skew <= self.timeout):
                raise ValueError("Explicit velocity parent and finite skew required")
            self.source_leases={"scan":MeasurementLease(self.timeout,self.expected_frame),
                "velocity":MeasurementLease(self.timeout,self.velocity_parent_frame)}
        except Exception:
            self.destroy_node();raise

    def accept_contract(self, message):
        return (all(canonical_seconds(h) is not None for h in
                    (message.header,message.scan_header,message.velocity_header))
                and message.velocity_child_frame_id == self.expected_frame)

    def values(self, message):
        return message.acceleration_mps2,message.yaw_rate_rps,message.longitudinal_velocity_mps

    def sources_valid(self, message):
        stamps=[canonical_seconds(h) for h in (message.header,message.scan_header,message.velocity_header)]
        if any(s is None for s in stamps) or max(stamps)-min(stamps) > self.skew+1e-12:
            return False
        ros_now=self.get_clock().now().nanoseconds*1e-9;now=time.monotonic()
        for name,header in (("scan",message.scan_header),("velocity",message.velocity_header)):
            stamp=canonical_seconds(header);lease=self.source_leases[name]
            if lease.stamp != stamp and not lease.observe(stamp,ros_now,now,header.frame_id):
                return False
            if header.frame_id != lease.frame or not lease.fresh(ros_now,now):
                return False
        return True

    def output_header(self, message):
        # This is acquisition provenance, not a relabelled body/world pose.
        return min((message.header,message.scan_header,message.velocity_header),
                   key=lambda h:h.stamp.sec*1_000_000_000+h.stamp.nanosec)


def main(args=None):
    spin(CbfYawRateAdapter,args)

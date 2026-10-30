"""Typed autonomous ports for two distinct preserved CBF models.

The shared yaw port provides identical scan/velocity provenance and time checks;
model-specific nominal and output types remain explicit and cannot be selected
by a ROS parameter. No physical conversion or manual RC occurs here.
"""
from copy import deepcopy
from ebva_control_msgs.msg import (AccelerationYawRateStamped,
    AccelerationSteeringAngleStamped, CbfYawRateMarginCommand,
    CbfSteeringTangentCommand)
from .yaw_rate_port import YawRatePort, stamp_ns
from .software import (CbfSoftwarePipeline, SourceStamp, YawRateNominal,
                       SteeringAngleNominal, PipelineDecision)


class _AlternateModelPort(YawRatePort):
    def __init__(self, control_frame, velocity_parent_frame, model_reference_point,
                 scan_geometry, timeout_s=.2, future_tolerance_s=.02, max_skew_s=.08):
        if future_tolerance_s >= timeout_s:
            raise ValueError("Future tolerance must be smaller than the observation lease")
        super().__init__(control_frame, velocity_parent_frame, model_reference_point,
                         scan_geometry, timeout_s, future_tolerance_s, max_skew_s)
        self.pipeline = CbfSoftwarePipeline(self.MODEL, frame=control_frame,
            timeout=timeout_s, future_slack=future_tolerance_s, max_skew=max_skew_s)

    def nominal(self, message, now_ns, monotonic_now):
        if not isinstance(message, self.NOMINAL_TYPE):
            return None, PipelineDecision("nominal_type_mismatch")
        reason = self._header(message.header, self.control_frame, now_ns)
        if reason:
            return None, PipelineDecision(reason)
        source = SourceStamp(stamp_ns(message.header.stamp)/1e9, message.header.frame_id)
        if self.MODEL == "steering_tangent":
            sample = SteeringAngleNominal(source, self.MODEL,
                message.acceleration_mps2, message.steering_angle_rad)
        else:
            sample = YawRateNominal(source, self.MODEL,
                message.acceleration_mps2, message.yaw_rate_rps)
        decision = self.pipeline.nominal(sample, now_ns/1e9, monotonic_now)
        if decision.command is None:
            return None, decision
        output = self.COMMAND_TYPE()
        output.header = deepcopy(message.header)
        output.scan_header = deepcopy(self.scan_header)
        output.velocity_header = deepcopy(self.velocity_header)
        output.velocity_child_frame_id = self.velocity_child
        output.longitudinal_velocity_mps = float(self.pipeline.speed.longitudinal_mps)
        output.acceleration_mps2 = decision.command.control[0]
        setattr(output, self.LATERAL_FIELD, decision.command.control[1])
        return output, decision


class YawRateMarginPort(_AlternateModelPort):
    MODEL = "yaw_rate_margin"
    NOMINAL_TYPE = AccelerationYawRateStamped
    COMMAND_TYPE = CbfYawRateMarginCommand
    LATERAL_FIELD = "yaw_rate_rps"


class SteeringTangentPort(_AlternateModelPort):
    MODEL = "steering_tangent"
    NOMINAL_TYPE = AccelerationSteeringAngleStamped
    COMMAND_TYPE = CbfSteeringTangentCommand
    LATERAL_FIELD = "steering_tangent"

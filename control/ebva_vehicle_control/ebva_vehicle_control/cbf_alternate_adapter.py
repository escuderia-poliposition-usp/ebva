"""Typed CBF adapters reuse source leases but keep each model's conversion."""
from ebva_control_msgs.msg import CbfYawRateMarginCommand,CbfSteeringTangentCommand
from .cbf_yaw_rate_adapter import CbfYawRateAdapter
from .cbf_alternate_mapping import CbfSteeringTangentMapping,UnrepresentableSteering
from .nodes import spin

class CbfYawRateMarginAdapter(CbfYawRateAdapter):
    node_name="cbf_yaw_rate_margin_adapter"
    input_type=CbfYawRateMarginCommand
    input_topic="control/autonomy/cbf_yaw_rate_margin"

class CbfSteeringTangentAdapter(CbfYawRateAdapter):
    node_name="cbf_steering_tangent_adapter"
    input_type=CbfSteeringTangentCommand
    input_topic="control/autonomy/cbf_steering_tangent"
    mapping_class=CbfSteeringTangentMapping
    def conversion_error_reason(self,error):
        return "unrepresentable_steering" if isinstance(error,UnrepresentableSteering) else "invalid_model_command"
    def values(self,message):
        return message.acceleration_mps2,message.steering_tangent,message.longitudinal_velocity_mps

def main_margin(args=None):spin(CbfYawRateMarginAdapter,args)
def main_tangent(args=None):spin(CbfSteeringTangentAdapter,args)

from ebva_control_msgs.msg import AccelerationSteeringAngleStamped
from ebva_cbf_control.alternate_model_port import YawRateMarginPort, SteeringTangentPort
from yaw_rate_fixtures import NOW, FRAME, PARENT, header, scan, velocity, nominal as yaw_nominal

PORTS = (YawRateMarginPort, SteeringTangentPort)


def port(model, **kwargs):
    cls = {p.MODEL: p for p in PORTS}[model]
    return cls(FRAME, PARENT, "rear_axle_center", "coincident_aligned", **kwargs)


def nominal(model, stamp=NOW, acceleration=.1, lateral=.05):
    if model == "yaw_rate_margin":
        return yaw_nominal(stamp, acceleration, lateral)
    message = header(AccelerationSteeringAngleStamped(), stamp)
    message.acceleration_mps2 = float(acceleration)
    message.steering_angle_rad = float(lateral)
    return message

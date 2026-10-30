import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ebva_control_msgs.msg import AccelerationYawRateStamped
from ebva_cbf_control.yaw_rate_port import YawRatePort

FRAME = "fixture_rear_axle_flu"
PARENT = "fixture_odom"
NOW = 10_000_000_000


def header(message, stamp=NOW, frame=FRAME):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id = frame
    return message


def scan(stamp=NOW, distance=8.):
    message = header(LaserScan(), stamp)
    message.angle_min, message.angle_max = -math.pi/2, math.pi/2
    message.angle_increment = math.pi/180
    message.range_min, message.range_max = .05, 10.
    message.ranges = [distance] * 181
    return message


def velocity(stamp=NOW, value=1.):
    message = header(Odometry(), stamp, PARENT)
    message.child_frame_id = FRAME
    message.twist.twist.linear.x = value
    return message


def nominal(stamp=NOW, acceleration=.1, omega=.05):
    message = header(AccelerationYawRateStamped(), stamp)
    message.acceleration_mps2, message.yaw_rate_rps = acceleration, omega
    return message


def port(**kwargs):
    return YawRatePort(FRAME, PARENT, "rear_axle_center", "coincident_aligned", **kwargs)

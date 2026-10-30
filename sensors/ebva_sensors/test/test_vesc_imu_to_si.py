"""Preserved IMU conversion contract; no node/device construction."""
import math
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
from sensor_msgs.msg import Imu
from ebva_sensors.vesc_imu_to_si import VescImuToSi


class VescImuConversionTests(unittest.TestCase):
    def test_conversion_stamp_covariance_and_source_preservation(self):
        msg = Imu()
        msg.header.stamp.sec = 18
        msg.angular_velocity.z = 180.0
        msg.linear_acceleration.z = 1.0
        msg.orientation.w = 1.0
        fake = SimpleNamespace(_frame_id="imu_link", _accel_to_ms2=9.806,
                               _deg_to_rad=math.pi / 180, _gyro_var=0.01,
                               _accel_var=0.25, _pub=Mock())
        VescImuToSi._callback(fake, msg)
        out = fake._pub.publish.call_args.args[0]
        self.assertAlmostEqual(out.angular_velocity.z, math.pi)
        self.assertAlmostEqual(out.linear_acceleration.z, 9.806)
        self.assertEqual(out.header.stamp, msg.header.stamp)
        self.assertEqual(out.header.frame_id, "imu_link")
        self.assertEqual(list(out.angular_velocity_covariance)[::4], [0.01]*3)
        self.assertEqual(out.orientation_covariance[0], -1.0)
        self.assertEqual(msg.angular_velocity.z, 180.0)
        self.assertEqual(msg.linear_acceleration.z, 1.0)

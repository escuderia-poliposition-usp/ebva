"""rosbag2 round-trip of directed state cases through the real projection callback."""
import json
from pathlib import Path
import tempfile
import unittest

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message, serialize_message

from ebva_localization.odometry_pose import OdometryPose
from test_state_contract import fake_node, odometry


class StateReplayTests(unittest.TestCase):
    def test_original_measurements_and_rejection_recovery_survive_rosbag_replay(self):
        cases = []
        def add(stamp, now, expected, mutate=None):
            message = odometry(stamp)
            if mutate:
                mutate(message)
            cases.append((message, now, expected))

        add(10_000_000_000, 10_000_000_000, "measurement_accepted")
        add(10_000_000_000, 10_010_000_000, "measurement_not_newer")
        add(9_990_000_000, 10_020_000_000, "measurement_not_newer")
        add(10_040_000_000, 10_240_000_000, "measurement_expired")
        add(10_500_000_000, 10_250_000_000, "measurement_in_future")
        add(10_251_000_000, 10_260_000_000, "parent_frame_mismatch",
            lambda m: setattr(m.header, "frame_id", "odom"))
        add(10_252_000_000, 10_270_000_000, "child_frame_mismatch",
            lambda m: setattr(m, "child_frame_id", "camera"))
        add(10_253_000_000, 10_280_000_000, "nonunit_quaternion",
            lambda m: setattr(m.pose.pose.orientation, "w", 2.))
        add(10_300_000_000, 10_310_000_000, "measurement_accepted",
            lambda m: setattr(m.pose.pose.position, "x", -2.))
        add(10_320_000_000, 10_330_000_000, "measurement_accepted",
            lambda m: setattr(m.pose, "covariance", [0.] * 36))
        add(0, 10_340_000_000, "invalid_stamp")
        add(10_350_000_000, 10_360_000_000, "measurement_accepted")
        add(8_000_000_000, 8_000_000_000, "clock_reset_required")
        add(10_500_000_000, 10_500_000_000, "clock_reset_required")

        with tempfile.TemporaryDirectory(prefix="ebva-localization-replay-") as temporary:
            uri = str(Path(temporary) / "state")
            options = rosbag2_py.ConverterOptions("", "")
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"), options)
            writer.create_topic(rosbag2_py.TopicMetadata(
                name="/fixture/state", type="nav_msgs/msg/Odometry",
                serialization_format="cdr"))
            for index, (message, _, _) in enumerate(cases):
                # Record order is independent of the simulated acquisition/clock epoch.
                writer.write("/fixture/state", serialize_message(message), index + 1)
            del writer
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"), options)
            fake = fake_node()
            replayed = 0
            while reader.has_next():
                topic, serialized, _ = reader.read_next()
                source, now, expected = cases[replayed]
                message = deserialize_message(serialized, Odometry)
                self.assertEqual(topic, "/fixture/state")
                self.assertEqual(message, source)
                fake.now_ns = now
                OdometryPose.on_odometry(fake, message)
                status = json.loads(fake.status_publisher.publish.call_args.args[0].data)
                self.assertEqual(status["reason"], expected)
                if status["accepted"]:
                    output = fake.publisher.publish.call_args.args[0]
                    self.assertEqual(output.header, source.header)
                    self.assertEqual(output.pose, source.pose)
                replayed += 1
            self.assertEqual(replayed, 14)
            self.assertEqual(fake.publisher.publish.call_count, 4)
            self.assertEqual(fake.rejected_count, 10)
            self.assertTrue(fake.gate.clock_reset_required)

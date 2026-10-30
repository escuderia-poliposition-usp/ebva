"""Exercise the runtime probe with real DDS, without drivers or a recorder."""
import time
import unittest
import uuid

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import String

from ebva_recording.session import GraphProbe


class RecordingDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.topic = '/recording_preflight_test_' + uuid.uuid4().hex
        self.config = {'topics': [self.topic]}

    def test_missing_topic_is_reported_without_using_the_default_context(self):
        self.assertFalse(rclpy.ok())
        probe = GraphProbe(self.config)
        try:
            self.assertEqual(probe.poll(0.1), [self.topic])
        finally:
            probe.close()
        self.assertFalse(rclpy.ok())

    def test_publisher_appearing_after_the_initial_poll_is_discovered(self):
        context = Context()
        rclpy.init(args=[], context=context)
        node = Node('delayed_recording_test_publisher', context=context,
                    enable_rosout=False, start_parameter_services=False)
        probe = GraphProbe(self.config)
        try:
            self.assertEqual(probe.poll(0.1), [self.topic])
            publisher = node.create_publisher(String, self.topic, 10)
            deadline = time.monotonic() + 3.0
            missing = [self.topic]
            while missing and time.monotonic() < deadline:
                missing = probe.poll(0.1)
            self.assertEqual(missing, [])
            self.assertFalse(rclpy.ok())
        finally:
            probe.close()
            node.destroy_node()
            context.try_shutdown()

"""Real SQLite bag with synthetic messages and no vehicle/hardware access."""
import os
from pathlib import Path
import tempfile
import unittest
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.events import Shutdown

from ebva_recording.configuration import load_preset
from ebva_recording.session import RecordingSession


@unittest.skipUnless(os.environ.get('CANDIDATES_INSTALLED') == '1', 'requires the candidate installed in an isolated workspace')
class RecordingBagTests(unittest.TestCase):
    def test_synthetic_topic_and_metrics_are_written_and_bag_is_closed(self):
        self.record_and_read(duration=4.0, shutdown_after=12.0)

    def test_parent_shutdown_closes_an_active_bag(self):
        self.record_and_read(duration=0.0, shutdown_after=6.0)

    def record_and_read(self, duration, shutdown_after):
        import rosbag2_py
        import json
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            preset = root / 'preset.json'
            preset.write_text(json.dumps({'version': 2, 'topics': ['/candidate_input'],
                'output_root': str(root / 'bags'), 'duration_s': duration,
                'interval_s': 0.1, 'discovery_timeout_s': 10.0, 'min_free_mib': 1}))
            session = RecordingSession(load_preset(preset), standalone=False)
            publisher = ExecuteProcess(cmd=[sys.executable, '-c', '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(args=[])
node = Node('candidate_publisher')
pub = node.create_publisher(String, '/candidate_input', 10)
timer = node.create_timer(0.05, lambda: pub.publish(String(data='synthetic')))
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
'''])
            service = LaunchService()
            service.include_launch_description(LaunchDescription([
                RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=session.shutdown)])),
                publisher,
                TimerAction(period=0.1, actions=[OpaqueFunction(function=session.poll)]),
                # Include recorder/DDS startup; 1.5 s can end before subscriptions
                # are established on a loaded Docker host, producing an empty bag.
                TimerAction(period=shutdown_after, actions=[EmitEvent(event=Shutdown(reason='synthetic test finished'))]),
            ]))
            service.run()
            bags = list((root / 'bags').glob('*/metadata.yaml'))
            self.assertEqual(len(bags), 1)
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bags[0].parent), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            counts = {}
            while reader.has_next():
                topic, _, _ = reader.read_next()
                counts[topic] = counts.get(topic, 0) + 1
            self.assertGreater(counts.get('/candidate_input', 0), 0)
            self.assertGreater(counts.get('/system/resource_usage', 0), 0)

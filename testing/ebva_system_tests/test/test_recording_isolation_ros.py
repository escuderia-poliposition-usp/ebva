"""Exercise real launch events with harmless child processes, no drivers."""
from pathlib import Path
import sys
import tempfile
import unittest

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


from ebva_recording.session import RecordingSession, _recording_actions


class RecordingIsolationTests(unittest.TestCase):
    def exercise(self, fail):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "vehicle-completed"
            session = RecordingSession({"discovery_timeout_s": 5}, standalone=False)
            session.recorder = ExecuteProcess(cmd=[sys.executable, '-c',
                'import time; time.sleep(0.15); raise SystemExit(7)' if fail else 'import time; time.sleep(5)'])
            session.monitor = ExecuteProcess(cmd=[sys.executable, '-c', 'import time; time.sleep(5)'])
            session.started = True
            _recording_actions.update((session.recorder, session.monitor))
            vehicle = ExecuteProcess(cmd=[sys.executable, '-c',
                'import pathlib,time; time.sleep(0.7); pathlib.Path(' + repr(str(marker)) + ').touch()'])
            service = LaunchService()
            actions = []
            actions.extend(RegisterEventHandler(OnProcessExit(target_action=action, on_exit=session.process_exit))
                           for action in (session.recorder, session.monitor))
            actions.extend([vehicle, session.recorder, session.monitor])
            if not fail:
                actions.append(TimerAction(period=0.15, actions=[OpaqueFunction(function=lambda context: session.stop('duration'))]))
            service.include_launch_description(LaunchDescription(actions))
            service.run()
            self.assertTrue(marker.exists(), 'Recording termination killed the unrelated process')

    def test_duration_stops_only_recording(self):
        self.exercise(False)

    def test_recorder_failure_preserves_vehicle(self):
        self.exercise(True)

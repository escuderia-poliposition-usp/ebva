"""Actual installed rosbag2 player and simulated-clock campaign."""
from pathlib import Path
import tempfile
import unittest

from ebva_system_tests.imu_prior_replay import run_campaign


class ImuPriorReplayTests(unittest.TestCase):
    def test_real_bag_player_uses_original_stamps_and_recovers_from_silence(self):
        with tempfile.TemporaryDirectory(prefix="imu-prior-replay-") as folder:
            result = run_campaign(Path(folder)/"campaign")
            self.assertTrue(result["passed"], result)
            self.assertEqual(result["output_count"], 6)
            self.assertEqual(result["status_count"], 20)
            self.assertTrue(result["clean_process_shutdown"])
            self.assertFalse(result["physical_qualification"])

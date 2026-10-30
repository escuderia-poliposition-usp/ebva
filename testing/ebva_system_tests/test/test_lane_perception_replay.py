"""Real rosbag2 player exercises installed ROS image/lanes edges without devices."""
from pathlib import Path
import tempfile
from ebva_system_tests.lane_perception_replay import run_campaign


def test_rosbag_player_preserves_measurement_headers_and_no_coasting():
    with tempfile.TemporaryDirectory(prefix="lane-perception-replay-") as folder:
        result = run_campaign(Path(folder)/"campaign")
        assert result["passed"], result.get("error", result)
        assert result["output_count"] == 15 and result["status_count"] == 30
        assert result["clean_process_shutdown"]
        assert result["physical_qualification"] is False

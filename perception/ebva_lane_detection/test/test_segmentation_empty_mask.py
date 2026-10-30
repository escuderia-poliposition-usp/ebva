"""Regression coverage for empty road masks and headless frame processing."""

import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault("MPLBACKEND", "Agg")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np
import pytest

from ebva_lane_detection import lane_segmentation as lane


@pytest.mark.parametrize("active_pixels", [0, 1, 10, 600])
def test_overlay_handles_empty_and_nonempty_masks(active_pixels):
    rng = np.random.default_rng(42)
    bgr = rng.integers(0, 256, (20, 30, 3), dtype=np.uint8)
    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    mask = np.zeros(hls.shape[:2], dtype=np.uint8)
    mask.flat[:active_pixels] = 255
    original_hls = hls.copy()
    original_mask = mask.copy()
    gray = cv2.cvtColor(cv2.cvtColor(hls, cv2.COLOR_HLS2BGR), cv2.COLOR_BGR2GRAY)
    expected = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    tint = np.full_like(expected, (255, 0, 0))
    blended = cv2.addWeighted(expected, 0.4, tint, 0.6, 0)
    expected[mask > 0] = blended[mask > 0]

    actual = lane.overlay_on_mask(hls, mask)

    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(hls, original_hls)
    np.testing.assert_array_equal(mask, original_mask)


@pytest.mark.parametrize("intensity", [0, 128, 255])
def test_process_frame_without_lane_pixels(intensity):
    frame = np.full((360, 640, 3), intensity, dtype=np.uint8)
    cfg = lane.LaneConfig()

    result = lane.process_frame(frame, cfg, return_uv=True)

    assert len(result) == 8
    debug, left, right, _, _, binary, birdseye, overlay = result
    assert left == []
    assert right == []
    assert binary.shape == frame.shape[:2]
    assert np.count_nonzero(binary) == 0
    assert debug.shape == (cfg.warped_height, cfg.warped_width, 3)
    assert birdseye.shape == debug.shape
    assert overlay.shape == frame.shape
    assert overlay.dtype == np.uint8


def test_default_pipeline_never_calls_show(monkeypatch):
    import matplotlib.pyplot as plt

    def unexpected_show(*args, **kwargs):
        pytest.fail("Default frame processing must not open blocking plot windows")

    monkeypatch.setattr(plt, "show", unexpected_show)
    try:
        lane.binarize(np.zeros((360, 640, 3), dtype=np.uint8), lane.LaneConfig())
    finally:
        plt.close("all")


def test_debug_plots_are_explicitly_enabled(monkeypatch):
    import matplotlib.pyplot as plt

    calls = []
    monkeypatch.setattr(plt, "show", lambda: calls.append(True))
    cfg = lane.LaneConfig(show_debug_plots=True)
    try:
        binary = lane.binarize(np.full((360, 640, 3), 128, dtype=np.uint8), cfg)
        assert binary.shape == (360, 640)
        assert len(calls) == 3
    finally:
        plt.close("all")


def test_default_pipeline_does_not_import_matplotlib():
    package_root = str(Path(lane.__file__).resolve().parents[1])
    script = (
        "import sys; "
        f"sys.path.insert(0, {package_root!r}); "
        "import numpy as np; "
        "from ebva_lane_detection.lane_segmentation import LaneConfig, process_frame; "
        "process_frame(np.full((360,640,3),128,dtype=np.uint8), LaneConfig()); "
        "assert not any(k == 'matplotlib' or k.startswith('matplotlib.') for k in sys.modules)"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_synthetic_scene_processes_without_plotting():
    cfg = lane.LaneConfig()
    frame = lane.generate_synthetic_frame(cfg, input_size=(360, 640))
    result = lane.process_frame(frame, cfg, return_uv=True)
    assert len(result) == 8
    assert result[-1].shape == frame.shape

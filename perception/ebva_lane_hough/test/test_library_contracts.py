import numpy as np
from ebva_lane_hough.hough_canny import LaneDetector as Canny
from ebva_lane_hough.hough_gradient import LaneDetector as Gradient


def test_canny_reports_no_detection_on_an_empty_frame():
    frame = np.zeros((160, 320, 3), dtype=np.uint8)
    detected, lateral, heading, debug = Canny().process(frame)
    assert not detected
    assert debug.shape == frame.shape
    assert not np.shares_memory(debug, frame)


def test_gradient_retains_its_explicit_confidence_contract():
    frame = np.zeros((160, 320, 3), dtype=np.uint8)
    detected, lateral, heading, confidence, debug = Gradient().process(frame)
    assert not detected
    assert 0 <= confidence <= 1
    assert debug.shape == frame.shape

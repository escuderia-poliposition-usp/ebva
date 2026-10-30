import json
import math

import pytest

from ebva_reference_tools.extract_path import _quat_to_yaw, _resample, _smooth
from ebva_reference_tools.publish_path import load_waypoints
from ebva_reference_tools.output_paths import output_path


def test_resampling_unwraps_yaw_across_pi():
    points = _resample([(0., 0., 3.1), (1., 0., -3.1)], .5)
    assert len(points) == 2
    assert points[1][0] == .5
    assert abs(abs(points[1][2]) - math.pi) < 1e-12
    assert _smooth(points, 0) == points


def test_waypoint_loading_requires_finite_complete_reference(tmp_path):
    path = tmp_path / "reference.json"
    path.write_text(json.dumps([{"x": 1., "y": 2., "yaw": .5}]))
    assert load_waypoints(path) == [(1., 2., .5)]
    path.write_text('[{"x": 1, "y": 2, "yaw": NaN}]')
    with pytest.raises(ValueError):
        load_waypoints(path)
    with pytest.raises(ValueError):
        load_waypoints("")


def test_collection_output_never_reuses_existing_file(tmp_path):
    output = tmp_path / "collected.csv"
    output.write_text("original")
    with pytest.raises(FileExistsError):
        output_path(str(output))
    assert output.read_text() == "original"
    with pytest.raises(ValueError):
        output_path("")

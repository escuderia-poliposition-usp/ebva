import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from ebva_path_control import waypoint_curvature as variant


def test_curvature_variant_retains_its_numerical_behavior(tmp_path):
    data = json.loads((Path(__file__).parent / "data/waypoint-curvature-regression.json").read_text())
    path = tmp_path / "waypoints.csv"
    np.savetxt(path, data["waypoints"], delimiter=",")
    config = SimpleNamespace(wpt_path=str(path), wpt_delim=",", wpt_rowskip=0,
                             wpt_xind=0, wpt_yind=1)
    for case in data["cases"]:
        planner = variant.PurePursuitPlanner(config, data["wheelbase"])
        result = planner.plan(*case["pose"], data["lookahead_distance"],
                              data["velocity_gain"], case["curvature"])
        np.testing.assert_allclose(result, case["expected"], rtol=1e-12, atol=1e-12)
    assert variant.calculate_average_curvature_simple(
        np.array(data["waypoints"]), 2, 5) == pytest.approx(data["average_curvature"])


def test_relative_reference_paths_resolve_against_the_configuration(tmp_path):
    config = SimpleNamespace(wpt_path="waypoints.csv", wpt_rowskip="0")
    variant._normalize_config_paths_and_params(config, str(tmp_path))
    assert config.wpt_path == str(tmp_path / "waypoints.csv")
    assert config.wpt_rowskip == 0

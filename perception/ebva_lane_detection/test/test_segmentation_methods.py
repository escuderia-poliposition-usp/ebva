"""Method preservation and directed behavior; no mask is treated as ground truth."""
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from ebva_lane_detection import lane_segmentation as lane
from ebva_lane_detection.segmentation_methods import METHODS, config_for_method, validate_config
from ebva_lane_detection.comparison import synthetic_cases, compare_frame


def evidence():
    return json.loads((Path(__file__).parent/"data/segmentation-regression.json").read_text())


def test_default_is_preserved_v4_80():
    assert lane.LaneConfig() == config_for_method()
    assert config_for_method().road_l_max == 80
    assert config_for_method().yellow_s_min == 50
    assert config_for_method().sobel_thresh == (10, 150)


def test_v3_source_specific_defaults_and_explicit_overrides():
    cfg = config_for_method("color_gradient_v3")
    assert cfg.yellow_s_min == 80 and cfg.sobel_thresh == (30, 150)
    assert config_for_method("color_gradient_v3", yellow_s_min=60).yellow_s_min == 60
    assert config_for_method("road_mask_v4_50").road_l_max == 50
    with pytest.raises(ValueError): config_for_method("not_a_method")




@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("case_index", range(5))
def test_directed_outputs_match_regression_reference(method, case_index):
    name, frame = list(synthetic_cases())[case_index]
    reference = evidence()["synthetic"][case_index]
    assert reference["case"] == name
    before = frame.copy()
    output = lane.process_frame(frame, config_for_method(method), return_uv=True)
    expected = reference["methods"][method]
    assert int(np.count_nonzero(output[5])) == expected["mask_pixels"]
    assert hashlib.sha256(output[5].tobytes()).hexdigest() == expected["mask_sha256"]
    np.testing.assert_allclose(np.asarray(output[1]).reshape(-1,2), np.asarray(expected["left_xy_m"]).reshape(-1,2),atol=1e-7,rtol=0)
    np.testing.assert_allclose(np.asarray(output[2]).reshape(-1,2), np.asarray(expected["right_xy_m"]).reshape(-1,2),atol=1e-7,rtol=0)
    np.testing.assert_array_equal(frame, before)


@pytest.mark.parametrize("intensity,expected", [(40,(30,30,30)),(65,(30,0,30)),(100,(0,0,30))])
def test_road_threshold_and_v3_have_distinct_valid_semantics(intensity, expected):
    frame = dict(synthetic_cases())["straight_road_l"+str(intensity)]
    counts = []
    for method in METHODS:
        output = lane.process_frame(frame,config_for_method(method),return_uv=True)
        counts.append(len(output[1])+len(output[2]))
    assert tuple(counts) == expected


@pytest.mark.parametrize("change", [
    {"road_l_max":-1}, {"road_l_max":256}, {"warped_width":0}, {"n_output_points":0},
    {"real_lane_width_m":0.}, {"real_forward_span_m":float("nan")},
    {"src_top_y_frac":1.1}, {"dst_margin_frac":.5}, {"sobel_kernel":2},
    {"hist_search_height_frac":0.}, {"morph_close_kernel":(0,15)}, {"sobel_thresh":(150,10)},
])
def test_invalid_numeric_configuration_rejected_before_callbacks(change):
    with pytest.raises(ValueError): validate_config(config_for_method(**change))


def test_comparison_explicitly_does_not_claim_accuracy():
    row = compare_frame("empty",np.full((32,48,3),128,np.uint8))
    assert row["ground_truth_available"] is False
    assert set(row["methods"]) == set(METHODS)
    assert all(v["points_finite"] for v in row["methods"].values())

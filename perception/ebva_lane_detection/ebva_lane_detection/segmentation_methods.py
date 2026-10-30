"""Selection and effective defaults for preserved lane segmentation methods."""
from dataclasses import replace
import math
import numbers
from .lane_segmentation import LaneConfig

METHODS = ("road_mask_v4_80", "road_mask_v4_50", "color_gradient_v3")


def config_for_method(method="road_mask_v4_80", **overrides):
    """Return source-specific defaults; overrides are explicit experiment parameters."""
    if method not in METHODS:
        raise ValueError("unknown segmentation_method: " + str(method))
    cfg = LaneConfig(segmentation_method=method)
    if method == "road_mask_v4_50":
        cfg = replace(cfg, road_l_max=50)
    elif method == "color_gradient_v3":
        cfg = replace(cfg, yellow_s_min=80, sobel_thresh=(30, 150))
    return replace(cfg, **overrides)


def validate_config(cfg):
    """Validate numerical preconditions; this does not establish camera calibration."""
    if cfg.segmentation_method not in METHODS:
        raise ValueError("unknown segmentation_method")
    for name in ("warped_width", "warped_height", "n_output_points", "crawl_step_px",
                 "crawl_window_half_size", "crawl_max_steps", "crawl_max_misses",
                 "min_candidate_points", "hist_min_peak_distance_px", "min_component_area"):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError("invalid positive integer: " + name)
    for name in ("road_l_max", "white_l_min", "yellow_s_min"):
        value = getattr(cfg, name)
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
            raise ValueError("invalid HLS threshold: " + name)
    if cfg.sobel_kernel not in (1, 3, 5, 7):
        raise ValueError("invalid sobel_kernel")
    for name in ("real_lane_width_m", "real_forward_span_m", "camera_to_roi_bottom_m",
                 "src_top_y_frac", "src_bottom_y_frac", "src_top_left_x_frac",
                 "src_top_right_x_frac", "src_bottom_left_x_frac", "src_bottom_right_x_frac",
                 "dst_margin_frac", "hist_search_height_frac", "min_elongation",
                 "crawl_direction_smoothing", "crawl_min_progress_frac"):
        value = getattr(cfg, name)
        if not isinstance(value, numbers.Real) or not math.isfinite(value):
            raise ValueError("nonfinite configuration: " + name)
    if min(cfg.real_lane_width_m, cfg.real_forward_span_m) <= 0:
        raise ValueError("nonpositive assumed metric scale")
    if not 0 < cfg.dst_margin_frac < .5 or not 0 < cfg.hist_search_height_frac <= 1:
        raise ValueError("invalid destination/search fraction")
    if not (cfg.src_top_y_frac < cfg.src_bottom_y_frac
            and cfg.src_top_left_x_frac < cfg.src_top_right_x_frac
            and cfg.src_bottom_left_x_frac < cfg.src_bottom_right_x_frac):
        raise ValueError("degenerate perspective trapezoid")
    for name, maximum in (("sobel_thresh", 255), ("yellow_h_range", 179)):
        value = getattr(cfg, name)
        if len(value) != 2 or any(isinstance(v, bool) or not isinstance(v, int) for v in value):
            raise ValueError("invalid threshold pair: " + name)
        if not 0 <= value[0] <= value[1] <= maximum:
            raise ValueError("invalid threshold range: " + name)
    if len(cfg.morph_close_kernel) != 2 or any(
            isinstance(v, bool) or not isinstance(v, int) or v < 1 for v in cfg.morph_close_kernel):
        raise ValueError("invalid morph_close_kernel")
    return cfg

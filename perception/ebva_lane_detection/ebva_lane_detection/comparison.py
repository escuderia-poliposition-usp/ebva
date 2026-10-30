"""Directed deterministic comparisons; masks/coordinates are observations, not GT."""
import hashlib
import cv2
import numpy as np
from .segmentation_methods import METHODS, config_for_method
from .lane_segmentation import process_frame, generate_synthetic_frame


def synthetic_cases():
    """Shared camera pixels generated once; each method receives identical inputs."""
    cfg = config_for_method()
    yield "empty_gray", np.full((180, 320, 3), 128, np.uint8)
    yield "shared_curve_seed0", generate_synthetic_frame(cfg, input_size=(360, 640))
    for luminance in (40, 65, 100):
        frame = np.full((180, 320, 3), luminance, np.uint8)
        cv2.line(frame, (105, 179), (145, 90), (230, 230, 230), 6)
        cv2.line(frame, (215, 179), (175, 90), (230, 230, 230), 6)
        yield "straight_road_l"+str(luminance), frame


def describe_frame(frame):
    return {"shape": list(frame.shape), "dtype": str(frame.dtype),
            "pixels_sha256": hashlib.sha256(frame.tobytes()).hexdigest()}


def compare_frame(name, frame):
    row = {"case": name, "input": describe_frame(frame), "methods": {}}
    masks = {}
    for method in METHODS:
        cfg = config_for_method(method)
        output = process_frame(frame, cfg, return_uv=True)
        mask = output[5]; masks[method] = mask
        row["methods"][method] = {
            "mask_pixels": int(np.count_nonzero(mask)),
            "mask_sha256": hashlib.sha256(mask.tobytes()).hexdigest(),
            "left_xy_m": output[1], "right_xy_m": output[2],
            "points_finite": bool(all(np.isfinite(v) for pts in output[1:3] for p in pts for v in p))}
    row["pairwise_different_pixels"] = {
        a+"__"+b: int(np.count_nonzero(masks[a] != masks[b]))
        for i, a in enumerate(METHODS) for b in METHODS[i+1:]}
    row["ground_truth_available"] = False
    return row

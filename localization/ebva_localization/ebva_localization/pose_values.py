"""Pose data validation independent of ROS and any estimator."""
import math


def pose_covariance(values=None):
    """Return a finite 6x6 row-major covariance, with default diagonal variances."""
    result = [0.5 if i % 7 == 0 else 0.0 for i in range(36)] if values is None else [float(v) for v in values]
    if len(result) != 36 or not all(math.isfinite(v) for v in result):
        raise ValueError("covariance must have 36 finite entries")
    return result

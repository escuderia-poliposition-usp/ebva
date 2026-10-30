from ebva_path_control.resource_paths import resolve_reference
"""Validated Frenet references; no implicit conversion between CSV schemas."""
import csv
import numpy as np


def load_frenet_reference(path):
    path = resolve_reference(path)
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"s", "e_y"}.issubset(reader.fieldnames or []):
            raise ValueError("Frenet CSV requires named s [m] and e_y [m] columns")
        values = np.asarray([(float(r["s"]), float(r["e_y"])) for r in reader], dtype=float)
    if values.shape[0] < 2 or values.ndim != 2 or not np.isfinite(values).all():
        raise ValueError("Frenet reference requires at least two finite samples")
    s, ey = values[:, 0], values[:, 1]
    if s[0] < 0 or s[-1] <= 0 or not np.all(np.diff(s) > 0):
        raise ValueError("Frenet s must be nonnegative and strictly increasing")
    return s, ey

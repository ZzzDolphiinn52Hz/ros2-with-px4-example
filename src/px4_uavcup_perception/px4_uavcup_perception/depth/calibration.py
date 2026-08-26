"""Calibration transforms for monocular metric-depth output."""

import math

import numpy as np


def apply_linear_depth_calibration(
        depth_m: np.ndarray,
        scale: float,
        bias_m: float) -> np.ndarray:
    """Apply ``scale * raw + bias`` while preserving invalid pixels."""
    scale = float(scale)
    bias_m = float(bias_m)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError('Depth calibration scale must be finite and positive')
    if not math.isfinite(bias_m):
        raise ValueError('Depth calibration bias must be finite')

    calibrated = np.asarray(depth_m, dtype=np.float32).copy()
    valid = np.isfinite(calibrated)
    calibrated[valid] = calibrated[valid] * scale + bias_m
    return calibrated

"""Fit metric inverse-depth alignment from measured ZipDepth samples."""

from __future__ import annotations

import math
from typing import Iterable, Mapping

import numpy as np


def central_roi_median(
        raw_inverse_depth: np.ndarray,
        roi_fraction: float = 0.25) -> float:
    """Return the finite median inside a centred rectangular ROI."""
    raw = np.asarray(raw_inverse_depth, dtype=np.float32)
    if raw.ndim != 2 or raw.size == 0:
        raise ValueError('raw inverse depth must be a non-empty 2D array')
    if not 0.0 < roi_fraction <= 1.0:
        raise ValueError('roi_fraction must be in (0, 1]')
    height, width = raw.shape
    roi_height = max(1, int(round(height * roi_fraction)))
    roi_width = max(1, int(round(width * roi_fraction)))
    top = (height - roi_height) // 2
    left = (width - roi_width) // 2
    roi = raw[top:top + roi_height, left:left + roi_width]
    values = roi[np.isfinite(roi)]
    if values.size == 0:
        raise ValueError('central ROI contains no finite inverse depth')
    return float(np.median(values))


def fit_metric_inverse_depth(
        records: Iterable[Mapping[str, float]]) -> dict:
    """Fit ``1/distance = scale * raw + shift`` by least squares."""
    samples = list(records)
    if len(samples) < 3:
        raise ValueError('at least three calibration samples are required')
    distances = np.asarray(
        [float(sample['distance_m']) for sample in samples],
        dtype=np.float64)
    raw = np.asarray(
        [float(sample['raw_inverse_depth_median']) for sample in samples],
        dtype=np.float64)
    if not np.all(np.isfinite(distances)) or np.any(distances <= 0.0):
        raise ValueError('calibration distances must be finite and positive')
    if not np.all(np.isfinite(raw)):
        raise ValueError('raw inverse-depth samples must be finite')
    if np.unique(distances).size < 3:
        raise ValueError('use at least three distinct calibration distances')
    if float(np.ptp(raw)) <= 1e-6:
        raise ValueError('raw inverse-depth samples have insufficient spread')

    target_inverse_m = np.reciprocal(distances)
    design = np.column_stack((raw, np.ones_like(raw)))
    scale, shift = np.linalg.lstsq(
        design, target_inverse_m, rcond=None)[0]
    if not math.isfinite(float(scale)) or scale <= 0.0:
        raise ValueError('fitted inverse-depth scale is not positive')
    calibrated_inverse_m = raw * scale + shift
    if np.any(calibrated_inverse_m <= 0.0):
        raise ValueError('fit predicts non-positive inverse depth')
    predicted_distance_m = np.reciprocal(calibrated_inverse_m)
    inverse_residual = calibrated_inverse_m - target_inverse_m
    distance_residual = predicted_distance_m - distances
    return {
        'inverse_depth_scale': float(scale),
        'inverse_depth_shift_per_m': float(shift),
        'inverse_depth_rmse_per_m': float(np.sqrt(np.mean(
            np.square(inverse_residual)))),
        'distance_rmse_m': float(np.sqrt(np.mean(
            np.square(distance_residual)))),
        'samples': [
            {
                'distance_m': float(distance),
                'raw_inverse_depth_median': float(raw_value),
                'predicted_distance_m': float(predicted),
                'error_m': float(error),
            }
            for distance, raw_value, predicted, error in zip(
                distances, raw, predicted_distance_m, distance_residual)
        ],
    }

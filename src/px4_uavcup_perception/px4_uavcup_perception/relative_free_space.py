"""Affine-invariant three-sector clearance for relative inverse depth."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass(frozen=True)
class RelativeFreeSpaceSummary:
    left: float
    center: float
    right: float
    nearest: float
    valid_fraction: float
    contrast_span: float

    def as_list(self) -> List[float]:
        return [
            self.left,
            self.center,
            self.right,
            self.nearest,
            self.valid_fraction,
        ]


def summarize_relative_free_space(
        raw_inverse_depth: np.ndarray,
        roi_top_fraction: float = 0.25,
        roi_bottom_fraction: float = 0.78,
        normalization_low_percentile: float = 2.0,
        normalization_high_percentile: float = 98.0,
        near_percentile: float = 85.0,
        minimum_contrast_span: float = 1e-3,
        ) -> RelativeFreeSpaceSummary:
    """Return affine-invariant clearance scores in [0, 1]."""
    raw = np.asarray(raw_inverse_depth, dtype=np.float32)
    if raw.ndim != 2 or raw.size == 0:
        raise ValueError('inverse depth must be a non-empty 2D array')
    if not 0.0 <= roi_top_fraction < roi_bottom_fraction <= 1.0:
        raise ValueError('ROI fractions must satisfy 0 <= top < bottom <= 1')
    if not (0.0 <= normalization_low_percentile
            < normalization_high_percentile <= 100.0):
        raise ValueError('normalization percentiles are invalid')
    if not 0.0 <= near_percentile <= 100.0:
        raise ValueError('near_percentile must be in [0, 100]')
    if minimum_contrast_span <= 0.0:
        raise ValueError('minimum_contrast_span must be positive')

    height, width = raw.shape
    row_start = min(height - 1, int(round(height * roi_top_fraction)))
    row_stop = min(
        height,
        max(row_start + 1, int(round(height * roi_bottom_fraction))),
    )
    roi = raw[row_start:row_stop, :]
    finite = np.isfinite(roi)
    finite_values = roi[finite]
    valid_fraction = float(np.count_nonzero(finite) / finite.size)
    if finite_values.size == 0:
        return _invalid(valid_fraction, float('nan'))
    low, high = np.percentile(finite_values, [
        normalization_low_percentile,
        normalization_high_percentile,
    ])
    span = float(high - low)
    if not np.isfinite(span) or span < minimum_contrast_span:
        return _invalid(valid_fraction, span)
    normalized_inverse = np.full(roi.shape, np.nan, dtype=np.float32)
    normalized_inverse[finite] = np.clip(
        (roi[finite] - float(low)) / span, 0.0, 1.0)

    bounds = ((0.00, 0.40), (0.30, 0.70), (0.60, 1.00))

    def sector_clearance(start_fraction: float, stop_fraction: float) -> float:
        start = min(width - 1, int(round(width * start_fraction)))
        stop = min(width, max(start + 1, int(round(width * stop_fraction))))
        values = normalized_inverse[:, start:stop]
        values = values[np.isfinite(values)]
        if values.size == 0:
            return float('nan')
        near_score = float(np.percentile(values, near_percentile))
        return 1.0 - near_score

    left, center, right = (sector_clearance(*bound) for bound in bounds)
    clearances = [value for value in (left, center, right)
                  if np.isfinite(value)]
    nearest = min(clearances) if clearances else float('nan')
    return RelativeFreeSpaceSummary(
        left, center, right, nearest, valid_fraction, span)


def _invalid(valid_fraction: float, span: float) -> RelativeFreeSpaceSummary:
    return RelativeFreeSpaceSummary(
        float('nan'), float('nan'), float('nan'), float('nan'),
        valid_fraction, span)

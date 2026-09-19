"""Find a large, relatively distant 2D corridor in inverse depth."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List

import numpy as np


@dataclass(frozen=True)
class RelativeCorridor:
    """Best fixed-size image corridor and its conservative clearance."""

    x_normalized: float
    y_normalized: float
    width_fraction: float
    height_fraction: float
    clearance: float
    score: float
    valid_fraction: float
    left_px: int = 0
    top_px: int = 0
    right_px: int = 0
    bottom_px: int = 0

    @property
    def valid(self) -> bool:
        return all(math.isfinite(value) for value in (
            self.x_normalized,
            self.y_normalized,
            self.clearance,
            self.score,
        ))

    def as_list(self) -> List[float]:
        return [
            self.x_normalized,
            self.y_normalized,
            self.width_fraction,
            self.height_fraction,
            self.clearance,
            self.score,
            self.valid_fraction,
        ]


def find_relative_corridor(
        raw_inverse_depth: np.ndarray,
        roi_top_fraction: float = 0.08,
        roi_bottom_fraction: float = 0.92,
        window_width_fraction: float = 0.32,
        window_height_fraction: float = 0.32,
        stride_fraction: float = 0.04,
        clearance_percentile: float = 20.0,
        minimum_clearance: float = 0.10,
        minimum_valid_fraction: float = 0.90,
        centre_bias: float = 0.12,
        normalization_low_percentile: float = 2.0,
        normalization_high_percentile: float = 98.0,
        minimum_contrast_span: float = 1e-3,
        ) -> RelativeCorridor:
    """Return the safest fixed-size window in normalized image coordinates.

    ZipDepth produces affine-invariant inverse depth, so this routine ranks
    candidate windows within one frame. It does not claim metric clearance.
    """
    raw = np.asarray(raw_inverse_depth, dtype=np.float32)
    if raw.ndim != 2 or raw.size == 0:
        raise ValueError('inverse depth must be a non-empty 2D array')
    if not 0.0 <= roi_top_fraction < roi_bottom_fraction <= 1.0:
        raise ValueError('ROI fractions must satisfy 0 <= top < bottom <= 1')
    for name, value in (
            ('window_width_fraction', window_width_fraction),
            ('window_height_fraction', window_height_fraction),
            ('stride_fraction', stride_fraction)):
        if not 0.0 < value <= 1.0:
            raise ValueError(f'{name} must be in (0, 1]')
    if not 0.0 <= clearance_percentile <= 100.0:
        raise ValueError('clearance_percentile must be in [0, 100]')
    if not 0.0 <= minimum_clearance <= 1.0:
        raise ValueError('minimum_clearance must be in [0, 1]')
    if not 0.0 <= minimum_valid_fraction <= 1.0:
        raise ValueError('minimum_valid_fraction must be in [0, 1]')
    if centre_bias < 0.0 or not math.isfinite(centre_bias):
        raise ValueError('centre_bias must be finite and non-negative')
    if not (0.0 <= normalization_low_percentile
            < normalization_high_percentile <= 100.0):
        raise ValueError('normalization percentiles are invalid')
    if minimum_contrast_span <= 0.0:
        raise ValueError('minimum_contrast_span must be positive')

    height, width = raw.shape
    roi_top = min(height - 1, int(round(height * roi_top_fraction)))
    roi_bottom = min(
        height,
        max(roi_top + 1, int(round(height * roi_bottom_fraction))),
    )
    finite = np.isfinite(raw[roi_top:roi_bottom, :])
    finite_values = raw[roi_top:roi_bottom, :][finite]
    overall_valid = float(np.count_nonzero(finite) / finite.size)
    if finite_values.size == 0:
        return _invalid(overall_valid, window_width_fraction,
                        window_height_fraction)
    low, high = np.percentile(finite_values, [
        normalization_low_percentile,
        normalization_high_percentile,
    ])
    span = float(high - low)
    if not np.isfinite(span) or span < minimum_contrast_span:
        return _invalid(overall_valid, window_width_fraction,
                        window_height_fraction)

    clearance_map = np.full(raw.shape, np.nan, dtype=np.float32)
    raw_finite = np.isfinite(raw)
    normalized_inverse = np.clip(
        (raw[raw_finite] - float(low)) / span, 0.0, 1.0)
    clearance_map[raw_finite] = 1.0 - normalized_inverse

    window_width = min(
        width, max(1, int(round(width * window_width_fraction))))
    window_height = min(
        roi_bottom - roi_top,
        max(1, int(round(height * window_height_fraction))))
    stride = max(1, int(round(min(width, height) * stride_fraction)))
    left_positions = _positions(0, width - window_width, stride)
    top_positions = _positions(
        roi_top, roi_bottom - window_height, stride)
    valid_map = np.isfinite(clearance_map)
    clear_map = valid_map & (clearance_map >= minimum_clearance)
    clearance_sum_map = np.where(
        valid_map, clearance_map, 0.0).astype(np.float64)
    valid_integral = _integral(valid_map)
    clear_integral = _integral(clear_map)
    clearance_integral = _integral(clearance_sum_map)
    required_clear_fraction = 1.0 - clearance_percentile / 100.0

    best = None
    best_score = float('-inf')
    for top in top_positions:
        bottom = top + window_height
        for left in left_positions:
            right = left + window_width
            pixel_count = window_width * window_height
            valid_count = _window_sum(
                valid_integral, left, top, right, bottom)
            valid_fraction = float(valid_count / pixel_count)
            if valid_fraction < minimum_valid_fraction:
                continue
            clear_count = _window_sum(
                clear_integral, left, top, right, bottom)
            clear_fraction = float(clear_count / valid_count)
            if clear_fraction < required_clear_fraction:
                continue
            clearance_sum = _window_sum(
                clearance_integral, left, top, right, bottom)
            mean_clearance = float(clearance_sum / valid_count)
            centre_x = left + (window_width - 1) / 2.0
            centre_y = top + (window_height - 1) / 2.0
            x_normalized = _normalize_pixel(centre_x, width)
            y_normalized = _normalize_pixel(centre_y, height)
            centre_distance = math.hypot(
                x_normalized, y_normalized) / math.sqrt(2.0)
            score = (
                0.75 * clear_fraction
                + 0.25 * mean_clearance
                - centre_bias * centre_distance)
            if score > best_score:
                best_score = score
                values = clearance_map[top:bottom, left:right]
                conservative = float(np.percentile(
                    values[np.isfinite(values)], clearance_percentile))
                best = RelativeCorridor(
                    x_normalized=x_normalized,
                    y_normalized=y_normalized,
                    width_fraction=window_width / width,
                    height_fraction=window_height / height,
                    clearance=conservative,
                    score=score,
                    valid_fraction=valid_fraction,
                    left_px=left,
                    top_px=top,
                    right_px=right,
                    bottom_px=bottom,
                )
    if best is None:
        return _invalid(overall_valid, window_width_fraction,
                        window_height_fraction)
    return best


def _positions(start: int, stop: int, stride: int) -> List[int]:
    if stop <= start:
        return [start]
    positions = list(range(start, stop + 1, stride))
    if positions[-1] != stop:
        positions.append(stop)
    return positions


def _normalize_pixel(pixel: float, extent: int) -> float:
    if extent <= 1:
        return 0.0
    return float(2.0 * pixel / (extent - 1) - 1.0)


def _integral(values: np.ndarray) -> np.ndarray:
    data = np.asarray(values, dtype=np.float64)
    return np.pad(
        np.cumsum(np.cumsum(data, axis=0), axis=1),
        ((1, 0), (1, 0)),
        mode='constant')


def _window_sum(
        integral: np.ndarray,
        left: int,
        top: int,
        right: int,
        bottom: int) -> float:
    return float(
        integral[bottom, right]
        - integral[top, right]
        - integral[bottom, left]
        + integral[top, left])


def _invalid(
        valid_fraction: float,
        width_fraction: float,
        height_fraction: float) -> RelativeCorridor:
    return RelativeCorridor(
        x_normalized=float('nan'),
        y_normalized=float('nan'),
        width_fraction=width_fraction,
        height_fraction=height_fraction,
        clearance=float('nan'),
        score=float('nan'),
        valid_fraction=valid_fraction,
    )

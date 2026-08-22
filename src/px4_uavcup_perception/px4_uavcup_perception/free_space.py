"""Pure functions for reducing a depth image to controller-friendly data."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FreeSpaceSummary:
    """Near-depth statistics in metres for three horizontal image sectors."""

    left_m: float
    center_m: float
    right_m: float
    nearest_m: float
    valid_fraction: float

    def as_list(self) -> list[float]:
        return [
            self.left_m,
            self.center_m,
            self.right_m,
            self.nearest_m,
            self.valid_fraction,
        ]


def summarize_free_space(
        depth_m: np.ndarray,
        minimum_depth_m: float = 0.15,
        maximum_depth_m: float = 20.0,
        near_percentile: float = 15.0,
        roi_top_fraction: float = 0.25,
        roi_bottom_fraction: float = 0.85) -> FreeSpaceSummary:
    """Summarize valid depth using robust near percentiles.

    The vertical crop suppresses sky/roof pixels at the top and the floor close
    to the vehicle at the bottom. Horizontal sectors overlap slightly so an
    obstacle on a boundary is visible to both neighbouring sectors.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError(
            'Depth image must be a non-empty two-dimensional array')
    if not 0.0 <= roi_top_fraction < roi_bottom_fraction <= 1.0:
        raise ValueError('ROI fractions must satisfy 0 <= top < bottom <= 1')
    if not 0.0 <= near_percentile <= 100.0:
        raise ValueError('near_percentile must be between 0 and 100')

    height, width = depth.shape
    row_start = min(height - 1, int(round(height * roi_top_fraction)))
    row_stop = max(row_start + 1, int(round(height * roi_bottom_fraction)))
    row_stop = min(height, row_stop)
    roi = depth[row_start:row_stop, :]

    valid = (
        np.isfinite(roi)
        & (roi >= float(minimum_depth_m))
        & (roi <= float(maximum_depth_m))
    )
    valid_fraction = float(np.count_nonzero(valid) / valid.size)

    bounds = (
        (0.00, 0.40),
        (0.30, 0.70),
        (0.60, 1.00),
    )

    def sector_distance(start_fraction: float, stop_fraction: float) -> float:
        start = min(width - 1, int(round(width * start_fraction)))
        stop = min(width, max(start + 1, int(round(width * stop_fraction))))
        sector = roi[:, start:stop]
        sector_valid = valid[:, start:stop]
        values = sector[sector_valid]
        if values.size == 0:
            return float('nan')
        return float(np.percentile(values, near_percentile))

    left, center, right = (sector_distance(*bound) for bound in bounds)
    valid_distances = [value for value in (left, center, right)
                       if np.isfinite(value)]
    nearest = min(valid_distances) if valid_distances else float('nan')
    return FreeSpaceSummary(left, center, right, nearest, valid_fraction)

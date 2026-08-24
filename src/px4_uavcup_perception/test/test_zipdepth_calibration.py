import numpy as np
import pytest

from px4_uavcup_perception.zipdepth_calibration import (
    central_roi_median,
    fit_metric_inverse_depth,
)


def test_central_roi_median_ignores_outer_pixels_and_nonfinite_values():
    raw = np.full((8, 8), 100.0, dtype=np.float32)
    raw[3:5, 3:5] = [[2.0, 3.0], [4.0, np.nan]]

    assert central_roi_median(raw, roi_fraction=0.25) == pytest.approx(3.0)


def test_metric_fit_recovers_inverse_depth_scale_and_shift():
    scale = 0.4
    shift = 0.2
    raw_values = [0.75, 1.25, 2.0, 3.0]
    records = [
        {
            'distance_m': 1.0 / (scale * raw + shift),
            'raw_inverse_depth_median': raw,
        }
        for raw in raw_values
    ]

    result = fit_metric_inverse_depth(records)

    assert result['inverse_depth_scale'] == pytest.approx(scale)
    assert result['inverse_depth_shift_per_m'] == pytest.approx(shift)
    assert result['distance_rmse_m'] == pytest.approx(0.0, abs=1e-10)


def test_metric_fit_requires_three_distinct_distances():
    with pytest.raises(ValueError):
        fit_metric_inverse_depth([
            {'distance_m': 1.0, 'raw_inverse_depth_median': 2.0},
            {'distance_m': 2.0, 'raw_inverse_depth_median': 1.0},
        ])

import math

import numpy as np

from px4_uavcup_perception.depth.free_space import summarize_free_space


def test_three_sectors_report_robust_near_depth():
    depth = np.full((20, 30), 9.0, dtype=np.float32)
    depth[:, :12] = 4.0
    depth[:, 9:21] = 2.0
    depth[:, 18:] = 6.0

    summary = summarize_free_space(
        depth,
        roi_top_fraction=0.0,
        roi_bottom_fraction=1.0,
        near_percentile=15.0,
    )

    assert summary.left_m == 2.0  # overlapping centre is intentionally visible
    assert summary.center_m == 2.0
    assert summary.right_m == 6.0
    assert summary.nearest_m == 2.0
    assert summary.valid_fraction == 1.0


def test_invalid_pixels_are_excluded():
    depth = np.array([
        [np.nan, np.inf, 0.0, 1.0, 2.0, 30.0],
        [np.nan, np.inf, 0.0, 1.0, 2.0, 30.0],
    ], dtype=np.float32)

    summary = summarize_free_space(
        depth,
        minimum_depth_m=0.15,
        maximum_depth_m=20.0,
        roi_top_fraction=0.0,
        roi_bottom_fraction=1.0,
    )

    assert math.isnan(summary.left_m)
    assert math.isfinite(summary.center_m)
    assert math.isfinite(summary.right_m)
    assert summary.valid_fraction == 2.0 / 6.0


def test_rejects_non_image_input():
    try:
        summarize_free_space(np.array([1.0, 2.0], dtype=np.float32))
    except ValueError as error:
        assert 'two-dimensional' in str(error)
    else:
        raise AssertionError('Expected ValueError')

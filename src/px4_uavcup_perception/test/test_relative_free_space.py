import math

import numpy as np

from px4_uavcup_perception.relative_free_space import (
    summarize_relative_free_space,
)


def test_center_foreground_has_less_relative_clearance_than_sides():
    raw = np.zeros((20, 30), dtype=np.float32)
    raw[:, 12:18] = 1.0

    summary = summarize_relative_free_space(
        raw,
        roi_top_fraction=0.0,
        roi_bottom_fraction=1.0,
        near_percentile=85.0,
        minimum_contrast_span=0.01,
    )

    assert summary.center < summary.left
    assert summary.center < summary.right
    assert summary.nearest == summary.center
    assert summary.valid_fraction == 1.0


def test_scores_are_invariant_to_positive_affine_transform():
    raw = np.arange(60, dtype=np.float32).reshape(6, 10)
    transformed = raw * 4.2 - 7.0

    original = summarize_relative_free_space(
        raw, roi_top_fraction=0.0, roi_bottom_fraction=1.0)
    changed = summarize_relative_free_space(
        transformed, roi_top_fraction=0.0, roi_bottom_fraction=1.0)

    np.testing.assert_allclose(
        original.as_list(), changed.as_list(), atol=1e-6)


def test_uniform_scene_is_invalid_instead_of_falsely_clear():
    raw = np.full((20, 30), 0.5, dtype=np.float32)

    summary = summarize_relative_free_space(raw)

    assert math.isnan(summary.center)
    assert summary.valid_fraction == 1.0

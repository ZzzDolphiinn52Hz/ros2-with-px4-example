import math

import numpy as np

from px4_uavcup_perception.depth.corridor import find_relative_corridor


def test_selects_large_far_opening_in_upper_left():
    inverse_depth = np.ones((100, 160), dtype=np.float32)
    inverse_depth[5:55, 5:75] = 0.0

    corridor = find_relative_corridor(
        inverse_depth,
        window_width_fraction=0.30,
        window_height_fraction=0.30,
        stride_fraction=0.02,
        centre_bias=0.02,
    )

    assert corridor.valid
    assert corridor.x_normalized < 0.0
    assert corridor.y_normalized < 0.0
    assert corridor.clearance > 0.9


def test_rejects_opening_smaller_than_required_window():
    inverse_depth = np.ones((100, 160), dtype=np.float32)
    inverse_depth[35:55, 65:95] = 0.0

    corridor = find_relative_corridor(
        inverse_depth,
        window_width_fraction=0.40,
        window_height_fraction=0.40,
        stride_fraction=0.02,
        minimum_clearance=0.5,
    )

    assert not corridor.valid


def test_centre_bias_prefers_centered_opening_when_clearance_matches():
    inverse_depth = np.ones((100, 160), dtype=np.float32)
    inverse_depth[20:80, 0:55] = 0.0
    inverse_depth[20:80, 55:105] = 0.0

    corridor = find_relative_corridor(
        inverse_depth,
        window_width_fraction=0.25,
        window_height_fraction=0.30,
        stride_fraction=0.02,
        centre_bias=0.3,
    )

    assert corridor.valid
    assert abs(corridor.x_normalized) < 0.2


def test_uniform_or_invalid_depth_has_no_corridor():
    uniform = find_relative_corridor(
        np.ones((40, 60), dtype=np.float32))
    invalid = find_relative_corridor(
        np.full((40, 60), np.nan, dtype=np.float32))

    assert not uniform.valid
    assert not invalid.valid
    assert math.isnan(uniform.clearance)

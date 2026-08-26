import math

import numpy as np

from px4_uavcup_perception.depth.calibration import (
    apply_linear_depth_calibration,
)


def test_linear_calibration_applies_scale_and_bias():
    raw = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)

    calibrated = apply_linear_depth_calibration(raw, 0.5, 0.2)

    np.testing.assert_allclose(
        calibrated,
        np.array([[0.7, 1.2], [1.7, 2.2]], dtype=np.float32),
    )
    np.testing.assert_array_equal(raw, [[1.0, 2.0], [3.0, 4.0]])


def test_linear_calibration_preserves_invalid_pixels():
    raw = np.array([[np.nan, np.inf, -np.inf, 2.0]], dtype=np.float32)

    calibrated = apply_linear_depth_calibration(raw, 0.5, 0.2)

    assert math.isnan(float(calibrated[0, 0]))
    assert math.isinf(float(calibrated[0, 1]))
    assert math.isinf(float(calibrated[0, 2]))
    assert np.isclose(calibrated[0, 3], 1.2)


def test_linear_calibration_rejects_non_positive_scale():
    try:
        apply_linear_depth_calibration(np.ones((1, 1)), 0.0, 0.0)
    except ValueError as error:
        assert 'positive' in str(error)
    else:
        raise AssertionError('Expected ValueError')

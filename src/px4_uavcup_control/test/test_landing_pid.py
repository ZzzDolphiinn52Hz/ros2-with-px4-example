import numpy as np
import pytest

from px4_uavcup_control.landing.pid import (
    DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
    PidAxis,
    camera_target_to_body_flu,
)


def test_down_camera_axes_match_forward_and_right_mounting():
    # Marker above the image centre is ahead; marker at image-right is right.
    ahead = camera_target_to_body_flu(
        [0.0, -0.2, 1.0], DOWN_CAMERA_OPTICAL_TO_BODY_FLU)
    right = camera_target_to_body_flu(
        [0.2, 0.0, 1.0], DOWN_CAMERA_OPTICAL_TO_BODY_FLU)
    np.testing.assert_allclose(ahead, [0.2, 0.0, -1.0])
    np.testing.assert_allclose(right, [0.0, -0.2, -1.0])


def test_pid_is_bounded_and_resettable():
    pid = PidAxis(1.0, 1.0, 0.0, integral_limit=0.1, output_limit=0.2)
    assert pid.update(1.0, 1.0) == pytest.approx(0.2)
    assert pid.integral == pytest.approx(0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.previous_error is None


def test_pid_rejects_invalid_dt():
    pid = PidAxis(1.0, 0.0, 0.0, integral_limit=1.0, output_limit=1.0)
    with pytest.raises(ValueError):
        pid.update(1.0, 0.0)

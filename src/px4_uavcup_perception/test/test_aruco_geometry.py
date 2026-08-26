import math

import numpy as np
import pytest

from px4_uavcup_perception.aruco_geometry import (
    camera_matrix,
    rotation_matrix_to_quaternion,
)
from px4_uavcup_perception.px4_bridge_geometry import (
    DOWN_CAMERA_OPTICAL_TO_BODY_FRD,
    quaternion_wxyz_to_matrix,
    target_camera_to_ned,
)


def test_camera_matrix_validates_intrinsics():
    result = camera_matrix([500, 0, 320, 0, 501, 240, 0, 0, 1])
    assert result.shape == (3, 3)
    with pytest.raises(ValueError):
        camera_matrix([0] * 9)


def test_rotation_matrix_identity_quaternion():
    quaternion = rotation_matrix_to_quaternion(np.eye(3))
    assert quaternion == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_quaternion_yaw_rotates_forward_to_east():
    half = math.pi / 4.0
    rotation = quaternion_wxyz_to_matrix(
        [math.cos(half), 0.0, 0.0, math.sin(half)])
    assert rotation @ [1.0, 0.0, 0.0] == pytest.approx([0.0, 1.0, 0.0])


def test_forward_optical_target_becomes_forward_body_and_ned():
    result = target_camera_to_ned(
        [0.0, 0.0, 2.0], [1.0, 0.0, 0.0, 0.0])
    assert result == pytest.approx([2.0, 0.0, 0.0])


def test_camera_translation_is_applied_in_body_frame():
    result = target_camera_to_ned(
        [0.0, 0.0, 2.0], [1.0, 0.0, 0.0, 0.0],
        camera_position_body=[0.1, 0.0, -0.05])
    assert result == pytest.approx([2.1, 0.0, -0.05])


def test_confirmed_down_camera_axes_map_to_body_frd():
    attitude_level = [1.0, 0.0, 0.0, 0.0]
    ahead = target_camera_to_ned(
        [0.0, -0.2, 1.0], attitude_level,
        DOWN_CAMERA_OPTICAL_TO_BODY_FRD)
    right = target_camera_to_ned(
        [0.2, 0.0, 1.0], attitude_level,
        DOWN_CAMERA_OPTICAL_TO_BODY_FRD)
    assert ahead == pytest.approx([0.2, 0.0, 1.0])
    assert right == pytest.approx([0.0, 0.2, 1.0])

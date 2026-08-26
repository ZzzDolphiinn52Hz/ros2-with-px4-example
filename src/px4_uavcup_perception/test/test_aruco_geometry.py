import numpy as np
import pytest

from px4_uavcup_perception.aruco.geometry import (
    camera_matrix,
    rotation_matrix_to_quaternion,
)


def test_camera_matrix_validates_intrinsics():
    result = camera_matrix([500, 0, 320, 0, 501, 240, 0, 0, 1])
    assert result.shape == (3, 3)
    with pytest.raises(ValueError):
        camera_matrix([0] * 9)


def test_rotation_matrix_identity_quaternion():
    quaternion = rotation_matrix_to_quaternion(np.eye(3))
    assert quaternion == pytest.approx([0.0, 0.0, 0.0, 1.0])

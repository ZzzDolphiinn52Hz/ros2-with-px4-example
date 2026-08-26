"""Frame conversions used by the ArUco-to-PX4 landing-target bridge."""

from __future__ import annotations

import numpy as np


OPTICAL_TO_BODY_FRD = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)

DOWN_CAMERA_OPTICAL_TO_BODY_FRD = np.array([
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)


def quaternion_wxyz_to_matrix(values) -> np.ndarray:
    """Convert a Hamilton quaternion [w, x, y, z] to a rotation matrix."""
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError('quaternion must contain four values')
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError('quaternion must be finite and non-zero')
    w, x, y, z = q / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])


def target_camera_to_ned(
        camera_xyz, attitude_wxyz,
        camera_to_body=OPTICAL_TO_BODY_FRD,
        camera_position_body=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Transform target position from camera optical coordinates to NED."""
    camera_xyz = np.asarray(camera_xyz, dtype=np.float64)
    rotation = np.asarray(camera_to_body, dtype=np.float64)
    translation = np.asarray(camera_position_body, dtype=np.float64)
    if camera_xyz.shape != (3,) or translation.shape != (3,):
        raise ValueError('positions must contain three values')
    if rotation.shape != (3, 3):
        raise ValueError('camera_to_body must have shape (3, 3)')
    if not np.all(np.isfinite(camera_xyz)):
        raise ValueError('camera target position must be finite')
    target_body = rotation @ camera_xyz + translation
    return quaternion_wxyz_to_matrix(attitude_wxyz) @ target_body

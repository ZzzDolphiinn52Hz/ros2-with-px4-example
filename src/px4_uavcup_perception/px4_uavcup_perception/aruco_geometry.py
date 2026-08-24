"""Pure geometry helpers for ArUco pose handling."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return a normalized Hamilton quaternion [x, y, z, w]."""
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError('rotation matrix must have shape (3, 3)')
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / s,
            (matrix[0, 2] - matrix[2, 0]) / s,
            (matrix[1, 0] - matrix[0, 1]) / s,
            0.25 * s,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * s,
                (matrix[0, 1] + matrix[1, 0]) / s,
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[2, 1] - matrix[1, 2]) / s,
            ])
        elif index == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / s,
                0.25 * s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
            ])
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / s,
                (matrix[1, 2] + matrix[2, 1]) / s,
                0.25 * s,
                (matrix[1, 0] - matrix[0, 1]) / s,
            ])
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError('rotation matrix produced an invalid quaternion')
    return quaternion / norm


def camera_matrix(values: Iterable[float]) -> np.ndarray:
    """Validate and reshape a ROS CameraInfo K array."""
    matrix = np.asarray(list(values), dtype=np.float64)
    if matrix.size != 9:
        raise ValueError('camera matrix must contain 9 values')
    matrix = matrix.reshape(3, 3)
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError('camera focal lengths must be positive')
    return matrix

"""Pure NumPy helpers for optional front-camera point-cloud debugging."""

from __future__ import annotations

import numpy as np


def depth_to_flu_points(
        depth_m: np.ndarray,
        focal_x_px: float,
        focal_y_px: float,
        principal_x_px: float,
        principal_y_px: float,
        stride: int = 4,
        minimum_depth_m: float = 0.3,
        maximum_depth_m: float = 8.0) -> np.ndarray:
    """
    Convert an OpenCV depth image into Nx3 ROS FLU points.

    OpenCV optical coordinates are X-right, Y-down, Z-forward. The returned
    coordinates are ROS FLU: X-forward, Y-left, Z-up.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError('Depth image must be a non-empty 2D array')
    if focal_x_px <= 0.0 or focal_y_px <= 0.0:
        raise ValueError('Camera focal lengths must be positive')
    if stride <= 0:
        raise ValueError('Point-cloud stride must be positive')
    if maximum_depth_m <= minimum_depth_m:
        raise ValueError('Maximum depth must be greater than minimum depth')

    rows = np.arange(0, depth.shape[0], stride, dtype=np.float32)
    columns = np.arange(0, depth.shape[1], stride, dtype=np.float32)
    u_grid, v_grid = np.meshgrid(columns, rows)
    sampled = depth[::stride, ::stride]
    valid = (
        np.isfinite(sampled)
        & (sampled >= float(minimum_depth_m))
        & (sampled <= float(maximum_depth_m))
    )

    forward = sampled[valid]
    left = -(u_grid[valid] - float(principal_x_px)) * forward / focal_x_px
    up = -(v_grid[valid] - float(principal_y_px)) * forward / focal_y_px
    return np.column_stack((forward, left, up)).astype(
        np.float32, copy=False)

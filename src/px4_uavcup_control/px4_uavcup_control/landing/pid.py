"""Pure PID and frame helpers for ArUco precision landing."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


DOWN_CAMERA_OPTICAL_TO_BODY_FLU = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)


def camera_target_to_body_flu(
        camera_xyz,
        rotation,
        camera_position_body=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Transform a camera-relative marker position into body FLU."""
    target = np.asarray(camera_xyz, dtype=np.float64)
    matrix = np.asarray(rotation, dtype=np.float64)
    camera_position = np.asarray(
        camera_position_body, dtype=np.float64)
    if (target.shape != (3,) or matrix.shape != (3, 3)
            or camera_position.shape != (3,)):
        raise ValueError('expected two 3-vectors and a 3x3 rotation')
    if (not np.all(np.isfinite(target))
            or not np.all(np.isfinite(matrix))
            or not np.all(np.isfinite(camera_position))):
        raise ValueError('landing geometry must be finite')
    return camera_position + matrix @ target


def gripper_clearance_from_marker(
        camera_marker_distance_m: float,
        camera_to_gripper_vertical_m: float) -> float:
    """Estimate gripper clearance above a marker on the target surface."""
    values = (camera_marker_distance_m, camera_to_gripper_vertical_m)
    if not all(np.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError('landing clearances must be finite and non-negative')
    return max(0.0, camera_marker_distance_m - camera_to_gripper_vertical_m)


def ready_for_blind_descent(
        gripper_clearance_m: float,
        horizontal_error_m: float,
        transition_clearance_m: float,
        maximum_horizontal_error_m: float) -> bool:
    """Return whether visual alignment is safe to hand off to blind descent."""
    values = (
        gripper_clearance_m,
        horizontal_error_m,
        transition_clearance_m,
        maximum_horizontal_error_m,
    )
    if not all(np.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError('blind-descent gate values must be non-negative')
    return (
        gripper_clearance_m <= transition_clearance_m
        and horizontal_error_m <= maximum_horizontal_error_m)


@dataclass
class PidAxis:
    """Small PID with integral/output clamps and derivative on error."""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    integral: float = 0.0
    previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        if not np.isfinite(error) or not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(
                'PID error and dt must be finite; dt must be positive')
        self.integral = float(np.clip(
            self.integral + error * dt,
            -self.integral_limit, self.integral_limit))
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = (
            self.kp * error + self.ki * self.integral
            + self.kd * derivative)
        return float(np.clip(output, -self.output_limit, self.output_limit))

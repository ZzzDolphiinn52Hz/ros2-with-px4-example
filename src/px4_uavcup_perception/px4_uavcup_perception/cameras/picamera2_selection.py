"""Deterministic Picamera2 selection for multi-camera Raspberry Pi systems."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any


def select_camera(
        cameras: Sequence[dict[str, Any]], *, model: str,
        index: int | None = None) -> dict[str, Any]:
    """Select one Picamera2 camera by index or unique model name."""
    if not cameras:
        raise RuntimeError('Picamera2 did not discover any cameras')
    if index is not None:
        matches = [camera for camera in cameras
                   if int(camera.get('Num', -1)) == index]
        if not matches:
            available = ', '.join(str(camera.get('Num', '?'))
                                  for camera in cameras)
            raise RuntimeError(
                f'Picamera2 camera index {index} is unavailable; '
                f'available indexes: {available}')
        return matches[0]

    requested = model.strip().lower()
    if not requested:
        raise ValueError('camera model must not be empty')
    matches = [
        camera for camera in cameras
        if requested in str(camera.get('Model', '')).lower()
    ]
    if len(matches) != 1:
        available = ', '.join(
            f"{camera.get('Num', '?')}:{camera.get('Model', '?')}"
            for camera in cameras)
        if not matches:
            raise RuntimeError(
                f'Picamera2 model {model!r} is unavailable; '
                f'available cameras: {available}')
        raise RuntimeError(
            f'Picamera2 model {model!r} is ambiguous; '
            f'available cameras: {available}')
    return matches[0]


def select_sensor_size(
        *, model: str, output_width: int, output_height: int,
        sensor_width: int = 0,
        sensor_height: int = 0) -> tuple[int, int] | None:
    """Choose a stable sensor mode while retaining the requested output."""
    if (sensor_width == 0) != (sensor_height == 0):
        raise ValueError(
            'sensor width and height must either both be zero or both be set')
    if sensor_width < 0 or sensor_height < 0:
        raise ValueError('sensor width and height must be non-negative')
    if sensor_width and sensor_height:
        return sensor_width, sensor_height
    if ('imx219' in model.lower()
            and output_width <= 1640 and output_height <= 1232):
        # The 640x480 sensor crop times out on the tested Pi 5/IMX219 pair.
        # Capture the stable 2x2-binned mode and let PiSP scale to the output.
        return 1640, 1232
    return None

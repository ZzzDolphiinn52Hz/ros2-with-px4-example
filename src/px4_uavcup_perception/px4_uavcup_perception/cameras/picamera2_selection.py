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

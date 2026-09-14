"""Picamera2 sensor-mode selection for Raspberry Pi camera pipelines."""

from __future__ import annotations


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
    return None


def select_capture_format(model: str) -> tuple[str, int]:
    """Return a working Picamera2 format and its array channel count."""
    if 'imx219' in model.lower():
        # RGB888 stalls the CSI frontend with the tested libcamera/PiSP stack.
        # XBGR8888 is RGBX in byte order on this little-endian platform.
        return 'XBGR8888', 4
    return 'RGB888', 3

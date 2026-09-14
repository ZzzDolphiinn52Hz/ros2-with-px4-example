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
    if ('imx219' in model.lower()
            and output_width <= 1640 and output_height <= 1232):
        # The 640x480 sensor crop times out on the tested Pi 5/IMX219 pair.
        # Capture the stable 2x2-binned mode and let PiSP scale to the output.
        return 1640, 1232
    return None

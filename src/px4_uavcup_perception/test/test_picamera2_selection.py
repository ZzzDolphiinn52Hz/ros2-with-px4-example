import pytest

from px4_uavcup_perception.cameras.picamera2_selection import (
    select_sensor_size,
)


def test_imx219_uses_stable_binned_sensor_mode_for_vga_output():
    assert select_sensor_size(
        model='imx219', output_width=640, output_height=480,
    ) == (1640, 1232)


def test_explicit_sensor_mode_overrides_model_default():
    assert select_sensor_size(
        model='imx219', output_width=640, output_height=480,
        sensor_width=1920, sensor_height=1080,
    ) == (1920, 1080)


def test_sensor_mode_requires_both_dimensions():
    with pytest.raises(ValueError, match='both be zero or both be set'):
        select_sensor_size(
            model='imx219', output_width=640, output_height=480,
            sensor_width=1640,
        )

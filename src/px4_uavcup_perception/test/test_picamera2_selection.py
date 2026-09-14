import pytest

from px4_uavcup_perception.cameras.picamera2_selection import (
    select_capture_format,
    select_sensor_size,
)


def test_imx219_does_not_force_a_raw_sensor_mode():
    assert select_sensor_size(
        model='imx219', output_width=640, output_height=480,
    ) is None


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


def test_imx219_uses_four_channel_capture_format():
    assert select_capture_format('imx219') == ('XBGR8888', 4)


def test_other_cameras_keep_rgb_capture_format():
    assert select_capture_format('imx500') == ('RGB888', 3)

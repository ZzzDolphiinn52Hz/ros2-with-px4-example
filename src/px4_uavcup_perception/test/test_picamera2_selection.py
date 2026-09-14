import pytest

from px4_uavcup_perception.cameras.picamera2_selection import (
    select_camera,
    select_sensor_size,
)


CAMERAS = [
    {'Num': 0, 'Model': 'imx500', 'Id': '/base/cam0/imx500@1a'},
    {'Num': 1, 'Model': 'imx219', 'Id': '/base/cam1/imx219@10'},
]


def test_select_camera_by_model_is_case_insensitive():
    assert select_camera(CAMERAS, model='IMX219')['Num'] == 1


def test_select_camera_by_index_overrides_model():
    assert select_camera(CAMERAS, model='imx500', index=1)['Model'] == 'imx219'


def test_select_camera_rejects_unavailable_model():
    with pytest.raises(RuntimeError, match='available cameras'):
        select_camera(CAMERAS, model='ov5647')


def test_select_camera_rejects_empty_inventory():
    with pytest.raises(RuntimeError, match='did not discover'):
        select_camera([], model='imx219')


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

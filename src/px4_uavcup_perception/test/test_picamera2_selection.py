import pytest

from px4_uavcup_perception.cameras.picamera2_selection import select_camera


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

from pathlib import Path

import numpy as np
import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


def _parameters(filename, node_name):
    with (CONFIG_DIR / filename).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)[node_name]['ros__parameters']


def test_imx500_intrinsics_are_valid_and_synchronized():
    detector = _parameters('aruco.yaml', 'aruco_detector_node')
    camera = _parameters('pi_cameras.yaml', 'down_picamera')

    detector_matrix = np.asarray(detector['camera_matrix']).reshape(3, 3)
    camera_matrix = np.asarray(camera['camera_matrix']).reshape(3, 3)
    detector_distortion = np.asarray(detector['distortion_coefficients'])
    camera_distortion = np.asarray(camera['distortion_coefficients'])

    np.testing.assert_allclose(detector_matrix, camera_matrix)
    np.testing.assert_allclose(detector_distortion, camera_distortion)
    assert np.all(np.isfinite(detector_matrix))
    assert np.all(np.isfinite(detector_distortion))
    assert detector_matrix[0, 0] > 0.0
    assert detector_matrix[1, 1] > 0.0
    assert 0.0 < detector_matrix[0, 2] < detector['camera_width']
    assert 0.0 < detector_matrix[1, 2] < detector['camera_height']
    assert detector_distortion.shape == (5,)


def test_aruco_runtime_matches_printed_80mm_target():
    detector = _parameters('aruco.yaml', 'aruco_detector_node')

    assert detector['dictionary'] == 'DICT_5X5_50'
    assert detector['target_marker_id'] == 0
    assert detector['marker_size_m'] == 0.08

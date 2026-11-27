from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _root(relative_path: str):
    return ET.parse(PACKAGE_ROOT / relative_path).getroot()


def test_down_camera_matches_imx500_contract():
    root = _root('models/x500_uavcup_aruco/model.sdf')
    sensor = root.find(".//sensor[@name='down_imx500']")
    assert sensor is not None
    assert sensor.get('type') == 'camera'
    assert sensor.findtext('topic') == '/camera/down/image_raw'
    assert sensor.findtext('camera/camera_info_topic') == (
        '/camera/down/camera_info')
    assert sensor.findtext('camera/image/width') == '640'
    assert sensor.findtext('camera/image/height') == '480'
    assert float(sensor.findtext('update_rate')) == 15.0
    assert sensor.findtext('camera/optical_frame_id') == (
        'down_camera_optical_frame')

    expected = {
        'fx': 480.125104,
        'fy': 480.224260,
        'cx': 320.272190,
        'cy': 228.696135,
    }
    for key, value in expected.items():
        actual = float(sensor.findtext(f'camera/lens/intrinsics/{key}'))
        assert abs(actual - value) < 1e-6
    assert sensor.find('camera/distortion') is None

    link = root.find(".//link[@name='down_camera_link']")
    pose = [float(value) for value in link.findtext('pose').split()]
    assert pose[:3] == [0.0, 0.0, -0.09]
    assert abs(pose[4] - np.pi / 2.0) < 1e-6


def test_marker_geometry_and_texture_are_id_zero():
    root = _root('models/aruco_cargo_80mm/model.sdf')
    marker = root.find(".//visual[@name='aruco_5x5_50_id_0']")
    assert marker is not None
    size = [float(value) for value in marker.findtext(
        'geometry/plane/size').split()]
    assert size == [0.08, 0.08]

    texture_path = (
        PACKAGE_ROOT / 'models/aruco_cargo_80mm/materials/textures/'
        'aruco_5x5_50_id_0.png')
    texture = cv2.imread(str(texture_path), cv2.IMREAD_GRAYSCALE)
    assert texture is not None
    assert texture.shape[0] == texture.shape[1]
    assert set(np.unique(texture)).issubset({0, 255})

    padded = cv2.copyMakeBorder(
        texture, 100, 100, 100, 100,
        cv2.BORDER_CONSTANT, value=255)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, rejected = cv2.aruco.ArucoDetector(
            dictionary).detectMarkers(padded)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            padded, dictionary)
    del corners, rejected
    assert ids is not None
    assert ids.flatten().tolist() == [0]


def test_detector_world_contains_supported_vehicle_and_marker():
    root = _root('worlds/aruco_detector.sdf')
    world = root.find("world[@name='aruco_detector']")
    assert world is not None
    uris = [element.text for element in world.findall('.//include/uri')]
    assert 'model://x500_uavcup_aruco' in uris
    assert 'model://aruco_cargo_80mm' in uris
    stand = world.find(".//model[@name='invisible_detector_stand']")
    assert stand is not None
    assert stand.find('.//collision') is not None
    assert stand.find('.//visual') is None

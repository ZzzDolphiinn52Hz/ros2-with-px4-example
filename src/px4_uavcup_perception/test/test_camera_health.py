import numpy as np

from px4_uavcup_perception.camera_health import (
    CameraHealthGate,
    assess_camera_health,
)


def test_textured_normally_exposed_image_is_healthy():
    rows, columns = np.indices((64, 64))
    gray = ((rows * 7 + columns * 11) % 180 + 35).astype(np.uint8)
    image = np.repeat(gray[:, :, None], 3, axis=2)

    health = assess_camera_health(image, sample_stride=1)

    assert health.healthy
    assert health.reason == 'valid'
    assert health.contrast_stddev > 6.0
    assert health.gradient_mean > 1.0


def test_black_covered_camera_is_rejected():
    image = np.zeros((64, 64, 3), dtype=np.uint8)

    health = assess_camera_health(image, sample_stride=1)

    assert not health.healthy
    assert 'dark' in health.reason
    assert health.dark_fraction == 1.0


def test_uniform_mid_gray_camera_is_rejected_as_textureless():
    image = np.full((64, 64, 3), 120, dtype=np.uint8)

    health = assess_camera_health(image, sample_stride=1)

    assert not health.healthy
    assert 'texture' in health.reason
    assert health.dark_fraction == 0.0
    assert health.bright_fraction == 0.0


def test_gate_debounces_failure_and_requires_stable_recovery():
    gate = CameraHealthGate(failure_frames=3, recovery_frames=2)

    assert gate.update(False)
    assert gate.update(False)
    assert not gate.update(False)
    assert not gate.update(True)
    assert gate.update(True)

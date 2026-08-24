import numpy as np
import pytest

from px4_uavcup_perception.zipdepth_onnx_backend import (
    ZipDepthOnnx,
    inverse_depth_to_metric,
)


class FakeCv2:
    INTER_LINEAR = 1

    @staticmethod
    def resize(image, size, interpolation):
        del interpolation
        width, height = size
        assert image.shape == (height, width, 3)
        return image.copy()


def test_zipdepth_preprocess_converts_bgr_to_nchw_rgb():
    backend = object.__new__(ZipDepthOnnx)
    backend.height = 1
    backend.width = 1
    bgr = np.array([[[0, 128, 255]]], dtype=np.uint8)
    result = backend.preprocess(bgr, FakeCv2)
    assert result.shape == (1, 3, 1, 1)
    assert result.dtype == np.float32
    assert result[0, :, 0, 0] == pytest.approx([1.0, 128 / 255.0, 0.0])


def test_inverse_depth_alignment_is_converted_to_metres():
    raw = np.array([[1.0, 2.0]], dtype=np.float32)
    metric = inverse_depth_to_metric(raw, scale=0.5, shift_inverse_m=0.5)
    np.testing.assert_allclose(metric, [[1.0, 2.0 / 3.0]])


def test_inverse_depth_conversion_rejects_nonfinite_calibration():
    raw = np.array([[1.0]], dtype=np.float32)
    with pytest.raises(ValueError):
        inverse_depth_to_metric(raw, scale=float('nan'), shift_inverse_m=0.0)

"""ONNX Runtime backend for the edge-friendly ZipDepth model."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def inverse_depth_to_metric(
        raw_inverse_depth: np.ndarray,
        scale: float,
        shift_inverse_m: float,
        minimum_inverse_depth: float = 1e-3) -> np.ndarray:
    """Align affine-invariant inverse depth, then convert it to metres."""
    raw = np.asarray(raw_inverse_depth, dtype=np.float32)
    values = raw * float(scale) + float(shift_inverse_m)
    if not np.all(np.isfinite(values)):
        raise ValueError('calibrated inverse depth must be finite')
    if minimum_inverse_depth <= 0.0:
        raise ValueError('minimum_inverse_depth must be positive')
    return np.reciprocal(np.maximum(values, minimum_inverse_depth)).astype(
        np.float32, copy=False)


class ZipDepthOnnx:
    """Run a static-shape ZipDepth ONNX graph on CPU."""

    def __init__(self, model_path: Path, threads: int = 3) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(f'ZipDepth ONNX model not found: {model_path}')
        if threads <= 0:
            raise ValueError('ONNX Runtime thread count must be positive')
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError('onnxruntime is required for ZipDepth on Pi') from error
        options = ort.SessionOptions()
        options.intra_op_num_threads = int(threads)
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self._session = ort.InferenceSession(
            str(model_path), sess_options=options,
            providers=['CPUExecutionProvider'])
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError('ZipDepth ONNX must have one input and one output')
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name
        shape = inputs[0].shape
        if len(shape) != 4 or not isinstance(shape[2], int) or not isinstance(shape[3], int):
            raise RuntimeError(f'ZipDepth ONNX input must be static NCHW, got {shape}')
        self.height = int(shape[2])
        self.width = int(shape[3])

    def preprocess(self, bgr_image: np.ndarray, cv2) -> np.ndarray:
        """Match the official exporter: resized RGB float32 in [0, 1]."""
        image = cv2.resize(
            bgr_image, (self.width, self.height),
            interpolation=cv2.INTER_LINEAR)
        image = image[:, :, ::-1].astype(np.float32) / 255.0
        return np.ascontiguousarray(image.transpose(2, 0, 1)[None, ...])

    def infer(self, bgr_image: np.ndarray, cv2) -> np.ndarray:
        input_tensor = self.preprocess(bgr_image, cv2)
        result = self._session.run(
            [self._output_name], {self._input_name: input_tensor})[0]
        result = np.squeeze(np.asarray(result, dtype=np.float32))
        if result.ndim != 2:
            raise RuntimeError(f'Expected 2D ZipDepth output, got {result.shape}')
        return result

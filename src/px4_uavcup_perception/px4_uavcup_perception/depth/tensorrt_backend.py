"""TensorRT 8 backend used by the JetPack 5 Xavier NX deployment."""

from __future__ import annotations

import importlib
from pathlib import Path

import numpy as np


class DepthAnythingTensorRT:
    """Execute a static Depth Anything V2 FP16 TensorRT engine."""

    def __init__(self, engine_path: Path, input_size: int = 364) -> None:
        if not engine_path.is_file():
            raise FileNotFoundError(f'TensorRT engine not found: {engine_path}')
        if input_size <= 0:
            raise ValueError('TensorRT input size must be positive')

        try:
            self._trt = importlib.import_module('tensorrt')
            self._cuda = importlib.import_module('pycuda.driver')
            # Creates the CUDA context required by pycuda allocations.
            importlib.import_module('pycuda.autoinit')
        except ImportError as error:
            raise RuntimeError(
                'TensorRT and PyCUDA must be installed on the Jetson') \
                from error

        self.input_size = int(input_size)
        self._mean = np.array(
            [0.485, 0.456, 0.406], dtype=np.float32)
        self._std = np.array(
            [0.229, 0.224, 0.225], dtype=np.float32)

        logger = self._trt.Logger(self._trt.Logger.WARNING)
        with engine_path.open('rb') as engine_file:
            serialized_engine = engine_file.read()
        with self._trt.Runtime(logger) as runtime:
            self._engine = runtime.deserialize_cuda_engine(serialized_engine)
        if self._engine is None:
            raise RuntimeError(f'Cannot deserialize TensorRT engine: {engine_path}')
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError('Cannot create TensorRT execution context')

        self._inputs = []
        self._outputs = []
        self._bindings = []
        self._stream = self._cuda.Stream()
        for binding_name in self._engine:
            binding_shape = self._engine.get_binding_shape(binding_name)
            element_count = int(self._trt.volume(binding_shape))
            dtype = self._trt.nptype(
                self._engine.get_binding_dtype(binding_name))
            host = self._cuda.pagelocked_empty(element_count, dtype)
            device = self._cuda.mem_alloc(host.nbytes)
            binding = {
                'name': binding_name,
                'shape': tuple(binding_shape),
                'host': host,
                'device': device,
            }
            self._bindings.append(int(device))
            if self._engine.binding_is_input(binding_name):
                self._inputs.append(binding)
            else:
                self._outputs.append(binding)
        if len(self._inputs) != 1 or len(self._outputs) != 1:
            raise RuntimeError(
                'Expected exactly one TensorRT input and one output, got '
                f'{len(self._inputs)} input(s) and {len(self._outputs)} '
                'output(s)')

    def preprocess(self, bgr_image, cv2) -> np.ndarray:
        """Return normalized NCHW RGB input for Depth Anything V2."""
        image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        if image.shape[:2] != (self.input_size, self.input_size):
            image = cv2.resize(
                image, (self.input_size, self.input_size),
                interpolation=cv2.INTER_LINEAR)
        image = image.astype(np.float32) / 255.0
        image = (image - self._mean) / self._std
        image = image.transpose(2, 0, 1)[None, ...]
        return np.ascontiguousarray(image)

    def infer(self, bgr_image, cv2) -> np.ndarray:
        """Return the metric-depth output as a two-dimensional array."""
        input_data = self.preprocess(bgr_image, cv2)
        input_binding = self._inputs[0]
        output_binding = self._outputs[0]
        if input_data.size != input_binding['host'].size:
            raise RuntimeError(
                f'Engine input has {input_binding["host"].size} elements, '
                f'but preprocessing produced {input_data.size}')

        np.copyto(input_binding['host'], input_data.ravel())
        self._cuda.memcpy_htod_async(
            input_binding['device'], input_binding['host'], self._stream)
        succeeded = self._context.execute_async_v2(
            bindings=self._bindings,
            stream_handle=self._stream.handle,
        )
        if not succeeded:
            raise RuntimeError('TensorRT execution failed')
        self._cuda.memcpy_dtoh_async(
            output_binding['host'], output_binding['device'], self._stream)
        self._stream.synchronize()

        output = np.asarray(output_binding['host']).reshape(
            output_binding['shape'])
        output = np.squeeze(output)
        if output.ndim != 2:
            raise RuntimeError(
                f'Expected a 2D depth output, got shape {output.shape}')
        return output.astype(np.float32, copy=False)

#!/usr/bin/env python3
"""Run the official metric Depth Anything V2 model on a ROS image stream."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import time
from typing import Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .image_utils import array_to_image, ros_image_to_bgr


_MODEL_CONFIGS = {
    'vits': {
        'encoder': 'vits',
        'features': 64,
        'out_channels': [48, 96, 192, 384],
    },
    'vitb': {
        'encoder': 'vitb',
        'features': 128,
        'out_channels': [96, 192, 384, 768],
    },
    'vitl': {
        'encoder': 'vitl',
        'features': 256,
        'out_channels': [256, 512, 1024, 1024],
    },
}


def _expanded_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


class DepthAnythingBackend:
    """Thin adapter around the official metric-depth implementation."""

    def __init__(
            self,
            repository: Path,
            checkpoint: Path,
            encoder: str,
            maximum_depth_m: float,
            input_size: int,
            device: str) -> None:
        if encoder not in _MODEL_CONFIGS:
            raise ValueError(f'Unsupported encoder: {encoder}')
        metric_root = repository / 'metric_depth'
        module_file = metric_root / 'depth_anything_v2' / 'dpt.py'
        if not module_file.is_file():
            raise FileNotFoundError(
                f'Depth Anything V2 metric source not found: {module_file}')
        if not checkpoint.is_file():
            raise FileNotFoundError(
                f'Model checkpoint not found: {checkpoint}')

        sys.path.insert(0, str(metric_root))
        try:
            import torch
            import torch.nn.functional as torch_functional
            from depth_anything_v2.dpt import DepthAnythingV2
        except Exception as error:
            raise RuntimeError(
                'Cannot import metric Depth Anything V2 dependencies. '
                'Run scripts/install_depth_anything_v2.sh in this package.'
            ) from error

        if device == 'auto':
            if torch.cuda.is_available():
                device = 'cuda'
            elif (
                    hasattr(torch.backends, 'mps')
                    and torch.backends.mps.is_available()):
                device = 'mps'
            else:
                device = 'cpu'
        self.device = device
        self.input_size = int(input_size)
        self._torch = torch
        self._functional = torch_functional
        configuration = {
            **_MODEL_CONFIGS[encoder],
            'max_depth': float(maximum_depth_m),
        }
        model = DepthAnythingV2(**configuration)
        state = torch.load(str(checkpoint), map_location='cpu')
        model.load_state_dict(state)
        self._model = model.to(self.device).eval()

    def infer(self, bgr_image: np.ndarray) -> np.ndarray:
        """Return metric depth in metres with the same size as the input."""
        height, width = bgr_image.shape[:2]
        with self._torch.no_grad():
            tensor, _ = self._model.image2tensor(
                bgr_image, self.input_size)
            tensor = tensor.to(self.device)
            depth = self._model.forward(tensor)
            depth = self._functional.interpolate(
                depth[:, None],
                (height, width),
                mode='bilinear',
                align_corners=True,
            )[0, 0]
        return depth.detach().cpu().numpy().astype(np.float32, copy=False)


class DepthAnythingNode(Node):
    def __init__(self) -> None:
        super().__init__('depth_anything_node')
        self.declare_parameter(
            'input_topic', '/uav/front_camera/image_raw')
        self.declare_parameter('depth_topic', '/uav/depth/image')
        self.declare_parameter(
            'visualization_topic', '/uav/depth/visualization')
        self.declare_parameter('status_topic', '/uav/depth/status')
        self.declare_parameter(
            'repository_path',
            '~/ros2_ws/third_party/Depth-Anything-V2')
        self.declare_parameter(
            'checkpoint_path',
            '~/ros2_ws/models/depth_anything_v2_metric_hypersim_vits.pth')
        self.declare_parameter('encoder', 'vits')
        self.declare_parameter('maximum_depth_m', 20.0)
        self.declare_parameter('input_size', 518)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('retry_model_load_s', 5.0)

        self._repository = _expanded_path(
            str(self.get_parameter('repository_path').value))
        self._checkpoint = _expanded_path(
            str(self.get_parameter('checkpoint_path').value))
        self._encoder = str(self.get_parameter('encoder').value)
        self._maximum_depth_m = float(
            self.get_parameter('maximum_depth_m').value)
        self._input_size = int(self.get_parameter('input_size').value)
        self._device = str(self.get_parameter('device').value)
        self._retry_model_load_s = float(
            self.get_parameter('retry_model_load_s').value)

        self._depth_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('depth_topic').value),
            qos_profile_sensor_data,
        )
        self._visualization_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('visualization_topic').value),
            qos_profile_sensor_data,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('input_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )

        self._backend: Optional[DepthAnythingBackend] = None
        self._last_load_attempt = float('-inf')
        self._last_error = ''
        self.get_logger().info(
            f'Waiting for RGB images; model={self._encoder} '
            f'checkpoint={self._checkpoint}')

    def _load_backend_if_due(self) -> bool:
        if self._backend is not None:
            return True
        now = time.monotonic()
        if now - self._last_load_attempt < self._retry_model_load_s:
            return False
        self._last_load_attempt = now
        try:
            self._backend = DepthAnythingBackend(
                repository=self._repository,
                checkpoint=self._checkpoint,
                encoder=self._encoder,
                maximum_depth_m=self._maximum_depth_m,
                input_size=self._input_size,
                device=self._device,
            )
        # Keep the bridge alive while dependencies are being installed.
        except Exception as error:
            self._last_error = str(error)
            self.get_logger().error(
                f'Depth model unavailable: {error}',
                throttle_duration_sec=max(1.0, self._retry_model_load_s),
            )
            self._publish_status(
                DiagnosticStatus.ERROR, 'model unavailable', 0.0)
            return False
        self._last_error = ''
        self.get_logger().info(
            f'Depth Anything V2 loaded on {self._backend.device}')
        return True

    def _on_image(self, message: Image) -> None:
        if not self._load_backend_if_due():
            return
        started = time.perf_counter()
        try:
            bgr = ros_image_to_bgr(message)
            depth = self._backend.infer(bgr)
        except Exception as error:
            self._last_error = str(error)
            self.get_logger().error(
                f'Depth inference failed: {error}', throttle_duration_sec=2.0)
            self._publish_status(
                DiagnosticStatus.ERROR, 'inference failed', 0.0)
            return

        latency_ms = (time.perf_counter() - started) * 1000.0
        self._depth_publisher.publish(
            array_to_image(depth, '32FC1', source=message))
        self._visualization_publisher.publish(
            array_to_image(self._visualize(depth), 'mono8', source=message))
        self._publish_status(DiagnosticStatus.OK, 'running', latency_ms)

    @staticmethod
    def _visualize(depth: np.ndarray) -> np.ndarray:
        valid = np.isfinite(depth) & (depth > 0.0)
        output = np.zeros(depth.shape, dtype=np.uint8)
        if not np.any(valid):
            return output
        low, high = np.percentile(depth[valid], [2.0, 98.0])
        if high <= low:
            output[valid] = 255
            return output
        normalized = np.clip((depth - low) / (high - low), 0.0, 1.0)
        output[valid] = ((1.0 - normalized[valid]) * 255.0).astype(np.uint8)
        return output

    def _publish_status(
            self, level: int, message: str, latency_ms: float) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'px4_uavcup/depth_anything_v2'
        status.hardware_id = self._backend.device if self._backend else 'none'
        status.message = message
        status.values = [
            KeyValue(key='encoder', value=self._encoder),
            KeyValue(key='latency_ms', value=f'{latency_ms:.2f}'),
            KeyValue(key='checkpoint', value=str(self._checkpoint)),
            KeyValue(key='error', value=self._last_error),
        ]
        array.status = [status]
        self._status_publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

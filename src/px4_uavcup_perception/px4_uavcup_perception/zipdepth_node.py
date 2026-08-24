#!/usr/bin/env python3
"""ROS 2 subscriber pipeline for ZipDepth on Raspberry Pi 5."""

from __future__ import annotations

import os
from pathlib import Path
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from .camera_health import assess_camera_health
from .free_space import summarize_free_space
from .image_utils import array_to_image, image_to_bgr
from .zipdepth_onnx_backend import ZipDepthOnnx, inverse_depth_to_metric


class ZipDepthNode(Node):
    """Run relative ZipDepth safely; metric output requires calibration."""

    def __init__(self) -> None:
        super().__init__('zipdepth_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'model_path', '~/models/zipdepth_base_npu_512x384.onnx')
        self.declare_parameter('onnx_threads', 3)
        self.declare_parameter('maximum_processing_rate_hz', 0.0)
        self.declare_parameter('metric_calibration_enabled', False)
        self.declare_parameter('inverse_depth_scale', 1.0)
        self.declare_parameter('inverse_depth_shift_per_m', 0.0)
        self.declare_parameter('minimum_depth_m', 0.3)
        self.declare_parameter('maximum_depth_m', 8.0)
        self.declare_parameter('publish_raw_output', True)

        model_path = Path(os.path.expanduser(os.path.expandvars(
            str(self.get_parameter('model_path').value)))).resolve()
        threads = int(self.get_parameter('onnx_threads').value)
        rate = float(self.get_parameter('maximum_processing_rate_hz').value)
        if not np.isfinite(rate) or rate < 0.0:
            raise ValueError(
                'maximum_processing_rate_hz must be finite and non-negative')
        # Zero disables throttling. In that mode inference itself determines
        # throughput and sensor-data QoS drops stale camera frames.
        self._minimum_period = 0.0 if rate == 0.0 else 1.0 / rate
        self._last_processed = float('-inf')
        self._last_completed = None
        self._processing_fps = 0.0
        self._metric_enabled = bool(
            self.get_parameter('metric_calibration_enabled').value)
        self._inverse_depth_scale = float(
            self.get_parameter('inverse_depth_scale').value)
        self._inverse_depth_shift = float(
            self.get_parameter('inverse_depth_shift_per_m').value)
        if (not np.isfinite(self._inverse_depth_scale)
                or self._inverse_depth_scale <= 0.0):
            raise ValueError('inverse_depth_scale must be finite and positive')
        if not np.isfinite(self._inverse_depth_shift):
            raise ValueError('inverse_depth_shift_per_m must be finite')
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError('OpenCV is required for ZipDepth') from error
        self._cv2 = cv2
        self._backend = ZipDepthOnnx(model_path, threads)
        self._raw_publisher = self.create_publisher(
            Image, '/uav/depth/zipdepth_raw', qos_profile_sensor_data) \
            if bool(self.get_parameter('publish_raw_output').value) else None
        self._metric_publisher = self.create_publisher(
            Image, '/camera/depth/image', qos_profile_sensor_data)
        self._free_space_publisher = self.create_publisher(
            Float32MultiArray, '/uav/depth/free_space', 1)
        self._status_publisher = self.create_publisher(
            DiagnosticArray, '/uav/depth/status', 10)
        latest_frame_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, latest_frame_qos)
        self.get_logger().info(
            f'ZipDepth ONNX ready: {model_path}, '
            f'{self._backend.width}x{self._backend.height}, '
            f'metric_calibration={self._metric_enabled}, '
            f'rate_limit={rate if rate > 0.0 else "unlimited"}')

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_processed < self._minimum_period:
            return
        self._last_processed = now
        started = time.perf_counter()
        try:
            bgr = image_to_bgr(message)
            health = assess_camera_health(bgr)
            if not health.healthy:
                raise RuntimeError(health.reason)
            raw = self._backend.infer(bgr, self._cv2)
            if self._raw_publisher is not None:
                raw_message = array_to_image(raw, '32FC1')
                raw_message.header = message.header
                self._raw_publisher.publish(raw_message)
            if self._metric_enabled:
                metric = inverse_depth_to_metric(
                    raw, self._inverse_depth_scale,
                    self._inverse_depth_shift)
                metric = self._cv2.resize(
                    metric, (int(message.width), int(message.height)),
                    interpolation=self._cv2.INTER_LINEAR)
                metric_message = array_to_image(metric, '32FC1')
                metric_message.header = message.header
                self._metric_publisher.publish(metric_message)
                summary = summarize_free_space(
                    metric,
                    minimum_depth_m=float(
                        self.get_parameter('minimum_depth_m').value),
                    maximum_depth_m=float(
                        self.get_parameter('maximum_depth_m').value),
                )
                free_space = Float32MultiArray()
                free_space.data = summary.as_list()
                self._free_space_publisher.publish(free_space)
            else:
                self._publish_invalid_free_space()
            completed = time.perf_counter()
            if self._last_completed is not None:
                elapsed = completed - self._last_completed
                if elapsed > 0.0:
                    self._processing_fps = 1.0 / elapsed
            self._last_completed = completed
            self._publish_status(
                DiagnosticStatus.OK
                if self._metric_enabled else DiagnosticStatus.WARN,
                'running_metric'
                if self._metric_enabled else 'running_relative_only',
                (completed - started) * 1000.0)
        except Exception as error:
            self._publish_invalid_free_space()
            self._publish_status(
                DiagnosticStatus.ERROR, str(error),
                (time.perf_counter() - started) * 1000.0)

    def _publish_invalid_free_space(self) -> None:
        message = Float32MultiArray()
        message.data = [float('nan')] * 4 + [0.0]
        self._free_space_publisher.publish(message)

    def _publish_status(
            self, level: int, text: str, latency_ms: float) -> None:
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'px4_uavcup/zipdepth_pi'
        status.hardware_id = 'raspberry_pi_5'
        status.level = level
        status.message = text
        status.values = [
            KeyValue(key='latency_ms', value=f'{latency_ms:.2f}'),
            KeyValue(key='metric_calibration_enabled',
                     value=str(self._metric_enabled).lower()),
            KeyValue(
                key='processing_fps', value=f'{self._processing_fps:.2f}'),
            KeyValue(key='output_width', value=str(self._backend.width)),
            KeyValue(key='output_height', value=str(self._backend.height)),
        ]
        diagnostic.status = [status]
        self._status_publisher.publish(diagnostic)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZipDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

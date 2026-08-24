#!/usr/bin/env python3
"""ROS 2 ZipDepth pipeline with direct or topic-based camera input."""

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
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension

from .camera_health import assess_camera_health
from .free_space import summarize_free_space
from .image_utils import array_to_image, image_to_bgr
from .pointcloud_utils import depth_to_flu_points
from .relative_free_space import summarize_relative_free_space
from .zipdepth_onnx_backend import (
    ZipDepthOnnx,
    inverse_depth_to_metric,
    normalize_inverse_depth_for_display,
)


class ZipDepthNode(Node):
    """Run relative ZipDepth safely; metric output requires calibration."""

    def __init__(self) -> None:
        super().__init__('zipdepth_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_capture_fps', 30.0)
        self.declare_parameter('camera_pixel_format', 'MJPG')
        self.declare_parameter('camera_frame_id', 'front_camera_optical_frame')
        self.declare_parameter('publish_input_image', False)
        self.declare_parameter(
            'model_path', '~/models/zipdepth_base_npu_512x384.onnx')
        self.declare_parameter('onnx_threads', 3)
        self.declare_parameter('maximum_processing_rate_hz', 0.0)
        self.declare_parameter('metric_calibration_enabled', False)
        self.declare_parameter('inverse_depth_scale', 1.0)
        self.declare_parameter('inverse_depth_shift_per_m', 0.0)
        self.declare_parameter('minimum_depth_m', 0.3)
        self.declare_parameter('maximum_depth_m', 8.0)
        self.declare_parameter('near_percentile', 15.0)
        self.declare_parameter('roi_top_fraction', 0.25)
        self.declare_parameter('roi_bottom_fraction', 0.78)
        self.declare_parameter('raw_topic', '/uav/depth/zipdepth_raw')
        self.declare_parameter('metric_depth_topic', '/camera/depth/image')
        self.declare_parameter(
            'visualization_topic', '/uav/depth/visualization')
        self.declare_parameter('pointcloud_topic', '/camera/depth/points')
        self.declare_parameter('free_space_topic', '/uav/depth/free_space')
        self.declare_parameter('status_topic', '/uav/depth/status')
        self.declare_parameter(
            'relative_free_space_topic',
            '/uav/depth/relative_free_space')
        self.declare_parameter('relative_near_percentile', 85.0)
        self.declare_parameter('relative_minimum_contrast_span', 0.001)
        self.declare_parameter('publish_raw_output', False)
        self.declare_parameter('publish_metric_depth', False)
        self.declare_parameter('publish_visualization', False)
        self.declare_parameter('publish_pointcloud', False)
        self.declare_parameter('pointcloud_stride', 4)
        self.declare_parameter('pointcloud_frame_id', 'front_camera_link')
        self.declare_parameter('focal_x_px', 430.0)
        self.declare_parameter('focal_y_px', 430.0)
        self.declare_parameter('principal_x_px', -1.0)
        self.declare_parameter('principal_y_px', -1.0)

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
        self._camera = None
        self._camera_device = str(
            self.get_parameter('camera_device').value).strip()
        self._publish_raw = bool(
            self.get_parameter('publish_raw_output').value)
        self._publish_metric = bool(
            self.get_parameter('publish_metric_depth').value)
        self._publish_visualization = bool(
            self.get_parameter('publish_visualization').value)
        self._publish_pointcloud = bool(
            self.get_parameter('publish_pointcloud').value)
        if self._publish_pointcloud and not self._metric_enabled:
            raise ValueError(
                'publish_pointcloud requires metric_calibration_enabled=true')
        if self._publish_metric and not self._metric_enabled:
            raise ValueError(
                'publish_metric_depth requires '
                'metric_calibration_enabled=true')
        self._pointcloud_stride = int(
            self.get_parameter('pointcloud_stride').value)
        self._pointcloud_frame_id = str(
            self.get_parameter('pointcloud_frame_id').value)
        self._focal_x = float(self.get_parameter('focal_x_px').value)
        self._focal_y = float(self.get_parameter('focal_y_px').value)
        self._principal_x = float(
            self.get_parameter('principal_x_px').value)
        self._principal_y = float(
            self.get_parameter('principal_y_px').value)
        self._raw_publisher = None
        self._metric_publisher = None
        self._visualization_publisher = None
        self._pointcloud_publisher = None
        if self._publish_raw:
            self._raw_publisher = self.create_publisher(
                Image, str(self.get_parameter('raw_topic').value),
                qos_profile_sensor_data)
        if self._publish_metric:
            self._metric_publisher = self.create_publisher(
                Image, str(self.get_parameter('metric_depth_topic').value),
                qos_profile_sensor_data)
        if self._publish_visualization:
            self._visualization_publisher = self.create_publisher(
                Image, str(self.get_parameter('visualization_topic').value),
                qos_profile_sensor_data)
        if self._publish_pointcloud:
            self._pointcloud_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter('pointcloud_topic').value),
                qos_profile_sensor_data)
        self._free_space_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('free_space_topic').value), 1)
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value), 10)
        self._relative_free_space_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('relative_free_space_topic').value), 1)
        self._input_publisher = None
        if self._camera_device:
            self._start_direct_camera(rate)
            input_source = f'direct:{self._camera_device}'
        else:
            latest_frame_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Image, str(self.get_parameter('image_topic').value),
                self._on_image, latest_frame_qos)
            input_source = str(self.get_parameter('image_topic').value)
        self._input_source = input_source
        self.get_logger().info(
            f'ZipDepth ONNX ready: {model_path}, '
            f'{self._backend.width}x{self._backend.height}, '
            f'metric_calibration={self._metric_enabled}, '
            f'rate_limit={rate if rate > 0.0 else "unlimited"}, '
            f'input={input_source}, raw={self._publish_raw}, '
            f'visualization={self._publish_visualization}, '
            f'pointcloud={self._publish_pointcloud}')

    def _start_direct_camera(self, rate: float) -> None:
        width = int(self.get_parameter('camera_width').value)
        height = int(self.get_parameter('camera_height').value)
        capture_fps = float(
            self.get_parameter('camera_capture_fps').value)
        pixel_format = str(
            self.get_parameter('camera_pixel_format').value)
        self._camera_frame_id = str(
            self.get_parameter('camera_frame_id').value)
        self._camera = self._cv2.VideoCapture(
            self._camera_device, self._cv2.CAP_V4L2)
        self._camera.set(self._cv2.CAP_PROP_FRAME_WIDTH, width)
        self._camera.set(self._cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._camera.set(self._cv2.CAP_PROP_FPS, capture_fps)
        if len(pixel_format) == 4:
            self._camera.set(
                self._cv2.CAP_PROP_FOURCC,
                self._cv2.VideoWriter_fourcc(*pixel_format))
        self._camera.set(self._cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._camera.isOpened():
            self._camera.release()
            self._camera = None
            raise RuntimeError(
                f'Cannot open V4L2 camera: {self._camera_device}')
        if bool(self.get_parameter('publish_input_image').value):
            self._input_publisher = self.create_publisher(
                Image, str(self.get_parameter('image_topic').value),
                qos_profile_sensor_data)
        timer_period = 0.001 if rate == 0.0 else 1.0 / rate
        self._camera_timer = self.create_timer(
            timer_period, self._capture_and_process)

    def _capture_and_process(self) -> None:
        received, bgr = self._camera.read()
        if not received or bgr is None:
            self._publish_invalid_free_space()
            self._publish_status(
                DiagnosticStatus.ERROR, 'USB camera frame unavailable', 0.0)
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._camera_frame_id
        if self._input_publisher is not None:
            input_message = array_to_image(bgr, 'bgr8')
            input_message.header = header
            self._input_publisher.publish(input_message)
        self._process_bgr(bgr, header)

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_processed < self._minimum_period:
            return
        self._last_processed = now
        started = time.perf_counter()
        try:
            bgr = image_to_bgr(message)
        except Exception as error:
            self._publish_invalid_free_space()
            self._publish_status(
                DiagnosticStatus.ERROR, str(error),
                (time.perf_counter() - started) * 1000.0)
            return
        self._process_bgr(bgr, message.header, started)

    def _process_bgr(
            self, bgr: np.ndarray, header: Header,
            started: float | None = None) -> None:
        if started is None:
            started = time.perf_counter()
        try:
            health = assess_camera_health(bgr)
            if not health.healthy:
                raise RuntimeError(health.reason)
            inference_started = time.perf_counter()
            raw = self._backend.infer(bgr, self._cv2)
            inference_ms = (
                time.perf_counter() - inference_started) * 1000.0
            relative_summary = summarize_relative_free_space(
                raw,
                roi_top_fraction=float(
                    self.get_parameter('roi_top_fraction').value),
                roi_bottom_fraction=float(
                    self.get_parameter('roi_bottom_fraction').value),
                near_percentile=float(
                    self.get_parameter('relative_near_percentile').value),
                minimum_contrast_span=float(self.get_parameter(
                    'relative_minimum_contrast_span').value),
            )
            self._publish_relative_free_space(relative_summary.as_list())
            if self._raw_publisher is not None:
                raw_message = array_to_image(raw, '32FC1')
                raw_message.header = header
                self._raw_publisher.publish(raw_message)
            if self._visualization_publisher is not None:
                visualization, _, _ = \
                    normalize_inverse_depth_for_display(raw)
                visualization_message = array_to_image(
                    visualization, 'mono8')
                visualization_message.header = header
                self._visualization_publisher.publish(
                    visualization_message)
            if self._metric_enabled:
                metric = inverse_depth_to_metric(
                    raw, self._inverse_depth_scale,
                    self._inverse_depth_shift)
                if self._metric_publisher is not None:
                    metric_message = array_to_image(metric, '32FC1')
                    metric_message.header = header
                    self._metric_publisher.publish(metric_message)
                if self._pointcloud_publisher is not None:
                    self._pointcloud_publisher.publish(
                        self._make_pointcloud(metric, header.stamp))
                summary = summarize_free_space(
                    metric,
                    minimum_depth_m=float(
                        self.get_parameter('minimum_depth_m').value),
                    maximum_depth_m=float(
                        self.get_parameter('maximum_depth_m').value),
                    near_percentile=float(
                        self.get_parameter('near_percentile').value),
                    roi_top_fraction=float(
                        self.get_parameter('roi_top_fraction').value),
                    roi_bottom_fraction=float(
                        self.get_parameter('roi_bottom_fraction').value),
                )
                self._publish_free_space(summary.as_list())
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
                (completed - started) * 1000.0,
                inference_ms)
        except Exception as error:
            self._publish_invalid_free_space()
            self._publish_status(
                DiagnosticStatus.ERROR, str(error),
                (time.perf_counter() - started) * 1000.0)

    def _publish_invalid_free_space(self) -> None:
        self._publish_free_space([float('nan')] * 4 + [0.0])

    def _publish_free_space(self, values) -> None:
        message = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = 'left,center,right,nearest,valid_fraction'
        dimension.size = 5
        dimension.stride = 5
        message.layout.dim = [dimension]
        message.data = list(values)
        self._free_space_publisher.publish(message)

    def _publish_relative_free_space(self, values) -> None:
        message = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = (
            'left,center,right,nearest,valid_fraction;'
            'units=relative_clearance_0_to_1')
        dimension.size = 5
        dimension.stride = 5
        message.layout.dim = [dimension]
        message.data = list(values)
        self._relative_free_space_publisher.publish(message)

    def _make_pointcloud(self, depth: np.ndarray, stamp) -> PointCloud2:
        principal_x = self._principal_x
        principal_y = self._principal_y
        if principal_x < 0.0:
            principal_x = depth.shape[1] / 2.0
        if principal_y < 0.0:
            principal_y = depth.shape[0] / 2.0
        points = depth_to_flu_points(
            depth,
            focal_x_px=self._focal_x,
            focal_y_px=self._focal_y,
            principal_x_px=principal_x,
            principal_y_px=principal_y,
            stride=self._pointcloud_stride,
            minimum_depth_m=float(
                self.get_parameter('minimum_depth_m').value),
            maximum_depth_m=float(
                self.get_parameter('maximum_depth_m').value),
        )
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = self._pointcloud_frame_id
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = [
            PointField(
                name='x', offset=0,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='y', offset=4,
                datatype=PointField.FLOAT32, count=1),
            PointField(
                name='z', offset=8,
                datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * message.width
        message.data = points.tobytes()
        message.is_dense = True
        return message

    def _publish_status(
            self, level: int, text: str, total_ms: float,
            inference_ms: float = float('nan')) -> None:
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'px4_uavcup/zipdepth_pi'
        status.hardware_id = 'raspberry_pi_5'
        status.level = level
        status.message = text
        status.values = [
            KeyValue(key='inference_ms', value=f'{inference_ms:.2f}'),
            KeyValue(key='total_ms', value=f'{total_ms:.2f}'),
            KeyValue(key='metric_calibration_enabled',
                     value=str(self._metric_enabled).lower()),
            KeyValue(
                key='processing_fps', value=f'{self._processing_fps:.2f}'),
            KeyValue(key='output_width', value=str(self._backend.width)),
            KeyValue(key='output_height', value=str(self._backend.height)),
            KeyValue(key='input_source', value=self._input_source),
            KeyValue(key='raw_enabled', value=str(self._publish_raw).lower()),
            KeyValue(
                key='visualization_enabled',
                value=str(self._publish_visualization).lower()),
            KeyValue(
                key='pointcloud_enabled',
                value=str(self._publish_pointcloud).lower()),
        ]
        diagnostic.status = [status]
        self._status_publisher.publish(diagnostic)

    def destroy_node(self):
        if self._camera is not None:
            self._camera.release()
            self._camera = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZipDepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

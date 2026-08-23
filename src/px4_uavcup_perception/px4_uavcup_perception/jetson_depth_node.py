#!/usr/bin/env python3
"""Low-overhead TensorRT depth and free-space pipeline for Jetson flight."""

from __future__ import annotations

import math
import os
from pathlib import Path
import time
from typing import Optional

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .free_space import FreeSpaceSummary, summarize_free_space
from .image_utils import array_to_image
from .jetson_tensorrt_backend import DepthAnythingTensorRT
from .pointcloud_utils import depth_to_flu_points


def _expanded_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def _depth_visualization(depth_m: np.ndarray) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    output = np.zeros(depth_m.shape, dtype=np.uint8)
    if not np.any(valid):
        return output
    low, high = np.percentile(depth_m[valid], [2.0, 98.0])
    if high <= low:
        output[valid] = 255
        return output
    normalized = np.clip((depth_m - low) / (high - low), 0.0, 1.0)
    output[valid] = ((1.0 - normalized[valid]) * 255.0).astype(np.uint8)
    return output


class JetsonDepthNode(Node):
    """Capture the front camera, run TensorRT and publish control data."""

    def __init__(self) -> None:
        super().__init__('jetson_depth_node')

        self.declare_parameter(
            'engine_path',
            '~/Depth-Anything-V2/metric_depth/'
            'depth_anything_v2_metric_hypersim_vits_364_fp16.engine')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('use_gstreamer', True)
        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('capture_fps', 30)
        self.declare_parameter('model_input_size', 364)
        self.declare_parameter('processing_rate_hz', 30.0)
        self.declare_parameter('warmup_iterations', 3)

        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('minimum_depth_m', 0.3)
        self.declare_parameter('maximum_depth_m', 8.0)
        self.declare_parameter('near_percentile', 15.0)
        self.declare_parameter('roi_top_fraction', 0.25)
        self.declare_parameter('roi_bottom_fraction', 0.78)
        self.declare_parameter('minimum_valid_fraction', 0.25)

        self.declare_parameter('free_space_topic', '/uav/depth/free_space')
        self.declare_parameter('status_topic', '/uav/depth/status')
        self.declare_parameter('depth_topic', '/camera/depth/image')
        self.declare_parameter(
            'visualization_topic', '/uav/depth/visualization')
        self.declare_parameter('pointcloud_topic', '/camera/depth/points')
        self.declare_parameter('publish_depth_image', False)
        self.declare_parameter('publish_visualization', False)
        self.declare_parameter('publish_pointcloud', False)

        self.declare_parameter('pointcloud_stride', 4)
        self.declare_parameter('focal_x_px', 300.0)
        self.declare_parameter('focal_y_px', 300.0)
        self.declare_parameter('principal_x_px', -1.0)
        self.declare_parameter('principal_y_px', -1.0)

        self._minimum_depth = float(
            self.get_parameter('minimum_depth_m').value)
        self._maximum_depth = float(
            self.get_parameter('maximum_depth_m').value)
        self._near_percentile = float(
            self.get_parameter('near_percentile').value)
        self._roi_top = float(
            self.get_parameter('roi_top_fraction').value)
        self._roi_bottom = float(
            self.get_parameter('roi_bottom_fraction').value)
        self._minimum_valid_fraction = float(
            self.get_parameter('minimum_valid_fraction').value)
        self._frame_id = str(self.get_parameter('camera_frame_id').value)
        self._pointcloud_stride = int(
            self.get_parameter('pointcloud_stride').value)
        self._focal_x = float(self.get_parameter('focal_x_px').value)
        self._focal_y = float(self.get_parameter('focal_y_px').value)
        self._principal_x = float(
            self.get_parameter('principal_x_px').value)
        self._principal_y = float(
            self.get_parameter('principal_y_px').value)
        self._publish_depth = bool(
            self.get_parameter('publish_depth_image').value)
        self._publish_visualization = bool(
            self.get_parameter('publish_visualization').value)
        self._publish_pointcloud = bool(
            self.get_parameter('publish_pointcloud').value)

        processing_rate = float(
            self.get_parameter('processing_rate_hz').value)
        if processing_rate <= 0.0:
            raise ValueError('processing_rate_hz must be positive')

        self._free_space_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter('free_space_topic').value),
            1,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._depth_publisher: Optional[object] = None
        self._visualization_publisher: Optional[object] = None
        self._pointcloud_publisher: Optional[object] = None
        if self._publish_depth:
            self._depth_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('depth_topic').value),
                qos_profile_sensor_data,
            )
        if self._publish_visualization:
            self._visualization_publisher = self.create_publisher(
                Image,
                str(self.get_parameter('visualization_topic').value),
                qos_profile_sensor_data,
            )
        if self._publish_pointcloud:
            self._pointcloud_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter('pointcloud_topic').value),
                qos_profile_sensor_data,
            )

        self._cv2 = self._import_opencv()
        self._capture = self._open_camera()
        self._backend = self._load_backend()
        self._warm_up_backend()

        self._frames_since_report = 0
        self._report_started = time.monotonic()
        self._last_error_status = float('-inf')
        self._camera_failures = 0
        self._timer = self.create_timer(
            1.0 / processing_rate, self._process_latest_frame)
        self.get_logger().info(
            'Jetson flight perception ready: direct free-space enabled, '
            f'depth={self._publish_depth}, visualization='
            f'{self._publish_visualization}, pointcloud='
            f'{self._publish_pointcloud}')

    @staticmethod
    def _import_opencv():
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError('OpenCV Python is required on Jetson') from error
        return cv2

    def _load_backend(self):
        engine_path = _expanded_path(
            str(self.get_parameter('engine_path').value))
        self.get_logger().info(f'Loading TensorRT engine: {engine_path}')
        return DepthAnythingTensorRT(
            engine_path,
            input_size=int(self.get_parameter('model_input_size').value),
        )

    def _default_gstreamer_pipeline(self) -> str:
        device = str(self.get_parameter('camera_device').value)
        width = int(self.get_parameter('capture_width').value)
        height = int(self.get_parameter('capture_height').value)
        fps = int(self.get_parameter('capture_fps').value)
        model_size = int(self.get_parameter('model_input_size').value)
        return (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height},framerate={fps}/1 ! '
            'jpegdec ! videoscale ! '
            f'video/x-raw,width={model_size},height={model_size} ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=1 max-buffers=1 sync=false'
        )

    def _open_camera(self):
        device = str(self.get_parameter('camera_device').value)
        use_gstreamer = bool(self.get_parameter('use_gstreamer').value)
        pipeline = str(self.get_parameter('gstreamer_pipeline').value)
        if not pipeline:
            pipeline = self._default_gstreamer_pipeline()

        capture = None
        if use_gstreamer:
            capture = self._cv2.VideoCapture(
                pipeline, self._cv2.CAP_GSTREAMER)
            if capture.isOpened():
                self.get_logger().info(
                    f'Camera opened with GStreamer: {device}')
                return capture
            capture.release()
            self.get_logger().warning(
                'GStreamer camera open failed; trying V4L2 fallback')

        try:
            device_index = int(device)
        except ValueError:
            if not device.startswith('/dev/video'):
                raise RuntimeError(f'Unsupported camera device: {device}')
            device_index = int(device[len('/dev/video'):])
        capture = self._cv2.VideoCapture(device_index)
        capture.set(
            self._cv2.CAP_PROP_FRAME_WIDTH,
            int(self.get_parameter('capture_width').value))
        capture.set(
            self._cv2.CAP_PROP_FRAME_HEIGHT,
            int(self.get_parameter('capture_height').value))
        capture.set(
            self._cv2.CAP_PROP_FPS,
            int(self.get_parameter('capture_fps').value))
        capture.set(self._cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f'Cannot open front camera: {device}')
        self.get_logger().info(f'Camera opened with V4L2: {device}')
        return capture

    def _warm_up_backend(self) -> None:
        iterations = max(
            0, int(self.get_parameter('warmup_iterations').value))
        size = int(self.get_parameter('model_input_size').value)
        image = np.zeros((size, size, 3), dtype=np.uint8)
        for _ in range(iterations):
            self._backend.infer(image, self._cv2)
        if iterations:
            self.get_logger().info(
                f'TensorRT warm-up complete ({iterations} iterations)')

    def _process_latest_frame(self) -> None:
        callback_started = time.perf_counter()
        received, frame = self._capture.read()
        if not received or frame is None:
            self._camera_failures += 1
            self.get_logger().error(
                'Front camera frame unavailable', throttle_duration_sec=2.0)
            self._publish_status(
                DiagnosticStatus.ERROR,
                'camera frame unavailable',
                0.0,
                0.0,
            )
            return

        model_size = int(self.get_parameter('model_input_size').value)
        if frame.shape[:2] != (model_size, model_size):
            frame = self._cv2.resize(frame, (model_size, model_size))

        inference_started = time.perf_counter()
        try:
            depth = np.asarray(
                self._backend.infer(frame, self._cv2), dtype=np.float32)
            if depth.ndim != 2 or depth.size == 0:
                raise ValueError(
                    f'Unexpected TensorRT depth shape: {depth.shape}')
            summary = summarize_free_space(
                depth,
                minimum_depth_m=self._minimum_depth,
                maximum_depth_m=self._maximum_depth,
                near_percentile=self._near_percentile,
                roi_top_fraction=self._roi_top,
                roi_bottom_fraction=self._roi_bottom,
            )
        except Exception as error:
            self.get_logger().error(
                f'Depth inference failed: {error}',
                throttle_duration_sec=2.0)
            self._publish_status(
                DiagnosticStatus.ERROR, 'inference failed', 0.0, 0.0)
            return
        inference_ms = (time.perf_counter() - inference_started) * 1000.0
        stamp = self.get_clock().now().to_msg()

        self._publish_free_space(summary)
        if self._depth_publisher is not None:
            message = array_to_image(depth, '32FC1')
            message.header.stamp = stamp
            message.header.frame_id = self._frame_id
            self._depth_publisher.publish(message)
        if self._visualization_publisher is not None:
            message = array_to_image(
                _depth_visualization(depth), 'mono8')
            message.header.stamp = stamp
            message.header.frame_id = self._frame_id
            self._visualization_publisher.publish(message)
        if self._pointcloud_publisher is not None:
            self._pointcloud_publisher.publish(
                self._make_pointcloud(depth, stamp))

        total_ms = (time.perf_counter() - callback_started) * 1000.0
        self._frames_since_report += 1
        now = time.monotonic()
        report_period = now - self._report_started
        if report_period >= 2.0:
            fps = self._frames_since_report / report_period
            sectors_valid = all(math.isfinite(value) for value in (
                summary.left_m, summary.center_m, summary.right_m))
            healthy = (
                sectors_valid
                and summary.valid_fraction >= self._minimum_valid_fraction)
            level = DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN
            status_message = (
                'running' if healthy else 'insufficient valid depth')
            self._publish_status(
                level,
                status_message,
                inference_ms,
                fps,
                summary,
                total_ms,
            )
            self.get_logger().info(
                f'FPS={fps:.1f} inference={inference_ms:.1f} ms '
                f'total={total_ms:.1f} ms nearest='
                f'{summary.nearest_m:.2f} m valid='
                f'{summary.valid_fraction:.2f}')
            self._frames_since_report = 0
            self._report_started = now

    def _publish_free_space(self, summary: FreeSpaceSummary) -> None:
        output = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = 'left,center,right,nearest,valid_fraction'
        dimension.size = 5
        dimension.stride = 5
        output.layout.dim = [dimension]
        output.data = summary.as_list()
        self._free_space_publisher.publish(output)

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
            minimum_depth_m=self._minimum_depth,
            maximum_depth_m=self._maximum_depth,
        )

        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
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
            self,
            level: int,
            message: str,
            inference_ms: float,
            fps: float,
            summary: Optional[FreeSpaceSummary] = None,
            total_ms: float = 0.0) -> None:
        now = time.monotonic()
        if level == DiagnosticStatus.ERROR:
            if now - self._last_error_status < 1.0:
                return
            self._last_error_status = now
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = 'px4_uavcup/jetson_depth'
        status.hardware_id = 'jetson_xavier_nx'
        status.message = message
        values = [
            KeyValue(key='inference_ms', value=f'{inference_ms:.2f}'),
            KeyValue(key='total_ms', value=f'{total_ms:.2f}'),
            KeyValue(key='fps', value=f'{fps:.2f}'),
            KeyValue(
                key='camera_failures', value=str(self._camera_failures)),
        ]
        if summary is not None:
            values.extend([
                KeyValue(
                    key='nearest_m', value=f'{summary.nearest_m:.3f}'),
                KeyValue(
                    key='valid_fraction',
                    value=f'{summary.valid_fraction:.3f}'),
            ])
        status.values = values
        array.status = [status]
        self._status_publisher.publish(array)

    def destroy_node(self) -> None:
        if hasattr(self, '_capture') and self._capture.isOpened():
            self._capture.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonDepthNode()
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

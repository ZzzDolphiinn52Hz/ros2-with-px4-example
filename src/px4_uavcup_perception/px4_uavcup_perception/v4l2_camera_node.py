#!/usr/bin/env python3
"""Low-latency V4L2 image publisher for the front USB camera."""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .image_utils import array_to_image


class V4l2CameraNode(Node):
    """Publish a USB camera without requiring cv_bridge or extra ROS drivers."""

    def __init__(self) -> None:
        super().__init__('front_usb_camera')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('capture_fps', 30.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'front_camera_optical_frame')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('pixel_format', 'MJPG')

        try:
            import cv2
        except ImportError as error:
            raise RuntimeError('python3-opencv is required') from error
        self._cv2 = cv2
        self._frame_id = str(self.get_parameter('frame_id').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        device = str(self.get_parameter('device').value)
        self._capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH,
                          int(self.get_parameter('width').value))
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT,
                          int(self.get_parameter('height').value))
        self._capture.set(cv2.CAP_PROP_FPS,
                          float(self.get_parameter('capture_fps').value))
        pixel_format = str(self.get_parameter('pixel_format').value)
        if len(pixel_format) == 4:
            self._capture.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._capture.isOpened():
            self._capture.release()
            raise RuntimeError(f'Cannot open V4L2 camera: {device}')

        self._images = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data)
        self._status = self.create_publisher(
            DiagnosticArray, '/uav/camera/front/status', 10)
        self._last_frame = time.monotonic()
        self._frames = 0
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._status_timer = self.create_timer(2.0, self._publish_status)
        self.get_logger().info(
            f'Front USB camera ready: {device} -> '
            f'{self.get_parameter("image_topic").value}')

    def _publish(self) -> None:
        received, frame = self._capture.read()
        if not received or frame is None:
            self.get_logger().error(
                'USB camera frame unavailable', throttle_duration_sec=1.0)
            return
        message = array_to_image(frame, 'bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        self._images.publish(message)
        self._last_frame = time.monotonic()
        self._frames += 1

    def _publish_status(self) -> None:
        age = time.monotonic() - self._last_frame
        status = DiagnosticStatus()
        status.name = 'front_usb_camera'
        status.hardware_id = str(self.get_parameter('device').value)
        status.level = (
            DiagnosticStatus.OK if age < 0.5 else DiagnosticStatus.ERROR)
        status.message = 'streaming' if age < 0.5 else 'frame_timeout'
        status.values = [
            KeyValue(key='last_frame_age_s', value=f'{age:.3f}'),
            KeyValue(key='frames_total', value=str(self._frames)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status.publish(array)

    def destroy_node(self):
        if getattr(self, '_capture', None) is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V4l2CameraNode()
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

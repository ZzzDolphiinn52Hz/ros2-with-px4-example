#!/usr/bin/env python3

"""Publish front-camera frames for one-time intrinsic calibration."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .image_utils import array_to_image


class CameraCalibrationPublisher(Node):
    """Keep calibration capture separate from the flight perception path."""

    def __init__(self) -> None:
        super().__init__('camera_calibration_publisher')

        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('capture_width', 1280)
        self.declare_parameter('capture_height', 720)
        self.declare_parameter('capture_fps', 30)
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 360)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('gstreamer_pipeline', '')

        self._cv2 = self._import_cv2()
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._capture = self._open_camera()
        self._publisher = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            1,
        )

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._timer = self.create_timer(1.0 / publish_rate, self._publish)
        self._report_started = time.monotonic()
        self._published_since_report = 0

        width = int(self.get_parameter('output_width').value)
        height = int(self.get_parameter('output_height').value)
        topic = str(self.get_parameter('image_topic').value)
        self.get_logger().info(
            f'Calibration camera ready: {width}x{height} -> {topic}. '
            'Do not run flight perception at the same time.')

    @staticmethod
    def _import_cv2():
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError('python3-opencv is required') from error
        return cv2

    def _default_pipeline(self) -> str:
        device = str(self.get_parameter('camera_device').value)
        width = int(self.get_parameter('capture_width').value)
        height = int(self.get_parameter('capture_height').value)
        fps = int(self.get_parameter('capture_fps').value)
        output_width = int(self.get_parameter('output_width').value)
        output_height = int(self.get_parameter('output_height').value)
        return (
            f'v4l2src device={device} ! '
            f'image/jpeg,width={width},height={height},framerate={fps}/1 ! '
            'jpegdec ! videoscale ! '
            f'video/x-raw,width={output_width},height={output_height} ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink drop=1 max-buffers=1 sync=false'
        )

    def _open_camera(self):
        pipeline = str(self.get_parameter('gstreamer_pipeline').value)
        if not pipeline:
            pipeline = self._default_pipeline()
        capture = self._cv2.VideoCapture(pipeline, self._cv2.CAP_GSTREAMER)
        if not capture.isOpened():
            capture.release()
            device = str(self.get_parameter('camera_device').value)
            raise RuntimeError(f'Cannot open calibration camera: {device}')
        return capture

    def _publish(self) -> None:
        received, frame = self._capture.read()
        if not received or frame is None:
            self.get_logger().error(
                'Calibration camera frame unavailable',
                throttle_duration_sec=1.0,
            )
            return

        message = array_to_image(frame, 'bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)
        self._published_since_report += 1

        elapsed = time.monotonic() - self._report_started
        if elapsed >= 2.0:
            rate = self._published_since_report / elapsed
            self.get_logger().info(
                f'Calibration image stream: {rate:.1f} FPS, '
                f'{frame.shape[1]}x{frame.shape[0]}')
            self._report_started = time.monotonic()
            self._published_since_report = 0

    def destroy_node(self):
        if getattr(self, '_capture', None) is not None:
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraCalibrationPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ROS launch can deliver a second SIGINT while OpenCV releases
                # the V4L2 pipeline. Process exit will release the descriptor.
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

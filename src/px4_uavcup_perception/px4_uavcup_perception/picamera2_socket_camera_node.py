#!/usr/bin/env python3
"""Publish Picamera2 host frames and calibrated CameraInfo in ROS 2."""

from __future__ import annotations

import socket
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .aruco_geometry import camera_matrix
from .picamera2_socket_protocol import HEADER, receive_exact, unpack_header


class Picamera2SocketCameraNode(Node):
    """Bridge raw RGB frames from Raspberry Pi OS into the ROS container."""

    def __init__(self) -> None:
        super().__init__('picamera2_socket_camera_node')
        self.declare_parameter(
            'socket_path', '/ros2_ws/run/down_camera.sock')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'down_camera_optical_frame')
        self.declare_parameter('image_topic', '/camera/down/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/down/camera_info')
        self.declare_parameter('distortion_model', 'plumb_bob')
        self.declare_parameter('camera_matrix', [0.0] * 9)
        self.declare_parameter('distortion_coefficients', [0.0] * 5)

        self._socket_path = str(self.get_parameter('socket_path').value)
        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        if self._width <= 0 or self._height <= 0:
            raise ValueError('camera width and height must be positive')
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._camera_matrix = camera_matrix(
            self.get_parameter('camera_matrix').value)
        self._distortion = np.asarray(
            self.get_parameter('distortion_coefficients').value,
            dtype=np.float64,
        )
        if (self._distortion.size < 4 or
                not np.all(np.isfinite(self._distortion))):
            raise ValueError(
                'distortion_coefficients must contain at least 4 finite '
                'values')
        self._distortion_model = str(
            self.get_parameter('distortion_model').value)

        self._image_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data,
        )
        self._info_publisher = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            qos_profile_sensor_data,
        )
        self._stop = threading.Event()
        self._connection = None
        self._connection_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._receive_loop,
            name='picamera2_socket_receiver',
            daemon=True,
        )
        self._thread.start()
        self.get_logger().info(
            f'Waiting for Picamera2 host stream: {self._socket_path}')

    def _camera_info(self, stamp) -> CameraInfo:
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.width = self._width
        message.height = self._height
        message.distortion_model = self._distortion_model
        message.d = self._distortion.tolist()
        message.k = self._camera_matrix.reshape(-1).tolist()
        message.r = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
        message.p = [
            float(self._camera_matrix[0, 0]), 0.0,
            float(self._camera_matrix[0, 2]), 0.0,
            0.0, float(self._camera_matrix[1, 1]),
            float(self._camera_matrix[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return message

    def _publish_frame(self, width: int, height: int, payload: bytes) -> None:
        if width != self._width or height != self._height:
            raise ValueError(
                f'host frame {width}x{height} does not match calibrated '
                f'resolution {self._width}x{self._height}')
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        image.height = height
        image.width = width
        image.encoding = 'rgb8'
        image.is_bigendian = False
        image.step = width * 3
        image.data = payload
        self._image_publisher.publish(image)
        self._info_publisher.publish(self._camera_info(stamp))

    def _receive_loop(self) -> None:
        report_started = time.monotonic()
        frames_since_report = 0
        warning_started = float('-inf')
        while not self._stop.is_set():
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.settimeout(2.0)
            try:
                connection.connect(self._socket_path)
                with self._connection_lock:
                    self._connection = connection
                self.get_logger().info('Connected to Picamera2 host stream')
                while not self._stop.is_set():
                    header = receive_exact(connection, HEADER.size)
                    width, height, payload_size = unpack_header(header)
                    payload = receive_exact(connection, payload_size)
                    self._publish_frame(width, height, payload)
                    frames_since_report += 1
                    elapsed = time.monotonic() - report_started
                    if elapsed >= 2.0:
                        self.get_logger().info(
                            f'Down camera stream: '
                            f'{frames_since_report / elapsed:.1f} FPS, '
                            f'{width}x{height}')
                        report_started = time.monotonic()
                        frames_since_report = 0
            except (ConnectionError, OSError, ValueError) as error:
                now = time.monotonic()
                if not self._stop.is_set() and now - warning_started >= 5.0:
                    self.get_logger().warning(
                        f'Picamera2 host stream unavailable: {error}')
                    warning_started = now
                self._stop.wait(1.0)
            finally:
                with self._connection_lock:
                    if self._connection is connection:
                        self._connection = None
                connection.close()

    def destroy_node(self):
        self._stop.set()
        with self._connection_lock:
            if self._connection is not None:
                try:
                    self._connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self._connection.close()
                self._connection = None
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Picamera2SocketCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

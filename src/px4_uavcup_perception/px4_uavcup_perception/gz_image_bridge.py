#!/usr/bin/env python3
"""Bridge a Gazebo Harmonic camera and clock to ROS 2.

Ubuntu's ROS 2 Humble bridge targets Gazebo Fortress, while this PX4 checkout
uses Gazebo Harmonic. The matching ``gz.transport13`` Python bindings avoid
mixing incompatible Gazebo message versions.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.image_pb2 import (
    BGRA_INT8,
    BGR_INT8,
    Image as GzImage,
    L_INT8,
    RGBA_INT8,
    RGB_INT8,
)
from gz.transport13 import Node as GzNode
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import CameraInfo, Image


_GZ_ENCODINGS = {
    L_INT8: ('mono8', 1),
    RGB_INT8: ('rgb8', 3),
    RGBA_INT8: ('rgba8', 4),
    BGR_INT8: ('bgr8', 3),
    BGRA_INT8: ('bgra8', 4),
}


class GazeboImageBridge(Node):
    def __init__(self) -> None:
        super().__init__('gz_image_bridge')
        self.declare_parameter(
            'gz_image_topic',
            '/world/urban_uavcup/model/x500_uavcup_0/'
            'link/front_camera_link/sensor/front_imager/image',
        )
        self.declare_parameter('gz_clock_topic', '/clock')
        self.declare_parameter(
            'ros_image_topic', '/uav/front_camera/image_raw')
        self.declare_parameter(
            'ros_camera_info_topic', '/uav/front_camera/camera_info')
        self.declare_parameter('frame_id', 'front_camera_link')
        self.declare_parameter('horizontal_fov_rad', 1.74)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('output_width', 320)
        self.declare_parameter('output_height', 240)

        self._gz_image_topic = str(
            self.get_parameter('gz_image_topic').value)
        self._gz_clock_topic = str(
            self.get_parameter('gz_clock_topic').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._horizontal_fov = float(
            self.get_parameter('horizontal_fov_rad').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._publish_clock = bool(self.get_parameter('publish_clock').value)
        self._output_width = int(self.get_parameter('output_width').value)
        self._output_height = int(self.get_parameter('output_height').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self._output_width < 0 or self._output_height < 0:
            raise ValueError('Output image dimensions cannot be negative')

        self._image_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('ros_image_topic').value),
            qos_profile_sensor_data,
        )
        self._info_publisher = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('ros_camera_info_topic').value),
            qos_profile_sensor_data,
        )
        self._clock_publisher = self.create_publisher(RosClock, '/clock', 10)

        self._lock = threading.Lock()
        self._latest_image: Optional[GzImage] = None
        self._latest_clock: Optional[GzClock] = None
        self._received = 0
        self._published = 0
        self._unsupported = 0

        self._gz = GzNode()
        self._gz.subscribe(
            GzImage, self._gz_image_topic, self._on_gz_image)
        if self._publish_clock:
            self._gz.subscribe(
                GzClock, self._gz_clock_topic, self._on_gz_clock)

        wall_clock = RclpyClock(clock_type=ClockType.SYSTEM_TIME)
        self.create_timer(1.0 / publish_rate, self._publish, clock=wall_clock)
        self.create_timer(2.0, self._heartbeat, clock=wall_clock)
        self.get_logger().info(
            f'Subscribed to Gazebo image: {self._gz_image_topic}')

    def _on_gz_image(self, message: GzImage) -> None:
        snapshot = GzImage()
        snapshot.CopyFrom(message)
        with self._lock:
            self._latest_image = snapshot
            self._received += 1

    def _on_gz_clock(self, message: GzClock) -> None:
        snapshot = GzClock()
        snapshot.CopyFrom(message)
        with self._lock:
            self._latest_clock = snapshot

    @staticmethod
    def _clock_stamp(clock: Optional[GzClock]) -> Time:
        stamp = Time()
        if clock is None:
            return stamp
        source = clock.sim
        if int(source.sec) == 0 and int(source.nsec) == 0:
            source = clock.system
        stamp.sec = int(source.sec)
        stamp.nanosec = int(source.nsec)
        return stamp

    def _publish(self) -> None:
        with self._lock:
            image = self._latest_image
            clock = self._latest_clock
            self._latest_image = None

        stamp = self._clock_stamp(clock)
        if self._publish_clock and clock is not None:
            clock_message = RosClock()
            clock_message.clock = stamp
            self._clock_publisher.publish(clock_message)

        if image is None:
            return
        encoding = _GZ_ENCODINGS.get(int(image.pixel_format_type))
        if encoding is None:
            with self._lock:
                self._unsupported += 1
            self.get_logger().error(
                'Unsupported Gazebo pixel format: '
                f'{int(image.pixel_format_type)}', throttle_duration_sec=5.0)
            return

        encoding_name, channels = encoding
        width = int(image.width)
        height = int(image.height)
        expected_step = width * channels
        source_step = int(image.step) or expected_step
        if (
                source_step < expected_step
                or len(image.data) < height * source_step):
            self.get_logger().error(
                'Malformed Gazebo image buffer', throttle_duration_sec=5.0)
            return

        output = Image()
        output.header.stamp = stamp
        output.header.frame_id = self._frame_id
        output.encoding = encoding_name
        output.is_bigendian = False
        resize = (
            self._output_width > 0
            and self._output_height > 0
            and (self._output_width != width or self._output_height != height)
        )
        if resize:
            output.width = self._output_width
            output.height = self._output_height
            output.step = self._output_width * channels
            output.data = self._resize_nearest(
                image.data,
                width,
                height,
                source_step,
                channels,
                self._output_width,
                self._output_height,
            )
        else:
            output.width = width
            output.height = height
            output.step = source_step
            output.data = bytes(image.data[:height * source_step])
        self._image_publisher.publish(output)
        self._info_publisher.publish(self._camera_info(output))
        with self._lock:
            self._published += 1

    @staticmethod
    def _resize_nearest(
            data: bytes,
            width: int,
            height: int,
            step: int,
            channels: int,
            output_width: int,
            output_height: int) -> bytes:
        rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
        pixels = rows[:, :width * channels]
        pixels = pixels.reshape(height, width, channels)
        row_indices = np.linspace(
            0, height - 1, output_height, dtype=np.intp)
        column_indices = np.linspace(
            0, width - 1, output_width, dtype=np.intp)
        resized = pixels[row_indices][:, column_indices]
        return np.ascontiguousarray(resized).tobytes()

    def _camera_info(self, image: Image) -> CameraInfo:
        width = float(image.width)
        height = float(image.height)
        focal = width / (2.0 * math.tan(self._horizontal_fov / 2.0))
        cx = (width - 1.0) / 2.0
        cy = (height - 1.0) / 2.0
        info = CameraInfo()
        info.header = image.header
        info.width = image.width
        info.height = image.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0,
                  0.0, focal, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _heartbeat(self) -> None:
        with self._lock:
            received = self._received
            published = self._published
            unsupported = self._unsupported
            self._received = 0
            self._published = 0
            self._unsupported = 0
        self.get_logger().info(
            'Camera last 2s: '
            f'GZ={received} ROS={published} unsupported={unsupported}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboImageBridge()
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

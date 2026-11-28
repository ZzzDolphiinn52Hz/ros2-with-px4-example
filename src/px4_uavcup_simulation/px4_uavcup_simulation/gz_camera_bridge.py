#!/usr/bin/env python3
"""Bridge a Gazebo Harmonic camera to ROS 2 without ros_gz ABI mixing.

Ubuntu 22.04 ROS Humble normally ships a Fortress-era ros_gz_bridge, while
PX4 v1.17 on this host uses Gazebo Harmonic / gz-transport13. Mixing their
discovery protocols produces repeated ``Unknown message type`` errors. This
small one-way adapter uses the matching Harmonic Python bindings and publishes
the same sensor_msgs contract consumed by the production detector.
"""

from __future__ import annotations

from array import array
import threading
from typing import Dict, Tuple

from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.image_pb2 import PixelFormatType
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image


PIXEL_FORMATS: Dict[int, Tuple[str, int]] = {
    PixelFormatType.L_INT8: ('mono8', 1),
    PixelFormatType.L_INT16: ('mono16', 2),
    PixelFormatType.RGB_INT8: ('rgb8', 3),
    PixelFormatType.RGBA_INT8: ('rgba8', 4),
    PixelFormatType.BGR_INT8: ('bgr8', 3),
    PixelFormatType.BGRA_INT8: ('bgra8', 4),
    PixelFormatType.BAYER_RGGB8: ('bayer_rggb8', 1),
    PixelFormatType.BAYER_BGGR8: ('bayer_bggr8', 1),
    PixelFormatType.BAYER_GBRG8: ('bayer_gbrg8', 1),
    PixelFormatType.BAYER_GRBG8: ('bayer_grbg8', 1),
}


def _frame_id(gz_header, fallback: str) -> str:
    for entry in gz_header.data:
        if entry.key == 'frame_id' and entry.value:
            return str(entry.value[0])
    return fallback


def _copy_header(gz_header, ros_header, fallback_frame_id: str) -> None:
    ros_header.stamp.sec = int(gz_header.stamp.sec)
    ros_header.stamp.nanosec = int(gz_header.stamp.nsec)
    ros_header.frame_id = _frame_id(gz_header, fallback_frame_id)


def image_to_ros(message: GzImage, fallback_frame_id: str) -> Image:
    """Convert a supported gz.msgs.Image into sensor_msgs/Image."""
    format_info = PIXEL_FORMATS.get(int(message.pixel_format_type))
    if format_info is None:
        raise ValueError(
            f'unsupported Gazebo pixel format: '
            f'{int(message.pixel_format_type)}')
    encoding, bytes_per_pixel = format_info
    expected_step = int(message.width) * bytes_per_pixel
    step = int(message.step) or expected_step
    expected_size = step * int(message.height)
    data = message.data
    if step < expected_step or len(data) != expected_size:
        raise ValueError(
            f'invalid Gazebo image buffer: {message.width}x{message.height}, '
            f'step={step}, bytes={len(data)}')

    output = Image()
    _copy_header(message.header, output.header, fallback_frame_id)
    output.width = int(message.width)
    output.height = int(message.height)
    output.encoding = encoding
    output.is_bigendian = 0
    output.step = step
    # An unsigned-byte array takes the ROS setter's bulk-copy path.
    output.data = array('B', data)
    return output


def camera_info_to_ros(
        message: GzCameraInfo, fallback_frame_id: str) -> CameraInfo:
    """Convert gz.msgs.CameraInfo into sensor_msgs/CameraInfo."""
    output = CameraInfo()
    _copy_header(message.header, output.header, fallback_frame_id)
    output.width = int(message.width)
    output.height = int(message.height)
    output.distortion_model = 'plumb_bob'
    output.d = [float(value) for value in message.distortion.k]
    if len(message.intrinsics.k) == 9:
        output.k = [float(value) for value in message.intrinsics.k]
    if len(message.rectification_matrix) == 9:
        output.r = [float(value) for value in message.rectification_matrix]
    if len(message.projection.p) == 12:
        output.p = [float(value) for value in message.projection.p]
    return output


class GazeboCameraBridge(Node):
    """One-way Gazebo Harmonic to ROS 2 camera bridge."""

    def __init__(self) -> None:
        super().__init__('gazebo_down_camera_bridge')
        self.declare_parameter('gz_image_topic', '/camera/down/image_raw')
        self.declare_parameter('gz_clock_topic', '/world/aruco_detector/clock')
        self.declare_parameter(
            'gz_camera_info_topic', '/camera/down/camera_info')
        self.declare_parameter('ros_image_topic', '/camera/down/image_raw')
        self.declare_parameter(
            'ros_camera_info_topic', '/camera/down/camera_info')
        self.declare_parameter(
            'fallback_frame_id', 'down_camera_optical_frame')

        self._fallback_frame_id = str(
            self.get_parameter('fallback_frame_id').value)
        self._image_pub = self.create_publisher(
            Image, str(self.get_parameter('ros_image_topic').value),
            qos_profile_sensor_data)
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('ros_camera_info_topic').value),
            qos_profile_sensor_data)
        self._clock_pub = self.create_publisher(
            Clock, '/uav/sim/clock', 1)
        # Transport callbacks only replace pending messages. All ROS publishing
        # runs on the executor, outside the lock.
        self._callback_lock = threading.Lock()
        self._shutting_down = False
        self._latest_image = None
        self._latest_info = None
        self._latest_clock = None
        self._gz_node = GzNode()
        self._clock_topic = str(self.get_parameter('gz_clock_topic').value)
        self._image_topic = str(self.get_parameter('gz_image_topic').value)
        self._camera_info_topic = str(
            self.get_parameter('gz_camera_info_topic').value)
        self._gz_node.subscribe(
            GzImage, self._image_topic, self._on_image)
        self._gz_node.subscribe(
            GzCameraInfo, self._camera_info_topic, self._on_camera_info)
        self._gz_node.subscribe(GzClock, self._clock_topic, self._on_clock)
        self._frames = 0
        self._last_frames = 0
        self.create_timer(0.02, self._publish_pending)
        self.create_timer(2.0, self._report_rate)
        self.get_logger().info(
            f'Gazebo Harmonic camera bridge ready: '
            f'{self._image_topic} + {self._camera_info_topic}')

    def _on_image(self, message: GzImage) -> None:
        with self._callback_lock:
            if not self._shutting_down:
                self._latest_image = message

    def _on_camera_info(self, message: GzCameraInfo) -> None:
        with self._callback_lock:
            if not self._shutting_down:
                self._latest_info = message

    def _on_clock(self, message: GzClock) -> None:
        with self._callback_lock:
            if not self._shutting_down:
                self._latest_clock = message.sim.sec, message.sim.nsec

    def _publish_pending(self) -> None:
        with self._callback_lock:
            clock, self._latest_clock = self._latest_clock, None
            info, self._latest_info = self._latest_info, None
            image, self._latest_image = self._latest_image, None
        try:
            if clock is not None:
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = clock
                self._clock_pub.publish(msg)
            if info is not None:
                self._camera_info_pub.publish(camera_info_to_ros(
                    info, self._fallback_frame_id))
            if image is not None:
                self._image_pub.publish(image_to_ros(
                    image, self._fallback_frame_id))
                self._frames += 1
        except (RuntimeError, ValueError) as error:
            if rclpy.ok():
                self.get_logger().error(
                    f'failed to publish Gazebo camera data: {error}',
                    throttle_duration_sec=5.0)

    def stop(self) -> None:
        """Prevent transport callbacks from using ROS entities on shutdown."""
        with self._callback_lock:
            self._shutting_down = True
        # Unsubscribe after releasing the callback lock: transport may wait for
        # an in-flight callback, and that callback must be allowed to finish.
        if self._gz_node is not None:
            self._gz_node.unsubscribe(self._image_topic)
            self._gz_node.unsubscribe(self._camera_info_topic)
            self._gz_node.unsubscribe(self._clock_topic)
            self._gz_node = None

    def _report_rate(self) -> None:
        frames = self._frames - self._last_frames
        self._last_frames = self._frames
        self.get_logger().info(
            f'Gazebo down camera stream: {frames / 2.0:.1f} FPS')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Detect ArUco markers and publish poses in the camera optical frame."""

from __future__ import annotations

import socket
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_msgs.msg import (
    Float64MultiArray,
    Int32MultiArray,
    MultiArrayDimension,
)

from ..common.image import array_to_image, image_to_bgr
from ..cameras.picamera2_protocol import HEADER, receive_exact, unpack_header
from .geometry import camera_matrix, rotation_matrix_to_quaternion


class ArucoDetectorNode(Node):
    """Estimate marker pose with calibrated camera intrinsics."""

    def __init__(self) -> None:
        super().__init__('aruco_detector_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('input_mode', 'ros_topic')
        self.declare_parameter(
            'camera_socket_path', '/ros2_ws/run/down_camera.sock')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter(
            'camera_frame_id', 'down_camera_optical_frame')
        self.declare_parameter('camera_matrix', [0.0] * 9)
        self.declare_parameter('distortion_coefficients', [0.0] * 5)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.16)
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('publish_debug_topics', False)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('maximum_processing_rate_hz', 15.0)
        self.declare_parameter('ids_topic', '/uav/aruco/ids')
        self.declare_parameter('rvecs_topic', '/uav/aruco/rvecs')
        self.declare_parameter('tvecs_topic', '/uav/aruco/tvecs')
        self.declare_parameter('target_pose_topic', '/uav/aruco/target_pose')
        self.declare_parameter('status_topic', '/uav/aruco/status')
        self.declare_parameter('debug_image_topic', '/uav/aruco/debug_image')

        marker_size = float(self.get_parameter('marker_size_m').value)
        if marker_size <= 0.0:
            raise ValueError('marker_size_m must be positive')
        self._marker_size = marker_size
        self._target_id = int(self.get_parameter('target_marker_id').value)
        self._publish_debug_topics = bool(
            self.get_parameter('publish_debug_topics').value)
        self._publish_debug = bool(
            self.get_parameter('publish_debug_image').value)
        rate = float(
            self.get_parameter('maximum_processing_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('maximum_processing_rate_hz must be positive')
        self._minimum_period = 1.0 / rate
        self._last_processed = float('-inf')
        self._input_mode = str(self.get_parameter('input_mode').value)
        if self._input_mode not in ('ros_topic', 'picamera2_socket'):
            raise ValueError(
                'input_mode must be ros_topic or picamera2_socket')
        self._camera_socket = None
        self._camera_socket_path = str(
            self.get_parameter('camera_socket_path').value)
        self._camera_width = int(self.get_parameter('camera_width').value)
        self._camera_height = int(self.get_parameter('camera_height').value)
        self._camera_frame_id = str(
            self.get_parameter('camera_frame_id').value)

        try:
            import cv2
        except ImportError as error:
            raise RuntimeError(
                'OpenCV with the aruco module is required') from error
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV was built without the aruco module')
        self._cv2 = cv2
        dictionary_name = str(self.get_parameter('dictionary').value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')
        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, 'DetectorParameters')
            else cv2.aruco.DetectorParameters_create())
        self._detector_parameters = parameters
        self._detector = cv2.aruco.ArucoDetector(
            self._dictionary, parameters) if hasattr(
                cv2.aruco, 'ArucoDetector') else None

        self._camera_matrix = None
        self._distortion = None
        if self._input_mode == 'picamera2_socket':
            if self._camera_width <= 0 or self._camera_height <= 0:
                raise ValueError('camera width and height must be positive')
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
        self._ids_topic = str(self.get_parameter('ids_topic').value)
        self._ids = self.create_publisher(
            Int32MultiArray, self._ids_topic, 10) \
            if self._publish_debug_topics else None
        self._rvecs = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('rvecs_topic').value), 10) \
            if self._publish_debug_topics else None
        self._tvecs = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('tvecs_topic').value), 10) \
            if self._publish_debug_topics else None
        self._target = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value), 10)
        self._status = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value), 10)
        self._debug = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            qos_profile_sensor_data) \
            if self._publish_debug else None
        if self._input_mode == 'ros_topic':
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter('camera_info_topic').value),
                self._on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter('image_topic').value),
                self._on_image,
                qos_profile_sensor_data,
            )
        else:
            self.create_timer(
                self._minimum_period, self._capture_socket_and_process)
        self.get_logger().info(
            f'ArUco ready: dictionary={dictionary_name}, '
            f'marker_size={marker_size:.3f}m, target_id={self._target_id}, '
            f'input={self._input_mode}, '
            f'debug_topics={self._publish_debug_topics}, '
            f'debug_image={self._publish_debug}')

    def _on_camera_info(self, message: CameraInfo) -> None:
        try:
            self._camera_matrix = camera_matrix(message.k)
            self._distortion = np.asarray(message.d, dtype=np.float64)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def _detect(self, gray):
        if self._detector is not None:
            return self._detector.detectMarkers(gray)
        return self._cv2.aruco.detectMarkers(
            gray, self._dictionary,
            parameters=self._detector_parameters)

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_processed < self._minimum_period:
            return
        self._last_processed = now
        started = time.perf_counter()
        if self._camera_matrix is None:
            self._publish_status(message.header, DiagnosticStatus.WARN,
                                 'waiting_for_camera_info', 0, False, started)
            return
        try:
            bgr = image_to_bgr(message)
        except (ValueError, RuntimeError) as error:
            self._publish_status(message.header, DiagnosticStatus.ERROR,
                                 str(error), 0, False, started)
            return
        self._process_bgr(bgr, message.header, started)

    def _close_camera_socket(self) -> None:
        if self._camera_socket is not None:
            self._camera_socket.close()
            self._camera_socket = None

    def _receive_socket_frame(self) -> np.ndarray:
        if self._camera_socket is None:
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            connection.settimeout(10.0)
            try:
                connection.connect(self._camera_socket_path)
            except Exception:
                connection.close()
                raise
            self._camera_socket = connection
            self.get_logger().info(
                f'Connected to Picamera2 host stream: '
                f'{self._camera_socket_path}')
        header = receive_exact(self._camera_socket, HEADER.size)
        width, height, payload_size = unpack_header(header)
        if width != self._camera_width or height != self._camera_height:
            raise ValueError(
                f'host frame {width}x{height} does not match configured '
                f'{self._camera_width}x{self._camera_height}')
        payload = receive_exact(self._camera_socket, payload_size)
        self._camera_socket.settimeout(2.0)
        rgb = np.frombuffer(payload, dtype=np.uint8).reshape(
            height, width, 3)
        return self._cv2.cvtColor(rgb, self._cv2.COLOR_RGB2BGR)

    def _capture_socket_and_process(self) -> None:
        started = time.perf_counter()
        try:
            bgr = self._receive_socket_frame()
        except (ConnectionError, OSError, ValueError) as error:
            self._close_camera_socket()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._camera_frame_id
            self._publish_status(
                header, DiagnosticStatus.ERROR,
                f'camera_frame_unavailable: {error}', 0, False, started)
            self.get_logger().warning(
                f'Picamera2 host stream unavailable: {error}',
                throttle_duration_sec=5.0)
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._camera_frame_id
        self._process_bgr(bgr, header, started)

    def _process_bgr(self, bgr, header: Header, started: float) -> None:
        gray = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        flat_ids = [] if ids is None else [int(value) for value in ids.flat]
        rvecs = np.empty((0, 3), dtype=np.float64)
        tvecs = np.empty((0, 3), dtype=np.float64)
        target_visible = False

        if flat_ids:
            rvecs, tvecs, _ = self._cv2.aruco.estimatePoseSingleMarkers(
                corners, self._marker_size, self._camera_matrix,
                self._distortion)
            for marker_id, rvec, tvec in zip(flat_ids, rvecs, tvecs):
                if marker_id == self._target_id:
                    target = PoseStamped()
                    target.header = header
                    target.pose = self._pose(rvec, tvec)
                    self._target.publish(target)
                    target_visible = True
                    break
            if self._debug is not None:
                self._cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                for rvec, tvec in zip(rvecs, tvecs):
                    self._cv2.drawFrameAxes(
                        bgr, self._camera_matrix, self._distortion,
                        rvec, tvec, self._marker_size * 0.5)

        if self._publish_debug_topics:
            id_message = Int32MultiArray()
            id_message.data = flat_ids
            self._ids.publish(id_message)
            self._rvecs.publish(self._vectors_message(rvecs, 'rx,ry,rz'))
            self._tvecs.publish(
                self._vectors_message(tvecs, 'x,y,z;units=m'))
        if self._debug is not None:
            debug = array_to_image(bgr, 'bgr8')
            debug.header = header
            self._debug.publish(debug)
        self._publish_status(header, DiagnosticStatus.OK, 'ok',
                             len(flat_ids), target_visible, started)

    def _vectors_message(
            self, values, component_label: str) -> Float64MultiArray:
        vectors = np.asarray(values, dtype=np.float64).reshape(-1, 3)
        message = Float64MultiArray()
        message.layout.dim = [
            MultiArrayDimension(
                label=f'markers;order_matches={self._ids_topic}',
                size=vectors.shape[0],
                stride=vectors.shape[0] * 3,
            ),
            MultiArrayDimension(
                label=component_label,
                size=3,
                stride=3,
            ),
        ]
        message.data = vectors.reshape(-1).tolist()
        return message

    def _pose(self, rvec, tvec) -> Pose:
        rotation, _ = self._cv2.Rodrigues(np.asarray(rvec).reshape(3))
        quaternion = rotation_matrix_to_quaternion(rotation)
        translation = np.asarray(tvec).reshape(3)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(
            float, translation)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, \
            pose.orientation.w = map(float, quaternion)
        return pose

    def _publish_status(
            self, header, level, message, count, target_visible,
            started) -> None:
        status = DiagnosticStatus()
        status.name = 'aruco_detector'
        status.hardware_id = header.frame_id or 'camera'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='marker_count', value=str(count)),
            KeyValue(key='target_marker_id', value=str(self._target_id)),
            KeyValue(
                key='target_visible', value=str(target_visible).lower()),
            KeyValue(key='latency_ms', value=(
                f'{(time.perf_counter() - started) * 1000.0:.2f}')),
        ]
        array = DiagnosticArray()
        array.header.stamp = header.stamp
        array.status = [status]
        self._status.publish(array)

    def destroy_node(self):
        self._close_camera_socket()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

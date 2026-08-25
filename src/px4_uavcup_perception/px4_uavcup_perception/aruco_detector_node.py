#!/usr/bin/env python3
"""Detect ArUco markers and publish poses in the camera optical frame."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import (
    Float64MultiArray,
    Int32MultiArray,
    MultiArrayDimension,
)

from .aruco_geometry import camera_matrix, rotation_matrix_to_quaternion
from .image_utils import array_to_image, image_to_bgr


class ArucoDetectorNode(Node):
    """Estimate marker pose with calibrated camera intrinsics."""

    def __init__(self) -> None:
        super().__init__('aruco_detector_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.16)
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('maximum_processing_rate_hz', 15.0)
        self.declare_parameter('poses_topic', '/uav/aruco/poses')
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
        self._publish_debug = bool(
            self.get_parameter('publish_debug_image').value)
        rate = float(
            self.get_parameter('maximum_processing_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('maximum_processing_rate_hz must be positive')
        self._minimum_period = 1.0 / rate
        self._last_processed = float('-inf')

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
        self._poses = self.create_publisher(
            PoseArray, str(self.get_parameter('poses_topic').value), 10)
        self._ids = self.create_publisher(
            Int32MultiArray, str(self.get_parameter('ids_topic').value), 10)
        self._rvecs = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('rvecs_topic').value), 10)
        self._tvecs = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('tvecs_topic').value), 10)
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
        self.get_logger().info(
            f'ArUco ready: dictionary={dictionary_name}, '
            f'marker_size={marker_size:.3f}m, target_id={self._target_id}')

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
            self._publish_status(message, DiagnosticStatus.WARN,
                                 'waiting_for_camera_info', 0, started)
            return
        try:
            bgr = image_to_bgr(message)
            gray = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detect(gray)
        except (ValueError, RuntimeError) as error:
            self._publish_status(message, DiagnosticStatus.ERROR,
                                 str(error), 0, started)
            return

        pose_array = PoseArray()
        pose_array.header = message.header
        id_message = Int32MultiArray()
        flat_ids = [] if ids is None else [int(value) for value in ids.flat]
        id_message.data = flat_ids
        rvecs = np.empty((0, 3), dtype=np.float64)
        tvecs = np.empty((0, 3), dtype=np.float64)

        if flat_ids:
            rvecs, tvecs, _ = self._cv2.aruco.estimatePoseSingleMarkers(
                corners, self._marker_size, self._camera_matrix,
                self._distortion)
            for marker_id, rvec, tvec in zip(flat_ids, rvecs, tvecs):
                pose = self._pose(rvec, tvec)
                pose_array.poses.append(pose)
                if marker_id == self._target_id:
                    target = PoseStamped()
                    target.header = message.header
                    target.pose = pose
                    self._target.publish(target)
            if self._debug is not None:
                self._cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                for rvec, tvec in zip(rvecs, tvecs):
                    self._cv2.drawFrameAxes(
                        bgr, self._camera_matrix, self._distortion,
                        rvec, tvec, self._marker_size * 0.5)

        self._poses.publish(pose_array)
        self._ids.publish(id_message)
        self._rvecs.publish(self._vectors_message(rvecs, 'rx,ry,rz'))
        self._tvecs.publish(self._vectors_message(tvecs, 'x,y,z;units=m'))
        if self._debug is not None:
            debug = array_to_image(bgr, 'bgr8')
            debug.header = message.header
            self._debug.publish(debug)
        self._publish_status(message, DiagnosticStatus.OK, 'ok',
                             len(flat_ids), started)

    @staticmethod
    def _vectors_message(values, component_label: str) -> Float64MultiArray:
        vectors = np.asarray(values, dtype=np.float64).reshape(-1, 3)
        message = Float64MultiArray()
        message.layout.dim = [
            MultiArrayDimension(
                label='markers;order_matches=/uav/aruco/ids',
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

    def _publish_status(self, image, level, message, count, started) -> None:
        status = DiagnosticStatus()
        status.name = 'aruco_detector'
        status.hardware_id = image.header.frame_id or 'camera'
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key='marker_count', value=str(count)),
            KeyValue(key='target_marker_id', value=str(self._target_id)),
            KeyValue(key='latency_ms', value=(
                f'{(time.perf_counter() - started) * 1000.0:.2f}')),
        ]
        array = DiagnosticArray()
        array.header.stamp = image.header.stamp
        array.status = [status]
        self._status.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

#!/usr/bin/env python3
"""Opt-in bridge from a selected ArUco pose to PX4 precision landing."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import LandingTargetPose, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .px4_bridge_geometry import OPTICAL_TO_BODY_FRD, target_camera_to_ned


class ArucoToPx4LandingTarget(Node):
    """Publish PX4 LandingTargetPose only after explicit enablement."""

    def __init__(self) -> None:
        super().__init__('aruco_to_px4_landing_target')
        self.declare_parameter('enabled', False)
        self.declare_parameter('target_pose_topic', '/uav/aruco/target_pose')
        self.declare_parameter(
            'vehicle_attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter(
            'px4_output_topic', '/fmu/in/landing_target_pose')
        self.declare_parameter('maximum_attitude_age_s', 0.25)
        self.declare_parameter('maximum_target_distance_m', 12.0)
        self.declare_parameter(
            'camera_to_body_frd_rotation',
            OPTICAL_TO_BODY_FRD.reshape(-1).tolist())
        self.declare_parameter(
            'camera_position_body_frd_m', [0.0, 0.0, 0.0])

        self._enabled = bool(self.get_parameter('enabled').value)
        self._maximum_attitude_age = float(
            self.get_parameter('maximum_attitude_age_s').value)
        self._maximum_distance = float(
            self.get_parameter('maximum_target_distance_m').value)
        self._camera_rotation = np.asarray(
            self.get_parameter('camera_to_body_frd_rotation').value,
            dtype=np.float64).reshape(3, 3)
        self._camera_position = np.asarray(
            self.get_parameter('camera_position_body_frd_m').value,
            dtype=np.float64)
        self._attitude = None
        self._attitude_received = float('-inf')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleAttitude,
            str(self.get_parameter('vehicle_attitude_topic').value),
            self._on_attitude, px4_qos)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            self._on_target, 10)
        self._publisher = self.create_publisher(
            LandingTargetPose,
            str(self.get_parameter('px4_output_topic').value), px4_qos)
        state = 'ENABLED' if self._enabled else 'disabled (monitor only)'
        self.get_logger().warning(f'ArUco PX4 landing-target bridge: {state}')

    def _on_attitude(self, message: VehicleAttitude) -> None:
        self._attitude = message
        self._attitude_received = time.monotonic()

    def _on_target(self, message: PoseStamped) -> None:
        if not self._enabled:
            return
        if self._attitude is None or (
                time.monotonic() - self._attitude_received
                > self._maximum_attitude_age):
            self.get_logger().warning(
                'Not publishing landing target: PX4 attitude is stale',
                throttle_duration_sec=1.0)
            return
        camera_xyz = np.array([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        if not np.all(np.isfinite(camera_xyz)) or (
                np.linalg.norm(camera_xyz) > self._maximum_distance):
            self.get_logger().warning(
                'Rejected invalid/out-of-range ArUco pose',
                throttle_duration_sec=1.0)
            return
        try:
            ned = target_camera_to_ned(
                camera_xyz, self._attitude.q,
                self._camera_rotation, self._camera_position)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=1.0)
            return
        output = LandingTargetPose()
        output.timestamp = self.get_clock().now().nanoseconds // 1000
        output.is_static = True
        output.rel_pos_valid = True
        output.rel_vel_valid = False
        output.x_rel, output.y_rel, output.z_rel = map(float, ned)
        output.vx_rel = float('nan')
        output.vy_rel = float('nan')
        distance = max(float(np.linalg.norm(camera_xyz)), 0.1)
        variance = (0.01 + 0.01 * distance) ** 2
        output.cov_x_rel = variance
        output.cov_y_rel = variance
        output.cov_vx_rel = float('nan')
        output.cov_vy_rel = float('nan')
        output.abs_pos_valid = False
        output.x_abs = float('nan')
        output.y_abs = float('nan')
        output.z_abs = float('nan')
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoToPx4LandingTarget()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

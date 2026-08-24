#!/usr/bin/env python3
"""Opt-in ArUco PID controller that publishes bounded body-FLU cmd_vel."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .landing_pid import (
    DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
    PidAxis,
    camera_target_to_body_flu,
)


class ArucoLandingPidNode(Node):
    """Center over a selected marker and descend only inside an alignment gate."""

    def __init__(self) -> None:
        super().__init__('aruco_landing_pid')
        self.declare_parameter('enabled', False)
        self.declare_parameter('target_pose_topic', '/uav/aruco/target_pose')
        self.declare_parameter('cmd_vel_topic', '/aruco_land/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('marker_timeout_s', 0.35)
        self.declare_parameter('horizontal_kp', 0.55)
        self.declare_parameter('horizontal_ki', 0.02)
        self.declare_parameter('horizontal_kd', 0.08)
        self.declare_parameter('horizontal_integral_limit_m_s', 0.3)
        self.declare_parameter('maximum_horizontal_speed_m_s', 0.35)
        self.declare_parameter('horizontal_deadband_m', 0.025)
        self.declare_parameter('descent_alignment_radius_m', 0.15)
        self.declare_parameter('vertical_kp', 0.35)
        self.declare_parameter('maximum_descent_speed_m_s', 0.20)
        self.declare_parameter('final_marker_distance_m', 0.35)
        self.declare_parameter(
            'camera_to_body_flu_rotation',
            DOWN_CAMERA_OPTICAL_TO_BODY_FLU.reshape(-1).tolist())

        self._enabled = bool(self.get_parameter('enabled').value)
        self._marker_timeout = float(
            self.get_parameter('marker_timeout_s').value)
        self._deadband = float(
            self.get_parameter('horizontal_deadband_m').value)
        self._alignment_radius = float(
            self.get_parameter('descent_alignment_radius_m').value)
        self._vertical_kp = float(self.get_parameter('vertical_kp').value)
        self._maximum_descent = float(
            self.get_parameter('maximum_descent_speed_m_s').value)
        self._final_distance = float(
            self.get_parameter('final_marker_distance_m').value)
        self._rotation = np.asarray(
            self.get_parameter('camera_to_body_flu_rotation').value,
            dtype=np.float64).reshape(3, 3)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._validate_parameters(rate)

        pid_args = dict(
            kp=float(self.get_parameter('horizontal_kp').value),
            ki=float(self.get_parameter('horizontal_ki').value),
            kd=float(self.get_parameter('horizontal_kd').value),
            integral_limit=float(self.get_parameter(
                'horizontal_integral_limit_m_s').value),
            output_limit=float(self.get_parameter(
                'maximum_horizontal_speed_m_s').value),
        )
        self._forward_pid = PidAxis(**pid_args)
        self._left_pid = PidAxis(**pid_args)
        self._command = Twist()
        self._last_pose_time = float('-inf')
        self._last_pid_time = None
        self._horizontal_error = float('nan')
        self._marker_distance = float('nan')
        self._state = 'disabled' if not self._enabled else 'waiting_for_marker'

        self._commands = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._ready = self.create_publisher(
            Bool, '/uav/aruco/landing_ready', 10)
        self._status = self.create_publisher(
            DiagnosticArray, '/uav/aruco/landing_status', 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            self._on_pose, 10)
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().warning(
            f'ArUco landing PID enabled={self._enabled}; it only publishes '
            'bounded cmd_vel and never arms, changes mode, lands, or disarms PX4')

    def _validate_parameters(self, rate: float) -> None:
        positive = {
            'publish_rate_hz': rate,
            'marker_timeout_s': self._marker_timeout,
            'descent_alignment_radius_m': self._alignment_radius,
            'vertical_kp': self._vertical_kp,
            'maximum_descent_speed_m_s': self._maximum_descent,
            'final_marker_distance_m': self._final_distance,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self._deadband < 0.0 or not math.isfinite(self._deadband):
            raise ValueError('horizontal_deadband_m must be finite and non-negative')

    def _on_enable(self, request, response):
        self._enabled = bool(request.data)
        self._reset_controller()
        self._state = 'waiting_for_marker' if self._enabled else 'disabled'
        response.success = True
        response.message = f'ArUco landing PID {self._state}'
        self.get_logger().warning(response.message)
        return response

    def _reset_controller(self) -> None:
        self._forward_pid.reset()
        self._left_pid.reset()
        self._command = Twist()
        self._last_pid_time = None

    def _on_pose(self, message: PoseStamped) -> None:
        if not self._enabled:
            return
        now = time.monotonic()
        camera_xyz = np.array([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        try:
            body = camera_target_to_body_flu(camera_xyz, self._rotation)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=1.0)
            return
        dt = 0.05 if self._last_pid_time is None else now - self._last_pid_time
        if dt <= 0.0 or dt > self._marker_timeout:
            self._forward_pid.reset()
            self._left_pid.reset()
            dt = 0.05
        self._last_pid_time = now
        self._last_pose_time = now

        forward_error = 0.0 if abs(body[0]) < self._deadband else float(body[0])
        left_error = 0.0 if abs(body[1]) < self._deadband else float(body[1])
        self._horizontal_error = math.hypot(body[0], body[1])
        self._marker_distance = max(0.0, -float(body[2]))

        command = Twist()
        command.linear.x = self._forward_pid.update(forward_error, dt)
        command.linear.y = self._left_pid.update(left_error, dt)
        if self._horizontal_error <= self._alignment_radius:
            remaining = self._marker_distance - self._final_distance
            if remaining > 0.0:
                command.linear.z = -min(
                    self._maximum_descent, self._vertical_kp * remaining)
                self._state = 'descending'
            else:
                command.linear.z = 0.0
                self._state = 'ready_for_final_land'
        else:
            command.linear.z = 0.0
            self._state = 'aligning'
        self._command = command

    def _publish(self) -> None:
        if not self._enabled:
            return
        marker_fresh = (
            time.monotonic() - self._last_pose_time <= self._marker_timeout)
        if not marker_fresh:
            if self._state != 'marker_lost':
                self.get_logger().warning(
                    'ArUco marker lost: commanding zero motion/altitude hold')
            self._state = 'marker_lost'
            self._reset_controller()
        self._commands.publish(self._command)
        ready = Bool()
        ready.data = self._state == 'ready_for_final_land'
        self._ready.publish(ready)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = 'aruco_landing_pid'
        status.hardware_id = 'down_camera'
        status.level = (
            DiagnosticStatus.OK if self._enabled and self._state not in (
                'marker_lost', 'waiting_for_marker') else DiagnosticStatus.WARN)
        if not self._enabled:
            status.level = DiagnosticStatus.STALE
        status.message = self._state
        status.values = [
            KeyValue(key='enabled', value=str(self._enabled).lower()),
            KeyValue(key='horizontal_error_m',
                     value=f'{self._horizontal_error:.3f}'),
            KeyValue(key='marker_distance_m',
                     value=f'{self._marker_distance:.3f}'),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoLandingPidNode()
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

#!/usr/bin/env python3
"""Opt-in ArUco landing machine that publishes bounded body-FLU cmd_vel."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .landing_state import (
    LandingState,
    approach_height_m,
    lock_state_from_frames,
    on_marker_stale,
    publishes_command,
    uses_marker,
)
from .pid import (
    DOWN_CAMERA_OPTICAL_TO_BODY_FLU,
    PidAxis,
    camera_target_to_body_flu,
    gripper_clearance_from_marker,
    ready_for_blind_descent,
)


class ArucoLandingPidNode(Node):
    """Search, visually approach, then hand off to a latched blind descent."""

    def __init__(self) -> None:
        super().__init__('aruco_landing_pid')
        self.declare_parameter('enabled', False)
        self.declare_parameter('target_pose_topic', '/uav/aruco/target_pose')
        self.declare_parameter('cmd_vel_topic', '/aruco_land/cmd_vel')
        self.declare_parameter(
            'land_detected_topic', '/fmu/out/vehicle_land_detected')
        self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter(
            'latch_xy_yaw_service', '/cmd_vel_to_px4/latch_xy_yaw')
        self.declare_parameter(
            'release_xy_yaw_service', '/cmd_vel_to_px4/release_xy_yaw')
        self.declare_parameter(
            'disarm_service', '/cmd_vel_to_px4/request_disarm')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('marker_timeout_s', 0.35)
        self.declare_parameter('latch_timeout_s', 1.0)
        self.declare_parameter('lock_confirmation_frames', 5)
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
        self.declare_parameter('camera_to_gripper_vertical_m', 0.0)
        self.declare_parameter('blind_transition_agl_m', 0.30)
        self.declare_parameter('blind_transition_alignment_radius_m', 0.08)
        self.declare_parameter('blind_transition_confirmation_frames', 5)
        self.declare_parameter('blind_descent_speed_m_s', 0.10)
        self.declare_parameter('touchdown_descent_speed_m_s', 0.05)
        self.declare_parameter('maximum_blind_descent_duration_s', 12.0)
        self.declare_parameter(
            'camera_to_body_flu_rotation',
            DOWN_CAMERA_OPTICAL_TO_BODY_FLU.reshape(-1).tolist())
        self.declare_parameter(
            'camera_position_body_flu_m', [0.0, 0.0, 0.0])

        self._marker_timeout = float(
            self.get_parameter('marker_timeout_s').value)
        self._latch_timeout = float(
            self.get_parameter('latch_timeout_s').value)
        self._deadband = float(
            self.get_parameter('horizontal_deadband_m').value)
        self._alignment_radius = float(
            self.get_parameter('descent_alignment_radius_m').value)
        self._vertical_kp = float(self.get_parameter('vertical_kp').value)
        self._maximum_descent = float(
            self.get_parameter('maximum_descent_speed_m_s').value)
        self._final_distance = float(
            self.get_parameter('final_marker_distance_m').value)
        self._lock_confirmation_frames = int(
            self.get_parameter('lock_confirmation_frames').value)
        self._camera_to_gripper_vertical = float(
            self.get_parameter('camera_to_gripper_vertical_m').value)
        self._blind_transition_agl = float(
            self.get_parameter('blind_transition_agl_m').value)
        self._blind_alignment_radius = float(self.get_parameter(
            'blind_transition_alignment_radius_m').value)
        self._blind_confirmation_frames = int(self.get_parameter(
            'blind_transition_confirmation_frames').value)
        self._blind_descent_speed = float(
            self.get_parameter('blind_descent_speed_m_s').value)
        self._touchdown_descent_speed = float(
            self.get_parameter('touchdown_descent_speed_m_s').value)
        self._maximum_blind_duration = float(self.get_parameter(
            'maximum_blind_descent_duration_s').value)
        self._rotation = np.asarray(
            self.get_parameter('camera_to_body_flu_rotation').value,
            dtype=np.float64).reshape(3, 3)
        self._camera_position = np.asarray(
            self.get_parameter('camera_position_body_flu_m').value,
            dtype=np.float64)
        if (self._camera_position.shape != (3,)
                or not np.all(np.isfinite(self._camera_position))):
            raise ValueError(
                'camera_position_body_flu_m must contain 3 finite values')
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
        self._gripper_clearance = float('nan')
        self._approach_height = float('nan')
        self._consecutive_pose_frames = 0
        self._blind_gate_frames = 0
        self._blind_started_at = None
        self._latch_future = None
        self._latch_started_at = None
        self._disarm_requested = False
        self._ground_contact = False
        self._landed = False
        self._dist_bottom_valid = False
        self._dist_bottom = float('nan')
        self._approach_substate = ''
        self._abort_reason = ''
        enabled = bool(self.get_parameter('enabled').value)
        self._state = (
            LandingState.SEARCH if enabled else LandingState.IDLE)

        self._commands = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._state_pub = self.create_publisher(
            String, '/uav/aruco/landing_state', 10)
        self._status = self.create_publisher(
            DiagnosticArray, '/uav/aruco/landing_status', 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('target_pose_topic').value),
            self._on_pose, 10)
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLandDetected,
            str(self.get_parameter('land_detected_topic').value),
            self._on_land_detected,
            px4_qos)
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter('local_position_topic').value),
            self._on_local_position,
            px4_qos)
        self._latch_client = self.create_client(
            Trigger, str(self.get_parameter('latch_xy_yaw_service').value))
        self._release_client = self.create_client(
            Trigger, str(self.get_parameter('release_xy_yaw_service').value))
        self._disarm_client = self.create_client(
            Trigger, str(self.get_parameter('disarm_service').value))
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.create_service(Trigger, '~/start_landing', self._on_start_landing)
        self.create_service(Trigger, '~/abort', self._on_abort)
        self.create_timer(1.0 / rate, self._publish)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().warning(
            f'ArUco landing state={self._state.value}; it only publishes '
            'bounded cmd_vel on /aruco_land/cmd_vel and never creates a '
            'second PX4 command path. start_landing does not request Offboard. '
            f'camera_position_body_flu_m={self._camera_position.tolist()}')

    def _validate_parameters(self, rate: float) -> None:
        positive = {
            'publish_rate_hz': rate,
            'marker_timeout_s': self._marker_timeout,
            'latch_timeout_s': self._latch_timeout,
            'descent_alignment_radius_m': self._alignment_radius,
            'vertical_kp': self._vertical_kp,
            'maximum_descent_speed_m_s': self._maximum_descent,
            'final_marker_distance_m': self._final_distance,
            'blind_transition_agl_m': self._blind_transition_agl,
            'blind_transition_alignment_radius_m':
                self._blind_alignment_radius,
            'blind_descent_speed_m_s': self._blind_descent_speed,
            'touchdown_descent_speed_m_s': self._touchdown_descent_speed,
            'maximum_blind_descent_duration_s': self._maximum_blind_duration,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self._deadband < 0.0 or not math.isfinite(self._deadband):
            raise ValueError(
                'horizontal_deadband_m must be finite and non-negative')
        if (self._camera_to_gripper_vertical < 0.0
                or not math.isfinite(self._camera_to_gripper_vertical)):
            raise ValueError(
                'camera_to_gripper_vertical_m must be finite and non-negative')
        if self._lock_confirmation_frames <= 0:
            raise ValueError('lock_confirmation_frames must be positive')
        if self._blind_confirmation_frames <= 0:
            raise ValueError(
                'blind_transition_confirmation_frames must be positive')
        if self._touchdown_descent_speed > self._blind_descent_speed:
            raise ValueError(
                'touchdown_descent_speed_m_s must not exceed '
                'blind_descent_speed_m_s')

    def _on_enable(self, request, response):
        if request.data:
            self._enter_search('enable')
            response.success = True
            response.message = f'ArUco landing {self._state.value}'
        else:
            self._enter_idle()
            response.success = True
            response.message = 'ArUco landing idle'
        self.get_logger().warning(response.message)
        return response

    def _on_start_landing(self, request, response):
        del request
        self._enter_search('start_landing')
        response.success = True
        response.message = (
            'landing started: SEARCH. Enable cmd_vel_to_px4 and request '
            'Offboard separately before the vehicle will follow cmd_vel.')
        self.get_logger().warning(response.message)
        return response

    def _on_abort(self, request, response):
        del request
        if self._state is LandingState.IDLE:
            response.success = True
            response.message = 'already idle'
            return response
        if self._state is LandingState.COMPLETE:
            response.success = True
            response.message = 'landing already complete'
            return response
        self._abort('operator abort')
        response.success = True
        response.message = 'landing aborted; publishing zero cmd_vel'
        return response

    def _enter_idle(self) -> None:
        self._state = LandingState.IDLE
        self._reset_session()
        self._call_trigger(self._release_client)

    def _enter_search(self, reason: str) -> None:
        self._call_trigger(self._release_client)
        self._reset_session()
        self._state = LandingState.SEARCH
        self.get_logger().warning(f'Landing SEARCH ({reason})')

    def _reset_controller(self) -> None:
        self._forward_pid.reset()
        self._left_pid.reset()
        self._command = Twist()
        self._last_pid_time = None

    def _reset_session(self) -> None:
        self._reset_controller()
        self._last_pose_time = float('-inf')
        self._horizontal_error = float('nan')
        self._marker_distance = float('nan')
        self._gripper_clearance = float('nan')
        self._approach_height = float('nan')
        self._consecutive_pose_frames = 0
        self._blind_gate_frames = 0
        self._blind_started_at = None
        self._latch_future = None
        self._latch_started_at = None
        self._disarm_requested = False
        self._approach_substate = ''
        self._abort_reason = ''

    def _call_trigger(self, client) -> None:
        if client.service_is_ready():
            client.call_async(Trigger.Request())

    def _on_local_position(self, message: VehicleLocalPosition) -> None:
        self._dist_bottom_valid = bool(message.dist_bottom_valid)
        self._dist_bottom = float(message.dist_bottom)

    def _on_land_detected(self, message: VehicleLandDetected) -> None:
        self._ground_contact = bool(
            message.ground_contact or message.maybe_landed)
        self._landed = bool(message.landed)
        if self._state is LandingState.BLIND_FINAL_DESCENT and self._landed:
            self._complete()

    def _on_pose(self, message: PoseStamped) -> None:
        if not uses_marker(self._state):
            return
        now = time.monotonic()
        camera_xyz = np.array([
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ], dtype=np.float64)
        try:
            body = camera_target_to_body_flu(
                camera_xyz, self._rotation, self._camera_position)
            self._marker_distance = max(0.0, float(camera_xyz[2]))
            self._gripper_clearance = gripper_clearance_from_marker(
                self._marker_distance, self._camera_to_gripper_vertical)
            self._approach_height = approach_height_m(
                self._gripper_clearance,
                self._dist_bottom,
                self._dist_bottom_valid)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=1.0)
            return

        if now - self._last_pose_time > self._marker_timeout:
            self._consecutive_pose_frames = 1
        else:
            self._consecutive_pose_frames += 1
        self._last_pose_time = now
        self._horizontal_error = math.hypot(body[0], body[1])

        if self._state in (LandingState.SEARCH, LandingState.LOCK):
            self._state = lock_state_from_frames(
                self._consecutive_pose_frames,
                self._lock_confirmation_frames)
            if self._state is not LandingState.VISUAL_APPROACH:
                self._command = Twist()
                self._approach_substate = ''
                return

        dt = 0.05 if self._last_pid_time is None else now - self._last_pid_time
        if dt <= 0.0 or dt > self._marker_timeout:
            self._forward_pid.reset()
            self._left_pid.reset()
            dt = 0.05
        self._last_pid_time = now

        forward_error = (
            0.0 if abs(body[0]) < self._deadband else float(body[0]))
        left_error = 0.0 if abs(body[1]) < self._deadband else float(body[1])
        command = Twist()
        command.linear.x = self._forward_pid.update(forward_error, dt)
        command.linear.y = self._left_pid.update(left_error, dt)
        if self._horizontal_error <= self._alignment_radius:
            remaining = self._approach_height - self._blind_transition_agl
            if remaining > 0.0:
                command.linear.z = -min(
                    self._maximum_descent, self._vertical_kp * remaining)
            self._approach_substate = 'descending'
        else:
            self._approach_substate = 'aligning'
        self._command = command

        try:
            ready = ready_for_blind_descent(
                self._approach_height,
                self._horizontal_error,
                self._blind_transition_agl,
                self._blind_alignment_radius)
        except ValueError:
            ready = False
        if ready:
            self._blind_gate_frames += 1
        else:
            self._blind_gate_frames = 0

    def _request_latch(self) -> None:
        if self._latch_future is not None:
            return
        if not self._latch_client.service_is_ready():
            self._abort('latch service unavailable')
            return
        self._latch_future = self._latch_client.call_async(Trigger.Request())
        self._latch_started_at = time.monotonic()
        self.get_logger().warning(
            'Requesting XY/yaw latch for blind final descent')

    def _poll_latch(self, now: float) -> None:
        if self._state is not LandingState.VISUAL_APPROACH:
            return
        if (self._latch_future is None
                and self._blind_gate_frames >= self._blind_confirmation_frames):
            self._request_latch()
            return
        if self._latch_future is None:
            return
        if not self._latch_future.done():
            started = self._latch_started_at or now
            if now - started > self._latch_timeout:
                self._latch_future = None
                self._abort('latch service timeout')
            return
        try:
            result = self._latch_future.result()
        except Exception as error:  # noqa: BLE001 - service future
            self._latch_future = None
            self._abort(f'latch failed: {error}')
            return
        self._latch_future = None
        if result is None or not result.success:
            reason = 'latch rejected'
            if result is not None and result.message:
                reason = result.message
            self._abort(reason)
            return
        self._state = LandingState.BLIND_FINAL_DESCENT
        self._blind_started_at = now
        self._reset_controller()
        self._approach_substate = ''
        self.get_logger().warning(
            'Blind final descent: XY/yaw latched, ArUco no longer required')

    def _abort(self, reason: str) -> None:
        if self._state in (LandingState.IDLE, LandingState.COMPLETE):
            return
        self._state = LandingState.ABORT
        self._abort_reason = reason
        self._latch_future = None
        self._reset_controller()
        self.get_logger().warning(f'Landing ABORT: {reason}')

    def _complete(self) -> None:
        if self._state is LandingState.COMPLETE:
            return
        self._state = LandingState.COMPLETE
        self._command = Twist()
        self._reset_controller()
        self.get_logger().warning(
            'Landing COMPLETE: PX4 reported landed')
        if not self._disarm_requested:
            self._disarm_requested = True
            if self._disarm_client.service_is_ready():
                self._call_trigger(self._disarm_client)
            else:
                self.get_logger().error(
                    'PX4 landed but request_disarm is unavailable')

    def _publish(self) -> None:
        if not publishes_command(self._state):
            return
        now = time.monotonic()
        self._poll_latch(now)

        if uses_marker(self._state):
            fresh = now - self._last_pose_time <= self._marker_timeout
            if not fresh:
                next_state = on_marker_stale(self._state)
                if next_state is LandingState.ABORT:
                    self._abort('marker lost before blind transition')
                elif next_state is LandingState.SEARCH:
                    self._state = LandingState.SEARCH
                    self._consecutive_pose_frames = 0
                    self._blind_gate_frames = 0
                    self._reset_controller()
                else:
                    self._command = Twist()

        if self._state is LandingState.BLIND_FINAL_DESCENT:
            command = Twist()
            speed = (
                self._touchdown_descent_speed
                if self._ground_contact else self._blind_descent_speed)
            command.linear.z = -speed
            self._command = command
            started = self._blind_started_at or now
            if now - started > self._maximum_blind_duration:
                self._abort(
                    'blind descent timeout; holding latched XY/yaw')
            elif self._landed:
                self._complete()

        if self._state in (LandingState.ABORT, LandingState.COMPLETE):
            self._command = Twist()

        self._commands.publish(self._command)
        state = String()
        state.data = self._state.value
        self._state_pub.publish(state)

    def _publish_status(self) -> None:
        status = DiagnosticStatus()
        status.name = 'aruco_landing_pid'
        status.hardware_id = 'down_camera'
        if self._state is LandingState.IDLE:
            status.level = DiagnosticStatus.STALE
        elif self._state in (LandingState.ABORT,):
            status.level = DiagnosticStatus.ERROR
        elif self._state in (
                LandingState.SEARCH, LandingState.LOCK,
                LandingState.COMPLETE):
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self._state.value
        status.values = [
            KeyValue(key='state', value=self._state.value),
            KeyValue(key='approach_substate', value=self._approach_substate),
            KeyValue(key='abort_reason', value=self._abort_reason),
            KeyValue(key='horizontal_error_m',
                     value=f'{self._horizontal_error:.3f}'),
            KeyValue(key='marker_distance_m',
                     value=f'{self._marker_distance:.3f}'),
            KeyValue(key='gripper_clearance_m',
                     value=f'{self._gripper_clearance:.3f}'),
            KeyValue(key='approach_height_m',
                     value=f'{self._approach_height:.3f}'),
            KeyValue(key='dist_bottom_valid',
                     value=str(self._dist_bottom_valid).lower()),
            KeyValue(key='dist_bottom_m',
                     value=f'{self._dist_bottom:.3f}'),
            KeyValue(key='lock_frames',
                     value=str(self._consecutive_pose_frames)),
            KeyValue(key='blind_gate_frames',
                     value=str(self._blind_gate_frames)),
            KeyValue(key='landed', value=str(self._landed).lower()),
            KeyValue(key='ground_contact',
                     value=str(self._ground_contact).lower()),
            KeyValue(
                key='camera_position_body_flu_m',
                value=','.join(
                    f'{value:.3f}' for value in self._camera_position)),
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

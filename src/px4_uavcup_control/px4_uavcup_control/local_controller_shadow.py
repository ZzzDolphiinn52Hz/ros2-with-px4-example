#!/usr/bin/env python3

"""ROS adapter for the three-sector controller; never publishes to PX4."""

import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .shadow_controller import (
    AvoidanceState,
    ControllerConfig,
    ControllerDecision,
    ShadowController,
)


class LocalControllerShadowNode(Node):
    """Publish advisory velocity and diagnostics without touching PX4."""

    def __init__(self) -> None:
        super().__init__('local_controller_shadow')

        self.declare_parameter('free_space_topic', '/uav/depth/free_space')
        self.declare_parameter(
            'advisory_velocity_topic',
            '/uav/local_controller/advisory_velocity')
        self.declare_parameter(
            'state_topic', '/uav/local_controller/state')
        self.declare_parameter(
            'status_topic', '/uav/local_controller/status')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('input_units', 'metres')
        self.declare_parameter('watchdog_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.3)

        self.declare_parameter('emergency_distance_m', 0.35)
        self.declare_parameter('avoid_enter_distance_m', 0.45)
        self.declare_parameter('clear_exit_distance_m', 0.50)
        self.declare_parameter('minimum_valid_fraction', 0.25)
        self.declare_parameter('forward_speed_mps', 0.4)
        self.declare_parameter('avoidance_forward_speed_mps', 0.1)
        self.declare_parameter('lateral_speed_mps', 0.3)
        self.declare_parameter('median_window', 5)
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('recovery_frames', 3)
        self.declare_parameter('minimum_direction_hold_sec', 0.5)
        self.declare_parameter('side_switch_margin_m', 0.25)

        config = ControllerConfig(
            emergency_distance_m=float(
                self.get_parameter('emergency_distance_m').value),
            avoid_enter_distance_m=float(
                self.get_parameter('avoid_enter_distance_m').value),
            clear_exit_distance_m=float(
                self.get_parameter('clear_exit_distance_m').value),
            minimum_valid_fraction=float(
                self.get_parameter('minimum_valid_fraction').value),
            forward_speed_mps=float(
                self.get_parameter('forward_speed_mps').value),
            avoidance_forward_speed_mps=float(
                self.get_parameter('avoidance_forward_speed_mps').value),
            lateral_speed_mps=float(
                self.get_parameter('lateral_speed_mps').value),
            median_window=int(self.get_parameter('median_window').value),
            ema_alpha=float(self.get_parameter('ema_alpha').value),
            recovery_frames=int(self.get_parameter('recovery_frames').value),
            minimum_direction_hold_sec=float(
                self.get_parameter('minimum_direction_hold_sec').value),
            side_switch_margin_m=float(
                self.get_parameter('side_switch_margin_m').value),
        )
        self._controller = ShadowController(config)
        self._input_timeout = float(
            self.get_parameter('input_timeout_sec').value)
        watchdog_rate = float(
            self.get_parameter('watchdog_rate_hz').value)
        if self._input_timeout <= 0.0 or watchdog_rate <= 0.0:
            raise ValueError('Watchdog rate and timeout must be positive')

        self._frame_id = str(self.get_parameter('body_frame_id').value)
        self._input_units = str(
            self.get_parameter('input_units').value)
        self._last_input_monotonic: Optional[float] = None
        self._last_decision: Optional[ControllerDecision] = None
        self._last_logged_state: Optional[AvoidanceState] = None
        self._last_watchdog_failsafe = float('-inf')

        self._advisory_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('advisory_velocity_topic').value),
            1,
        )
        self._state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('state_topic').value),
            1,
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._subscription = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('free_space_topic').value),
            self._on_free_space,
            1,
        )
        self._watchdog = self.create_timer(
            1.0 / watchdog_rate, self._on_watchdog)

        self.get_logger().warning(
            'SHADOW MODE active: advisory velocity only; no PX4 command '
            'topics are created')

    def _on_free_space(self, message: Float32MultiArray) -> None:
        now = time.monotonic()
        self._last_input_monotonic = now
        if len(message.data) < 5:
            decision = self._controller.invalidate(
                'free-space message has fewer than five values', now)
            self._publish(decision, input_age_sec=0.0)
            return

        left, center, right, _, valid_fraction = (
            float(value) for value in message.data[:5])
        decision = self._controller.update(
            left,
            center,
            right,
            valid_fraction,
            now,
        )
        self._publish(decision, input_age_sec=0.0)

    def _on_watchdog(self) -> None:
        now = time.monotonic()
        age = (
            math.inf if self._last_input_monotonic is None
            else now - self._last_input_monotonic)
        if age <= self._input_timeout:
            return
        if now - self._last_watchdog_failsafe < 0.5:
            return
        self._last_watchdog_failsafe = now
        decision = self._controller.invalidate(
            'free-space input stale or unavailable', now)
        self._publish(decision, input_age_sec=age)

    def _publish(
            self,
            decision: ControllerDecision,
            input_age_sec: float) -> None:
        stamp = self.get_clock().now().to_msg()
        advisory = TwistStamped()
        advisory.header.stamp = stamp
        advisory.header.frame_id = self._frame_id
        advisory.twist.linear.x = decision.forward_mps
        advisory.twist.linear.y = decision.left_mps
        self._advisory_publisher.publish(advisory)

        state = String()
        state.data = decision.state.value
        self._state_publisher.publish(state)

        status_array = DiagnosticArray()
        status_array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'px4_uavcup/local_controller_shadow'
        status.hardware_id = 'shadow_only'
        status.level = (
            DiagnosticStatus.ERROR
            if decision.state == AvoidanceState.FAILSAFE
            else DiagnosticStatus.WARN
            if decision.state == AvoidanceState.BRAKE
            else DiagnosticStatus.OK)
        status.message = decision.reason
        value_suffix = '_m' if self._input_units == 'metres' else ''
        status.values = [
            KeyValue(key='shadow_mode', value='true'),
            KeyValue(key='input_units', value=self._input_units),
            KeyValue(key='state', value=decision.state.value),
            KeyValue(
                key='forward_advisory_mps',
                value=f'{decision.forward_mps:.3f}'),
            KeyValue(
                key='left_advisory_mps',
                value=f'{decision.left_mps:.3f}'),
            KeyValue(
                key='input_age_sec', value=f'{input_age_sec:.3f}'),
            KeyValue(
                key=f'filtered_left{value_suffix}',
                value=f'{decision.filtered_left_m:.3f}'),
            KeyValue(
                key=f'filtered_center{value_suffix}',
                value=f'{decision.filtered_center_m:.3f}'),
            KeyValue(
                key=f'filtered_right{value_suffix}',
                value=f'{decision.filtered_right_m:.3f}'),
        ]
        status_array.status = [status]
        self._status_publisher.publish(status_array)
        self._last_decision = decision

        if decision.state != self._last_logged_state:
            self.get_logger().info(
                f'State={decision.state.value} reason="{decision.reason}" '
                f'advisory=({decision.forward_mps:.2f}, '
                f'{decision.left_mps:.2f}) m/s '
                f'L/C/R={decision.filtered_left_m:.2f}/'
                f'{decision.filtered_center_m:.2f}/'
                f'{decision.filtered_right_m:.2f}')
            self._last_logged_state = decision.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalControllerShadowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ROS launch may forward a second SIGINT during teardown.
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()

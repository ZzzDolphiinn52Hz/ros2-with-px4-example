#!/usr/bin/env python3
"""ROS shadow adapter for 2D relative-depth corridor guidance."""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .corridor_controller import (
    CorridorController,
    CorridorControllerConfig,
    CorridorDecision,
    CorridorState,
)


class CorridorControllerShadowNode(Node):
    """Publish 3D advisory velocity without creating PX4 command topics."""

    def __init__(self) -> None:
        super().__init__('corridor_controller_shadow')
        self.declare_parameter(
            'corridor_topic', '/uav/depth/relative_corridor')
        self.declare_parameter(
            'advisory_velocity_topic',
            '/uav/local_controller/advisory_velocity')
        self.declare_parameter(
            'state_topic', '/uav/local_controller/state')
        self.declare_parameter(
            'status_topic', '/uav/local_controller/status')
        self.declare_parameter('body_frame_id', 'base_link')
        self.declare_parameter('watchdog_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.4)
        self.declare_parameter('minimum_clearance', 0.10)
        self.declare_parameter('minimum_valid_fraction', 0.90)
        self.declare_parameter('target_deadband', 0.08)
        self.declare_parameter('forward_speed_mps', 0.20)
        self.declare_parameter('minimum_forward_scale', 0.25)
        self.declare_parameter('maximum_lateral_speed_mps', 0.20)
        self.declare_parameter('maximum_vertical_speed_mps', 0.12)
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('maximum_target_step', 0.20)
        self.declare_parameter('recovery_frames', 3)

        config = CorridorControllerConfig(
            minimum_clearance=float(
                self.get_parameter('minimum_clearance').value),
            minimum_valid_fraction=float(
                self.get_parameter('minimum_valid_fraction').value),
            target_deadband=float(
                self.get_parameter('target_deadband').value),
            forward_speed_mps=float(
                self.get_parameter('forward_speed_mps').value),
            minimum_forward_scale=float(
                self.get_parameter('minimum_forward_scale').value),
            maximum_lateral_speed_mps=float(self.get_parameter(
                'maximum_lateral_speed_mps').value),
            maximum_vertical_speed_mps=float(self.get_parameter(
                'maximum_vertical_speed_mps').value),
            ema_alpha=float(self.get_parameter('ema_alpha').value),
            maximum_target_step=float(
                self.get_parameter('maximum_target_step').value),
            recovery_frames=int(
                self.get_parameter('recovery_frames').value),
        )
        self._controller = CorridorController(config)
        self._input_timeout = float(
            self.get_parameter('input_timeout_sec').value)
        watchdog_rate = float(
            self.get_parameter('watchdog_rate_hz').value)
        if self._input_timeout <= 0.0 or watchdog_rate <= 0.0:
            raise ValueError('watchdog rate and timeout must be positive')
        self._frame_id = str(self.get_parameter('body_frame_id').value)
        self._last_input_monotonic: Optional[float] = None
        self._last_logged_state: Optional[CorridorState] = None
        self._last_watchdog_failsafe = float('-inf')

        self._advisory_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter('advisory_velocity_topic').value), 1)
        self._state_publisher = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), 1)
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter('corridor_topic').value),
            self._on_corridor,
            1,
        )
        self.create_timer(1.0 / watchdog_rate, self._on_watchdog)
        self.get_logger().warning(
            '2D CORRIDOR SHADOW MODE active: advisory velocity only; '
            'no PX4 command topics are created')

    def _on_corridor(self, message: Float32MultiArray) -> None:
        now = time.monotonic()
        self._last_input_monotonic = now
        if len(message.data) < 7:
            decision = self._controller.invalidate(
                'corridor message has fewer than seven values')
        else:
            x, y, width, height, clearance, _, valid_fraction = (
                float(value) for value in message.data[:7])
            decision = self._controller.update(
                x, y, width, height, clearance, valid_fraction)
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
            'corridor input stale or unavailable')
        self._publish(decision, input_age_sec=age)

    def _publish(
            self,
            decision: CorridorDecision,
            input_age_sec: float) -> None:
        stamp = self.get_clock().now().to_msg()
        advisory = TwistStamped()
        advisory.header.stamp = stamp
        advisory.header.frame_id = self._frame_id
        advisory.twist.linear.x = decision.forward_mps
        advisory.twist.linear.y = decision.left_mps
        advisory.twist.linear.z = decision.up_mps
        self._advisory_publisher.publish(advisory)

        state = String()
        state.data = decision.state.value
        self._state_publisher.publish(state)

        status_array = DiagnosticArray()
        status_array.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = 'px4_uavcup/corridor_controller_shadow'
        status.hardware_id = 'shadow_only'
        status.level = (
            DiagnosticStatus.ERROR
            if decision.state == CorridorState.FAILSAFE
            else DiagnosticStatus.WARN
            if decision.state == CorridorState.BRAKE
            else DiagnosticStatus.OK)
        status.message = decision.reason
        status.values = [
            KeyValue(key='shadow_mode', value='true'),
            KeyValue(key='input_units', value='relative_image_corridor'),
            KeyValue(key='state', value=decision.state.value),
            KeyValue(
                key='forward_advisory_mps',
                value=f'{decision.forward_mps:.3f}'),
            KeyValue(
                key='left_advisory_mps',
                value=f'{decision.left_mps:.3f}'),
            KeyValue(
                key='up_advisory_mps',
                value=f'{decision.up_mps:.3f}'),
            KeyValue(
                key='target_x_normalized',
                value=f'{decision.target_x_normalized:.3f}'),
            KeyValue(
                key='target_y_normalized',
                value=f'{decision.target_y_normalized:.3f}'),
            KeyValue(
                key='corridor_clearance',
                value=f'{decision.clearance:.3f}'),
            KeyValue(
                key='input_age_sec', value=f'{input_age_sec:.3f}'),
        ]
        status_array.status = [status]
        self._status_publisher.publish(status_array)

        if decision.state != self._last_logged_state:
            self.get_logger().info(
                f'State={decision.state.value} reason="{decision.reason}" '
                f'target=({decision.target_x_normalized:.2f}, '
                f'{decision.target_y_normalized:.2f}) '
                f'advisory=({decision.forward_mps:.2f}, '
                f'{decision.left_mps:.2f}, {decision.up_mps:.2f}) m/s')
            self._last_logged_state = decision.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorControllerShadowNode()
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

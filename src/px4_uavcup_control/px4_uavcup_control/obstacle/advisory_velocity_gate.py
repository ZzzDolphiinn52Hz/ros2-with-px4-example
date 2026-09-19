#!/usr/bin/env python3
"""Explicit safety gate from shadow advisory velocity to ``cmd_vel``.

The gate starts disabled, forgets every command when its state changes, and
requires a new, finite body-FLU advisory after it is enabled.  It never arms
the vehicle or requests an PX4 flight mode.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_srvs.srv import SetBool


def bounded_velocity(
    forward: float,
    left: float,
    up: float,
    maximum_horizontal: float,
    maximum_vertical: float,
) -> Tuple[float, float, float]:
    """Validate and bound a body-FLU velocity without changing XY direction."""
    values = (forward, left, up, maximum_horizontal, maximum_vertical)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('velocity limits and components must be finite')
    if maximum_horizontal <= 0.0 or maximum_vertical <= 0.0:
        raise ValueError('velocity limits must be positive')
    magnitude = math.hypot(forward, left)
    if magnitude > maximum_horizontal:
        scale = maximum_horizontal / magnitude
        forward *= scale
        left *= scale
    up = max(-maximum_vertical, min(maximum_vertical, up))
    return forward, left, up


class AdvisoryVelocityGate(Node):
    """Manually enabled watchdog gate; disabled means no output publication."""

    def __init__(self) -> None:
        super().__init__('corridor_velocity_gate')
        self.declare_parameter(
            'advisory_topic', '/uav/local_controller/advisory_velocity')
        self.declare_parameter(
            'cmd_vel_topic', '/uav/local_controller/cmd_vel')
        self.declare_parameter('required_frame_id', 'base_link')
        self.declare_parameter('input_timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('maximum_horizontal_speed_mps', 0.20)
        self.declare_parameter('maximum_vertical_speed_mps', 0.12)

        self._required_frame = str(
            self.get_parameter('required_frame_id').value)
        self._timeout = float(self.get_parameter('input_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._maximum_horizontal = float(
            self.get_parameter('maximum_horizontal_speed_mps').value)
        self._maximum_vertical = float(
            self.get_parameter('maximum_vertical_speed_mps').value)
        if self._timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')
        bounded_velocity(
            0.0, 0.0, 0.0,
            self._maximum_horizontal, self._maximum_vertical)

        self._enabled = False
        self._latest: Optional[Tuple[float, float, float]] = None
        self._latest_time: Optional[float] = None
        self._stale_logged = False
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.create_subscription(
            TwistStamped,
            str(self.get_parameter('advisory_topic').value),
            self._on_advisory,
            10,
        )
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warning(
            'Corridor velocity gate is DISABLED. It does not arm or request '
            'Offboard; enable explicitly through /corridor_velocity_gate/enable.')

    def _on_advisory(self, message: TwistStamped) -> None:
        if message.header.frame_id != self._required_frame:
            self.get_logger().error(
                f'Rejected advisory frame "{message.header.frame_id}"; '
                f'expected "{self._required_frame}"',
                throttle_duration_sec=2.0,
            )
            return
        try:
            self._latest = bounded_velocity(
                float(message.twist.linear.x),
                float(message.twist.linear.y),
                float(message.twist.linear.z),
                self._maximum_horizontal,
                self._maximum_vertical,
            )
        except ValueError as error:
            self.get_logger().error(f'Rejected advisory: {error}')
            self._latest = None
            self._latest_time = None
            return
        self._latest_time = time.monotonic()
        self._stale_logged = False

    def _on_enable(self, request, response):
        self._enabled = bool(request.data)
        # Never reuse a command sampled before an enable/disable transition.
        self._latest = None
        self._latest_time = None
        self._stale_logged = False
        response.success = True
        response.message = (
            'enabled; waiting for a new body-FLU advisory'
            if self._enabled else 'disabled; cmd_vel publication stopped')
        self.get_logger().warning(response.message)
        return response

    def _publish(self) -> None:
        if not self._enabled:
            return
        now = time.monotonic()
        if (
            self._latest is None
            or self._latest_time is None
            or now - self._latest_time > self._timeout
        ):
            # Publish zero while enabled so a live adapter brakes immediately;
            # the adapter's own timeout remains an independent second guard.
            velocity = (0.0, 0.0, 0.0)
            if not self._stale_logged:
                self.get_logger().warning(
                    'Advisory unavailable/stale: publishing zero cmd_vel')
                self._stale_logged = True
        else:
            velocity = self._latest

        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = velocity
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdvisoryVelocityGate()
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

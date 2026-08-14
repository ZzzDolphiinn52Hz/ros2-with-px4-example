#!/usr/bin/env python3
"""Adapt Nav2 ``/cmd_vel`` commands to PX4 Offboard trajectory setpoints.

Nav2 publishes ``geometry_msgs/Twist`` in the ROS base FLU frame. PX4 expects
trajectory velocities in the earth-fixed NED frame. This node rotates and
converts the horizontal velocity, keeps a fixed NED altitude setpoint, and
limits acceleration so a 2D lidar-equipped multicopter does not tilt sharply.

Safety policy:

* disabled by default;
* never arms the vehicle or requests Offboard mode;
* stops horizontal/yaw motion when ``/cmd_vel`` times out;
* continues holding altitude after a command timeout while enabled.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_srvs.srv import SetBool, Trigger


def clamp_xy(vx: float, vy: float, maximum: float) -> Tuple[float, float]:
    """Limit a 2D vector by magnitude without changing its direction."""
    if maximum < 0.0:
        raise ValueError('maximum must be non-negative')
    magnitude = math.hypot(vx, vy)
    if magnitude <= maximum or magnitude == 0.0:
        return vx, vy
    scale = maximum / magnitude
    return vx * scale, vy * scale


def slew_xy(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
    max_delta: float,
) -> Tuple[float, float]:
    """Move a 2D command toward its target by at most ``max_delta``."""
    if max_delta < 0.0:
        raise ValueError('max_delta must be non-negative')
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    delta_norm = math.hypot(delta_x, delta_y)
    if delta_norm <= max_delta or delta_norm == 0.0:
        return target_x, target_y
    scale = max_delta / delta_norm
    return current_x + delta_x * scale, current_y + delta_y * scale


def body_flu_to_ned_velocity(
    forward: float,
    left: float,
    heading_ned: float,
) -> Tuple[float, float]:
    """Rotate ROS body FLU horizontal velocity into PX4 earth NED."""
    cos_heading = math.cos(heading_ned)
    sin_heading = math.sin(heading_ned)
    velocity_north = cos_heading * forward + sin_heading * left
    velocity_east = sin_heading * forward - cos_heading * left
    return velocity_north, velocity_east


def ros_yaw_rate_to_ned(yaw_rate_enu: float) -> float:
    """ROS positive CCW/up becomes PX4 NED positive clockwise/down."""
    return -yaw_rate_enu


def px4_quaternion_to_heading_ned(q_wxyz) -> float:
    """Extract PX4 NED heading from a FRD-body-to-NED quaternion."""
    w, x, y, z = (float(value) for value in q_wxyz)
    norm_sq = w * w + x * x + y * y + z * z
    if not math.isfinite(norm_sq) or norm_sq == 0.0:
        raise ValueError('PX4 attitude quaternion is invalid')
    return math.atan2(
        2.0 * (w * z + x * y) / norm_sq,
        1.0 - 2.0 * (y * y + z * z) / norm_sq,
    )


def vehicle_command_result_text(result: int) -> str:
    """Return a readable MAVLink/PX4 vehicle-command result."""
    names = {
        VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
        VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            'TEMPORARILY_REJECTED',
        VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
        VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
        VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
        VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'IN_PROGRESS',
        VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: 'CANCELLED',
    }
    return names.get(int(result), f'UNKNOWN({int(result)})')


class CmdVelToPx4(Node):
    """Safe, explicitly-enabled velocity adapter for PX4 Offboard mode."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_to_px4')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter(
            'attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter(
            'vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        self.declare_parameter('target_altitude_m', 0.7)
        self.declare_parameter('max_xy_speed_m_s', 0.4)
        self.declare_parameter('max_yaw_rate_rad_s', 0.3)
        self.declare_parameter('max_xy_accel_m_s2', 0.3)
        self.declare_parameter('max_yaw_accel_rad_s2', 0.5)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._target_altitude_m = float(
            self.get_parameter('target_altitude_m').value)
        self._max_xy_speed = float(
            self.get_parameter('max_xy_speed_m_s').value)
        self._max_yaw_rate = float(
            self.get_parameter('max_yaw_rate_rad_s').value)
        self._max_xy_accel = float(
            self.get_parameter('max_xy_accel_m_s2').value)
        self._max_yaw_accel = float(
            self.get_parameter('max_yaw_accel_rad_s2').value)
        self._cmd_timeout_s = float(
            self.get_parameter('cmd_timeout_s').value)
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)
        self._validate_parameters()

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._local_position: Optional[VehicleLocalPosition] = None
        self._attitude: Optional[VehicleAttitude] = None
        self._vehicle_status: Optional[VehicleStatus] = None
        self._last_cmd_time_ns: Optional[int] = None
        self._z_reset_counter: Optional[int] = None
        self._target_z_ned = -self._target_altitude_m
        self._enabled = False
        self._target_forward = 0.0
        self._target_left = 0.0
        self._target_yaw_rate = 0.0
        self._active_forward = 0.0
        self._active_left = 0.0
        self._active_yaw_rate = 0.0
        self._timeout_logged = False

        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._on_cmd_vel,
            10,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter('local_position_topic').value,
            self._on_local_position,
            px4_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            self.get_parameter('attitude_topic').value,
            self._on_attitude,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            self.get_parameter('vehicle_status_topic').value,
            self._on_vehicle_status,
            px4_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._on_vehicle_command_ack,
            px4_qos,
        )

        self._offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self._trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self._vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )
        self.create_service(SetBool, '~/enable', self._on_enable)
        self.create_service(
            Trigger, '~/request_offboard', self._on_request_offboard)

        period = 1.0 / self._publish_rate_hz
        self.create_timer(period, self._publish)
        self.create_timer(2.0, self._status)

        self.get_logger().warning(
            'Adapter is DISABLED. It never arms or changes PX4 mode '
            'automatically. Enable it first, then request Offboard explicitly '
            'through /cmd_vel_to_px4/request_offboard.')

    def _validate_parameters(self) -> None:
        positive_values = {
            'target_altitude_m': self._target_altitude_m,
            'max_xy_speed_m_s': self._max_xy_speed,
            'max_yaw_rate_rad_s': self._max_yaw_rate,
            'max_xy_accel_m_s2': self._max_xy_accel,
            'max_yaw_accel_rad_s2': self._max_yaw_accel,
            'cmd_timeout_s': self._cmd_timeout_s,
            'publish_rate_hz': self._publish_rate_hz,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_cmd_vel(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejected non-finite /cmd_vel command')
            return

        forward, left = clamp_xy(
            float(msg.linear.x),
            float(msg.linear.y),
            self._max_xy_speed,
        )
        self._target_forward = forward
        self._target_left = left
        self._target_yaw_rate = max(
            -self._max_yaw_rate,
            min(self._max_yaw_rate, float(msg.angular.z)),
        )
        self._last_cmd_time_ns = self.get_clock().now().nanoseconds
        self._timeout_logged = False

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        if self._z_reset_counter is None:
            self._z_reset_counter = int(msg.z_reset_counter)
        elif int(msg.z_reset_counter) != self._z_reset_counter:
            # PX4 delta_z is new estimate - old estimate. Move the target by
            # the same delta to preserve the same physical altitude.
            self._target_z_ned += float(msg.delta_z)
            self.get_logger().warning(
                'PX4 EKF Z reset detected: '
                f'{self._z_reset_counter}->{msg.z_reset_counter}; '
                f'target_z += {msg.delta_z:.3f} m')
            self._z_reset_counter = int(msg.z_reset_counter)
        self._local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def _on_vehicle_command_ack(self, msg: VehicleCommandAck) -> None:
        if int(msg.command) != VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            return
        result = vehicle_command_result_text(msg.result)
        log = self.get_logger().info
        if msg.result not in (
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        ):
            log = self.get_logger().error
        log(
            'PX4 Offboard mode request ACK: '
            f'{result} param1={msg.result_param1} param2={msg.result_param2}')

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        self._attitude = msg

    def _heading_ned(self) -> Optional[float]:
        if self._attitude is None:
            return None
        try:
            return px4_quaternion_to_heading_ned(self._attitude.q)
        except ValueError:
            return None

    def _position_is_valid(self) -> bool:
        position = self._local_position
        return bool(
            position is not None
            and position.xy_valid
            and position.z_valid
            and all(math.isfinite(value) for value in (
                position.x,
                position.y,
                position.z,
            ))
        )

    def _on_enable(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if request.data:
            if not self._position_is_valid() or self._heading_ned() is None:
                response.success = False
                response.message = (
                    'PX4 local position/attitude is not valid; adapter remains disabled')
                return response

            self._target_z_ned = -self._target_altitude_m
            self._enabled = True
            self._zero_motion()
            response.success = True
            response.message = (
                f'enabled; target altitude={self._target_altitude_m:.2f} m, '
                'now wait for heartbeat before selecting Offboard')
            self.get_logger().warning(response.message)
            return response

        self._enabled = False
        self._zero_motion()
        response.success = True
        response.message = 'disabled; Offboard heartbeat/setpoint publishing stopped'
        self.get_logger().warning(response.message)
        return response

    def _on_request_offboard(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        if not self._enabled:
            response.success = False
            response.message = 'adapter is disabled; enable it first'
            return response

        status = self._vehicle_status
        if status is None:
            response.success = False
            response.message = 'PX4 vehicle status has not been received'
            return response
        if status.failsafe:
            response.success = False
            response.message = 'PX4 is in failsafe; Offboard request not sent'
            return response
        if not status.pre_flight_checks_pass:
            response.success = False
            response.message = 'PX4 preflight checks have not passed'
            return response

        command = VehicleCommand()
        command.timestamp = self._now_us()
        command.param1 = 1.0  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        command.param2 = 6.0  # PX4 custom main mode: OFFBOARD
        command.param3 = 0.0
        command.param4 = 0.0
        command.param5 = 0.0
        command.param6 = 0.0
        command.param7 = 0.0
        command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        command.target_system = 1
        command.target_component = 1
        command.source_system = 1
        command.source_component = 1
        command.confirmation = 0
        command.from_external = True
        self._vehicle_command_pub.publish(command)

        response.success = True
        response.message = (
            'Offboard mode request published; verify PX4 ACK and nav_state=14')
        self.get_logger().warning(response.message)
        return response

    def _zero_motion(self) -> None:
        self._target_forward = 0.0
        self._target_left = 0.0
        self._target_yaw_rate = 0.0
        self._active_forward = 0.0
        self._active_left = 0.0
        self._active_yaw_rate = 0.0
        self._last_cmd_time_ns = None
        self._timeout_logged = False

    def _command_timed_out(self) -> bool:
        if self._last_cmd_time_ns is None:
            return True
        age_s = (
            self.get_clock().now().nanoseconds - self._last_cmd_time_ns
        ) / 1e9
        return age_s > self._cmd_timeout_s

    def _apply_timeout(self) -> None:
        if not self._command_timed_out():
            return
        self._target_forward = 0.0
        self._target_left = 0.0
        self._target_yaw_rate = 0.0
        if self._last_cmd_time_ns is not None and not self._timeout_logged:
            self.get_logger().warning(
                '/cmd_vel timeout: braking XY/yaw command to zero; holding altitude')
            self._timeout_logged = True

    def _publish(self) -> None:
        if not self._enabled:
            return

        position = self._local_position
        heading_ned = self._heading_ned()
        if not self._position_is_valid() or heading_ned is None:
            self.get_logger().error(
                'Lost valid PX4 position/attitude; stopping Offboard setpoints',
                throttle_duration_sec=2.0,
            )
            return

        self._apply_timeout()
        dt = 1.0 / self._publish_rate_hz
        self._active_forward, self._active_left = slew_xy(
            self._active_forward,
            self._active_left,
            self._target_forward,
            self._target_left,
            self._max_xy_accel * dt,
        )
        yaw_delta = self._max_yaw_accel * dt
        yaw_error = self._target_yaw_rate - self._active_yaw_rate
        self._active_yaw_rate += max(-yaw_delta, min(yaw_delta, yaw_error))

        velocity_north, velocity_east = body_flu_to_ned_velocity(
            self._active_forward,
            self._active_left,
            heading_ned,
        )

        mode = OffboardControlMode()
        mode.timestamp = self._now_us()
        # Position is the highest-level active controller because Z is held by
        # a position setpoint. NaN XY position allows velocity control there.
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        self._offboard_mode_pub.publish(mode)

        nan = float('nan')
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = mode.timestamp
        setpoint.position = [nan, nan, float(self._target_z_ned)]
        setpoint.velocity = [velocity_north, velocity_east, nan]
        setpoint.acceleration = [nan, nan, nan]
        setpoint.jerk = [nan, nan, nan]
        setpoint.yaw = nan
        setpoint.yawspeed = ros_yaw_rate_to_ned(self._active_yaw_rate)
        self._trajectory_pub.publish(setpoint)

    def _status(self) -> None:
        position = self._local_position
        altitude = (
            'unknown' if position is None or not position.z_valid
            else f'{max(0.0, -float(position.z)):.2f}m'
        )
        nav_state = (
            'unknown' if self._vehicle_status is None
            else str(int(self._vehicle_status.nav_state))
        )
        self.get_logger().info(
            f'enabled={self._enabled} altitude={altitude} '
            f'target={self._target_altitude_m:.2f}m nav_state={nav_state} '
            f'active_cmd=[{self._active_forward:.2f} forward, '
            f'{self._active_left:.2f} left, '
            f'{self._active_yaw_rate:.2f} yaw]')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToPx4()
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

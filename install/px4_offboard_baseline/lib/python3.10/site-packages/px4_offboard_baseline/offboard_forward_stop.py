#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class OffboardForwardStopNode(Node):

    TAKEOFF_HEIGHT_M = 2.0
    FORWARD_DISTANCE_M = 2.0

    # Node chạy 10 Hz.
    STABLE_CYCLES = 20          # 2 giây ổn định.
    HOVER_WAIT_CYCLES = 30      # Hover 3 giây trước khi tiến.

    PHASE_WAIT = 'WAIT'
    PHASE_TAKEOFF = 'TAKEOFF'
    PHASE_PRE_FORWARD = 'PRE_FORWARD_HOLD'
    PHASE_FORWARD = 'FORWARD'
    PHASE_FINAL_HOLD = 'FINAL_HOLD'

    def __init__(self) -> None:
        super().__init__('offboard_forward_stop')

        self.px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # (x, y, z, yaw) tại vị trí mặt đất ban đầu.
        self.origin: Optional[
            Tuple[float, float, float, float]
        ] = None

        # Setpoint đang gửi cho PX4.
        self.active_target: Optional[
            Tuple[float, float, float, float]
        ] = None

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_z: Optional[float] = None

        self.current_vx: Optional[float] = None
        self.current_vy: Optional[float] = None
        self.current_vz: Optional[float] = None

        self.arming_state: Optional[int] = None
        self.nav_state: Optional[int] = None

        self.preflight_ok = False
        self.failsafe = False

        self.heartbeat_counter = 0
        self.phase_counter = 0
        self.stable_counter = 0

        self.offboard_command_sent = False
        self.arm_command_sent = False

        self.phase = self.PHASE_WAIT

        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            self.px4_qos,
        )

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            self.px4_qos,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self.px4_qos,
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.px4_qos,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.px4_qos,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.get_logger().warning(
            'Mission SITL: takeoff 2 m → tiến 2 m → dừng.'
        )

    def now_us(self) -> int:
        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    def vehicle_status_callback(
        self,
        msg: VehicleStatus,
    ) -> None:
        self.arming_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)

        self.preflight_ok = bool(
            msg.pre_flight_checks_pass
        )

        self.failsafe = bool(msg.failsafe)

    def local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ) -> None:
        if msg.xy_valid and msg.z_valid:
            values = (
                msg.x,
                msg.y,
                msg.z,
                msg.vx,
                msg.vy,
                msg.vz,
            )

            if all(
                math.isfinite(value)
                for value in values
            ):
                self.current_x = float(msg.x)
                self.current_y = float(msg.y)
                self.current_z = float(msg.z)

                self.current_vx = float(msg.vx)
                self.current_vy = float(msg.vy)
                self.current_vz = float(msg.vz)

        # Chụp origin đúng một lần.
        if self.origin is not None:
            return

        initial_values = (
            msg.x,
            msg.y,
            msg.z,
            msg.heading,
        )

        if not (
            msg.xy_valid
            and msg.z_valid
            and all(
                math.isfinite(value)
                for value in initial_values
            )
        ):
            return

        self.origin = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.heading),
        )

        self.active_target = self.origin

        x, y, z, yaw = self.origin

        self.get_logger().info(
            'Đã chụp origin NED: '
            f'x={x:.3f}, y={y:.3f}, '
            f'z={z:.3f}, yaw={yaw:.3f}'
        )

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()

        msg.timestamp = self.now_us()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        if self.active_target is None:
            return

        x, y, z, yaw = self.active_target
        nan = float('nan')

        msg = TrajectorySetpoint()

        msg.timestamp = self.now_us()

        msg.position = [x, y, z]
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]

        msg.yaw = yaw
        msg.yawspeed = nan

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ) -> None:
        msg = VehicleCommand()

        msg.timestamp = self.now_us()

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0

        msg.command = int(command)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.confirmation = 0
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    def request_offboard(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

        self.offboard_command_sent = True

        self.get_logger().info(
            'Đã yêu cầu OFFBOARD.'
        )

    def request_arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand
            .VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

        self.arm_command_sent = True

        self.get_logger().info(
            'Đã yêu cầu ARM.'
        )

    def start_takeoff(self) -> None:
        if self.origin is None:
            return

        x, y, ground_z, yaw = self.origin

        target_z = (
            ground_z - self.TAKEOFF_HEIGHT_M
        )

        self.active_target = (
            x,
            y,
            target_z,
            yaw,
        )

        self.phase = self.PHASE_TAKEOFF
        self.stable_counter = 0

        self.get_logger().info(
            'TAKEOFF: '
            f'z {ground_z:.3f} → {target_z:.3f}'
        )

    def start_forward(self) -> None:
        if self.origin is None:
            return

        x0, y0, ground_z, yaw = self.origin

        target_z = (
            ground_z - self.TAKEOFF_HEIGHT_M
        )

        # Heading NED:
        # yaw=0      → North (+X)
        # yaw=pi/2   → East (+Y)
        delta_x = (
            self.FORWARD_DISTANCE_M
            * math.cos(yaw)
        )

        delta_y = (
            self.FORWARD_DISTANCE_M
            * math.sin(yaw)
        )

        target_x = x0 + delta_x
        target_y = y0 + delta_y

        self.active_target = (
            target_x,
            target_y,
            target_z,
            yaw,
        )

        self.phase = self.PHASE_FORWARD
        self.stable_counter = 0

        self.get_logger().info(
            'BAY THẲNG THEO HƯỚNG MŨI: '
            f'target_x={target_x:.3f}, '
            f'target_y={target_y:.3f}, '
            f'target_z={target_z:.3f}'
        )

    def update_takeoff_phase(self) -> None:
        if (
            self.active_target is None
            or self.current_z is None
            or self.current_vz is None
        ):
            return

        target_z = self.active_target[2]

        z_error = abs(
            self.current_z - target_z
        )

        if (
            z_error < 0.20
            and abs(self.current_vz) < 0.25
        ):
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        if self.stable_counter >= self.STABLE_CYCLES:
            self.phase = self.PHASE_PRE_FORWARD
            self.phase_counter = 0
            self.stable_counter = 0

            self.get_logger().info(
                'HOVER 2 M ĐÃ ỔN ĐỊNH. '
                'Giữ 3 giây trước khi bay tiến.'
            )

    def update_pre_forward_phase(self) -> None:
        self.phase_counter += 1

        if (
            self.phase_counter
            >= self.HOVER_WAIT_CYCLES
        ):
            self.start_forward()

    def update_forward_phase(self) -> None:
        if (
            self.active_target is None
            or self.current_x is None
            or self.current_y is None
            or self.current_z is None
            or self.current_vx is None
            or self.current_vy is None
            or self.current_vz is None
        ):
            return

        target_x, target_y, target_z, _ = (
            self.active_target
        )

        horizontal_error = math.hypot(
            self.current_x - target_x,
            self.current_y - target_y,
        )

        horizontal_speed = math.hypot(
            self.current_vx,
            self.current_vy,
        )

        z_error = abs(
            self.current_z - target_z
        )

        if (
            horizontal_error < 0.20
            and horizontal_speed < 0.25
            and z_error < 0.25
            and abs(self.current_vz) < 0.25
        ):
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        if self.stable_counter >= self.STABLE_CYCLES:
            self.phase = self.PHASE_FINAL_HOLD
            self.stable_counter = 0

            self.get_logger().info(
                'BAY THẲNG + DỪNG THÀNH CÔNG: '
                f'x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, '
                f'z={self.current_z:.3f}, '
                f'vxy={horizontal_speed:.3f} m/s'
            )

    def update_mission(self) -> None:
        if self.phase == self.PHASE_TAKEOFF:
            self.update_takeoff_phase()

        elif self.phase == self.PHASE_PRE_FORWARD:
            self.update_pre_forward_phase()

        elif self.phase == self.PHASE_FORWARD:
            self.update_forward_phase()

    def timer_callback(self) -> None:
        if (
            self.origin is None
            or self.active_target is None
        ):
            return

        # Heartbeat và setpoint luôn duy trì 10 Hz.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        self.heartbeat_counter += 1

        if self.failsafe:
            if self.heartbeat_counter % 10 == 0:
                self.get_logger().error(
                    'PX4 đang FAILSAFE.'
                )
            return

        # Chờ heartbeat 2 giây.
        if (
            self.heartbeat_counter >= 20
            and not self.offboard_command_sent
        ):
            if not self.preflight_ok:
                if self.heartbeat_counter % 10 == 0:
                    self.get_logger().error(
                        'Preflight chưa pass.'
                    )
                return

            self.request_offboard()
            return

        if (
            self.offboard_command_sent
            and not self.arm_command_sent
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.request_arm()
            return

        if (
            self.arm_command_sent
            and self.phase == self.PHASE_WAIT
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            self.start_takeoff()
            return

        self.update_mission()

        if self.heartbeat_counter % 10 == 0:
            target = self.active_target

            self.get_logger().info(
                f'phase={self.phase} | '
                f'arming={self.arming_state} | '
                f'nav={self.nav_state} | '
                f'position='
                f'({self.current_x}, '
                f'{self.current_y}, '
                f'{self.current_z}) | '
                f'target='
                f'({target[0]:.3f}, '
                f'{target[1]:.3f}, '
                f'{target[2]:.3f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OffboardForwardStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Đang dừng node...'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

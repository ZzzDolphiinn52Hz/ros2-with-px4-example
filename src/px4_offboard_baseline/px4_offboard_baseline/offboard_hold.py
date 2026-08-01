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


class OffboardMissionNode(Node):
    """
    Mission:
      1. Chờ heartbeat.
      2. Chuyển OFFBOARD.
      3. ARM.
      4. Takeoff 2 m.
      5. Bay tiến 2 m theo hướng mũi.
      6. Bay ngang sang phải 2 m.
      7. Gửi lệnh NAV_LAND.
      8. Chờ PX4 tự disarm.

    Chỉ sử dụng trong SITL/Gazebo.
    """

    TAKEOFF_HEIGHT_M = 2.0
    FORWARD_DISTANCE_M = 2.0
    RIGHT_DISTANCE_M = 2.0

    TIMER_PERIOD_S = 0.1
    HEARTBEAT_WARMUP_CYCLES = 20

    # Phải ổn định liên tục 2 giây.
    STABLE_CYCLES = 20

    # Giữ tại mỗi waypoint 2 giây.
    HOLD_CYCLES = 20

    POSITION_TOLERANCE_M = 0.20
    ALTITUDE_TOLERANCE_M = 0.20
    HORIZONTAL_SPEED_TOLERANCE_M_S = 0.25
    VERTICAL_SPEED_TOLERANCE_M_S = 0.25

    PHASE_WAIT = 'WAIT'
    PHASE_TAKEOFF = 'TAKEOFF'
    PHASE_HOLD_AFTER_TAKEOFF = 'HOLD_AFTER_TAKEOFF'
    PHASE_FORWARD = 'FORWARD'
    PHASE_HOLD_AFTER_FORWARD = 'HOLD_AFTER_FORWARD'
    PHASE_RIGHT = 'RIGHT'
    PHASE_HOLD_AFTER_RIGHT = 'HOLD_AFTER_RIGHT'
    PHASE_LANDING = 'LANDING'
    PHASE_COMPLETE = 'COMPLETE'

    def __init__(self) -> None:
        super().__init__('offboard_mission')

        self.px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # (x, y, z, yaw) tại vị trí bắt đầu.
        self.origin: Optional[
            Tuple[float, float, float, float]
        ] = None

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
        self.stable_counter = 0
        self.hold_counter = 0

        self.offboard_command_sent = False
        self.arm_command_sent = False
        self.land_command_sent = False

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
            self.TIMER_PERIOD_S,
            self.timer_callback,
        )

        self.get_logger().warning(
            'Mission SITL: takeoff → tiến → phải → land.'
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
            state_values = (
                msg.x,
                msg.y,
                msg.z,
                msg.vx,
                msg.vy,
                msg.vz,
            )

            if all(
                math.isfinite(value)
                for value in state_values
            ):
                self.current_x = float(msg.x)
                self.current_y = float(msg.y)
                self.current_z = float(msg.z)

                self.current_vx = float(msg.vx)
                self.current_vy = float(msg.vy)
                self.current_vz = float(msg.vz)

        # Chỉ chụp origin đúng một lần.
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
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}, '
            f'yaw={yaw:.3f}'
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

    def request_land(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        self.land_command_sent = True
        self.phase = self.PHASE_LANDING

        self.get_logger().info(
            'Đã gửi VEHICLE_CMD_NAV_LAND. '
            'PX4 đang hạ tại vị trí hiện tại.'
        )

    def set_phase(
        self,
        phase: str,
    ) -> None:
        self.phase = phase
        self.stable_counter = 0
        self.hold_counter = 0

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

        self.set_phase(self.PHASE_TAKEOFF)

        self.get_logger().info(
            'TAKEOFF: '
            f'target=({x:.3f}, {y:.3f}, {target_z:.3f})'
        )

    def start_forward(self) -> None:
        if self.origin is None:
            return

        x0, y0, ground_z, yaw = self.origin

        target_z = (
            ground_z - self.TAKEOFF_HEIGHT_M
        )

        # Forward vector theo heading trong mặt phẳng NED.
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

        self.set_phase(self.PHASE_FORWARD)

        self.get_logger().info(
            'BAY TIẾN: '
            f'target=({target_x:.3f}, '
            f'{target_y:.3f}, '
            f'{target_z:.3f})'
        )

    def start_right(self) -> None:
        if (
            self.origin is None
            or self.active_target is None
        ):
            return

        _, _, _, yaw = self.origin

        forward_x, forward_y, target_z, _ = (
            self.active_target
        )

        # Vector bên phải của drone:
        # right_x = -sin(yaw)
        # right_y =  cos(yaw)
        delta_x = (
            -self.RIGHT_DISTANCE_M
            * math.sin(yaw)
        )

        delta_y = (
            self.RIGHT_DISTANCE_M
            * math.cos(yaw)
        )

        target_x = forward_x + delta_x
        target_y = forward_y + delta_y

        self.active_target = (
            target_x,
            target_y,
            target_z,
            yaw,
        )

        self.set_phase(self.PHASE_RIGHT)

        self.get_logger().info(
            'BAY NGANG SANG PHẢI: '
            f'target=({target_x:.3f}, '
            f'{target_y:.3f}, '
            f'{target_z:.3f})'
        )

    def target_is_stable(self) -> bool:
        if (
            self.active_target is None
            or self.current_x is None
            or self.current_y is None
            or self.current_z is None
            or self.current_vx is None
            or self.current_vy is None
            or self.current_vz is None
        ):
            return False

        target_x, target_y, target_z, _ = (
            self.active_target
        )

        horizontal_error = math.hypot(
            self.current_x - target_x,
            self.current_y - target_y,
        )

        altitude_error = abs(
            self.current_z - target_z
        )

        horizontal_speed = math.hypot(
            self.current_vx,
            self.current_vy,
        )

        vertical_speed = abs(self.current_vz)

        return (
            horizontal_error
            < self.POSITION_TOLERANCE_M
            and altitude_error
            < self.ALTITUDE_TOLERANCE_M
            and horizontal_speed
            < self.HORIZONTAL_SPEED_TOLERANCE_M_S
            and vertical_speed
            < self.VERTICAL_SPEED_TOLERANCE_M_S
        )

    def update_movement_phase(
        self,
        next_hold_phase: str,
        success_text: str,
    ) -> None:
        if self.target_is_stable():
            self.stable_counter += 1
        else:
            self.stable_counter = 0

        if self.stable_counter >= self.STABLE_CYCLES:
            self.set_phase(next_hold_phase)

            self.get_logger().info(
                success_text
            )

    def update_hold_phase(self) -> None:
        self.hold_counter += 1

        if self.hold_counter < self.HOLD_CYCLES:
            return

        if self.phase == self.PHASE_HOLD_AFTER_TAKEOFF:
            self.start_forward()

        elif self.phase == self.PHASE_HOLD_AFTER_FORWARD:
            self.start_right()

        elif self.phase == self.PHASE_HOLD_AFTER_RIGHT:
            self.request_land()

    def update_mission(self) -> None:
        if self.phase == self.PHASE_TAKEOFF:
            self.update_movement_phase(
                self.PHASE_HOLD_AFTER_TAKEOFF,
                'TAKEOFF THÀNH CÔNG. Hover 2 giây.',
            )

        elif self.phase == self.PHASE_FORWARD:
            self.update_movement_phase(
                self.PHASE_HOLD_AFTER_FORWARD,
                'BAY TIẾN + DỪNG THÀNH CÔNG. '
                'Giữ 2 giây.',
            )

        elif self.phase == self.PHASE_RIGHT:
            self.update_movement_phase(
                self.PHASE_HOLD_AFTER_RIGHT,
                'BAY PHẢI + DỪNG THÀNH CÔNG. '
                'Giữ 2 giây trước khi land.',
            )

        elif self.phase in (
            self.PHASE_HOLD_AFTER_TAKEOFF,
            self.PHASE_HOLD_AFTER_FORWARD,
            self.PHASE_HOLD_AFTER_RIGHT,
        ):
            self.update_hold_phase()

        elif self.phase == self.PHASE_LANDING:
            self.update_landing_phase()

    def update_landing_phase(self) -> None:
        # PX4 thường tự disarm sau khi land.
        if (
            self.land_command_sent
            and self.arming_state
            == VehicleStatus.ARMING_STATE_DISARMED
        ):
            self.phase = self.PHASE_COMPLETE

            self.get_logger().info(
                'MISSION HOÀN TẤT: '
                'PX4 đã land và DISARM.'
            )

    def publish_offboard_stream(self) -> None:
        """
        Khi vừa gửi NAV_LAND, tiếp tục stream cho đến khi PX4
        thực sự rời Offboard. Sau đó không gửi setpoint nữa.
        """
        if (
            self.phase == self.PHASE_LANDING
            and self.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            return

        if self.phase == self.PHASE_COMPLETE:
            return

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

    def timer_callback(self) -> None:
        if (
            self.origin is None
            or self.active_target is None
        ):
            return

        self.publish_offboard_stream()

        self.heartbeat_counter += 1

        if self.failsafe:
            if self.heartbeat_counter % 10 == 0:
                self.get_logger().error(
                    'PX4 đang FAILSAFE.'
                )
            return

        if (
            self.heartbeat_counter
            >= self.HEARTBEAT_WARMUP_CYCLES
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
            self.get_logger().info(
                f'phase={self.phase} | '
                f'arming={self.arming_state} | '
                f'nav={self.nav_state} | '
                f'position=('
                f'{self.current_x}, '
                f'{self.current_y}, '
                f'{self.current_z})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OffboardMissionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warning(
            'Node bị dừng thủ công. '
            'Kiểm tra và land drone trong QGroundControl.'
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

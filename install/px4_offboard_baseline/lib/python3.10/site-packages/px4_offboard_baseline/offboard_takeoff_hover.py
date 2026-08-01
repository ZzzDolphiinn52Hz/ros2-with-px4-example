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


class OffboardTakeoffNode(Node):
    """
    Trình tự:
      1. Chụp vị trí mặt đất hiện tại.
      2. Publish heartbeat + hold setpoint.
      3. Chuyển sang Offboard.
      4. Arm.
      5. Takeoff thẳng đứng 2 m.
      6. Hover tại vị trí takeoff.

    Chưa bay tiến, chưa bay ngang và chưa tự land.
    """

    TAKEOFF_HEIGHT_M = 2.0

    def __init__(self) -> None:
        super().__init__('offboard_takeoff')

        self.px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # (x, y, z, yaw) tại mặt đất.
        self.ground_target: Optional[
            Tuple[float, float, float, float]
        ] = None

        # Setpoint hiện đang gửi sang PX4.
        self.active_target: Optional[
            Tuple[float, float, float, float]
        ] = None

        self.current_x: Optional[float] = None
        self.current_y: Optional[float] = None
        self.current_z: Optional[float] = None
        self.current_vz: Optional[float] = None

        self.arming_state: Optional[int] = None
        self.nav_state: Optional[int] = None
        self.preflight_ok = False
        self.failsafe = False

        self.heartbeat_counter = 0
        self.offboard_command_sent = False
        self.arm_command_sent = False
        self.takeoff_started = False
        self.hover_confirm_counter = 0
        self.hover_success_logged = False

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

        # Heartbeat và setpoint ở 10 Hz.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Đang chờ local position và vehicle status...'
        )

        self.get_logger().warning(
            'Node này sẽ arm và takeoff 2 m. '
            'Chỉ chạy trong SITL/Gazebo.'
        )

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def vehicle_status_callback(
        self,
        msg: VehicleStatus,
    ) -> None:
        self.arming_state = int(msg.arming_state)
        self.nav_state = int(msg.nav_state)
        self.preflight_ok = bool(msg.pre_flight_checks_pass)
        self.failsafe = bool(msg.failsafe)

    def local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ) -> None:
        if msg.xy_valid and msg.z_valid:
            if all(
                math.isfinite(value)
                for value in (msg.x, msg.y, msg.z, msg.vz)
            ):
                self.current_x = float(msg.x)
                self.current_y = float(msg.y)
                self.current_z = float(msg.z)
                self.current_vz = float(msg.vz)

        # Chỉ chụp ground target một lần.
        if self.ground_target is not None:
            return

        if not (msg.xy_valid and msg.z_valid):
            return

        values = (
            msg.x,
            msg.y,
            msg.z,
            msg.heading,
        )

        if not all(math.isfinite(value) for value in values):
            return

        self.ground_target = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.heading),
        )

        self.active_target = self.ground_target

        x, y, z, yaw = self.ground_target

        self.get_logger().info(
            'Đã chụp ground target NED: '
            f'x={x:.3f} m, '
            f'y={y:.3f} m, '
            f'z={z:.3f} m, '
            f'yaw={yaw:.3f} rad'
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

    def request_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

        self.offboard_command_sent = True

        self.get_logger().info(
            'Đã gửi yêu cầu chuyển sang OFFBOARD.'
        )

    def request_arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

        self.arm_command_sent = True

        self.get_logger().info(
            'PX4 đã vào OFFBOARD; đã gửi yêu cầu ARM.'
        )

    def start_takeoff(self) -> None:
        if self.ground_target is None:
            return

        x, y, ground_z, yaw = self.ground_target

        takeoff_z = ground_z - self.TAKEOFF_HEIGHT_M

        self.active_target = (
            x,
            y,
            takeoff_z,
            yaw,
        )

        self.takeoff_started = True

        self.get_logger().info(
            'BẮT ĐẦU TAKEOFF: '
            f'ground_z={ground_z:.3f} m → '
            f'target_z={takeoff_z:.3f} m'
        )

    def check_hover_reached(self) -> None:
        if (
            not self.takeoff_started
            or self.hover_success_logged
            or self.active_target is None
            or self.current_z is None
            or self.current_vz is None
        ):
            return

        target_z = self.active_target[2]

        z_error = abs(self.current_z - target_z)
        vertical_speed = abs(self.current_vz)

        # Yêu cầu ổn định liên tục 2 giây:
        # 20 chu kỳ × 0,1 giây.
        if z_error < 0.20 and vertical_speed < 0.25:
            self.hover_confirm_counter += 1
        else:
            self.hover_confirm_counter = 0

        if self.hover_confirm_counter >= 20:
            self.hover_success_logged = True

            self.get_logger().info(
                'TAKEOFF THÀNH CÔNG: '
                f'z={self.current_z:.3f} m, '
                f'target_z={target_z:.3f} m, '
                f'vz={self.current_vz:.3f} m/s. '
                'Drone đang hover.'
            )

    def timer_callback(self) -> None:
        if (
            self.ground_target is None
            or self.active_target is None
        ):
            return

        # Duy trì heartbeat và setpoint liên tục.
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        self.heartbeat_counter += 1

        if self.failsafe:
            if self.heartbeat_counter % 10 == 0:
                self.get_logger().error(
                    'PX4 đang FAILSAFE.'
                )
            return

        # Chờ 2 giây heartbeat trước khi yêu cầu Offboard.
        if (
            self.heartbeat_counter >= 20
            and not self.offboard_command_sent
        ):
            if not self.preflight_ok:
                if self.heartbeat_counter % 10 == 0:
                    self.get_logger().error(
                        'Preflight chưa pass; chưa chuyển Offboard.'
                    )
                return

            self.request_offboard_mode()
            return

        # Chỉ arm sau khi xác nhận Offboard.
        if (
            self.offboard_command_sent
            and not self.arm_command_sent
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.request_arm()
            return

        # Ngay khi xác nhận Armed + Offboard, đổi target lên 2 m.
        if (
            self.arm_command_sent
            and not self.takeoff_started
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            self.start_takeoff()
            return

        self.check_hover_reached()

        # In trạng thái mỗi giây.
        if self.heartbeat_counter % 10 == 0:
            target_z = self.active_target[2]

            current_z_text = (
                f'{self.current_z:.3f}'
                if self.current_z is not None
                else 'None'
            )

            current_vz_text = (
                f'{self.current_vz:.3f}'
                if self.current_vz is not None
                else 'None'
            )

            self.get_logger().info(
                'Heartbeat 10 Hz | '
                f'arming_state={self.arming_state} | '
                f'nav_state={self.nav_state} | '
                f'z={current_z_text} m | '
                f'target_z={target_z:.3f} m | '
                f'vz={current_vz_text} m/s'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OffboardTakeoffNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Đang dừng offboard_takeoff...'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

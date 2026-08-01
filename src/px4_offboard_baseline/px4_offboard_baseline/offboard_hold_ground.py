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


class OffboardHoldNode(Node):
    """
    Chuyển PX4 sang Offboard, arm và giữ nguyên vị trí hiện tại.

    Chưa thực hiện:
    - takeoff;
    - bay tiến;
    - bay ngang;
    - land.
    """

    def __init__(self) -> None:
        super().__init__('offboard_hold')

        self.px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.hold_target: Optional[
            Tuple[float, float, float, float]
        ] = None

        self.arming_state: Optional[int] = None
        self.nav_state: Optional[int] = None
        self.preflight_ok = False
        self.failsafe = False

        self.heartbeat_counter = 0

        self.offboard_command_sent = False
        self.arm_command_sent = False
        self.success_logged = False

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

        # 10 Hz.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Đang chờ local position và vehicle status hợp lệ...'
        )

        self.get_logger().warning(
            'Chỉ chạy node này trong PX4 SITL/Gazebo.'
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
        # Chỉ chụp target một lần.
        if self.hold_target is not None:
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

        self.hold_target = (
            float(msg.x),
            float(msg.y),
            float(msg.z),
            float(msg.heading),
        )

        x, y, z, yaw = self.hold_target

        self.get_logger().info(
            'Đã chụp hold target NED: '
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

    def publish_hold_setpoint(self) -> None:
        if self.hold_target is None:
            return

        x, y, z, yaw = self.hold_target
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

    def timer_callback(self) -> None:
        if self.hold_target is None:
            return

        # Heartbeat và hold setpoint luôn tiếp tục ở 10 Hz.
        self.publish_offboard_control_mode()
        self.publish_hold_setpoint()

        self.heartbeat_counter += 1

        # Chờ 2 giây heartbeat trước khi yêu cầu Offboard.
        if (
            self.heartbeat_counter >= 20
            and not self.offboard_command_sent
        ):
            if not self.preflight_ok:
                self.get_logger().error(
                    'Preflight check chưa pass; '
                    'không gửi lệnh Offboard.'
                )
                return

            if self.failsafe:
                self.get_logger().error(
                    'PX4 đang failsafe; '
                    'không gửi lệnh Offboard.'
                )
                return

            self.request_offboard_mode()
            return

        # Chỉ arm sau khi PX4 xác nhận đã ở Offboard.
        if (
            self.offboard_command_sent
            and not self.arm_command_sent
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.request_arm()
            return

        # Xác nhận thành công một lần.
        if (
            not self.success_logged
            and self.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            self.success_logged = True

            x, y, z, yaw = self.hold_target

            self.get_logger().info(
                'THÀNH CÔNG: PX4 đang ARMED + OFFBOARD | '
                f'hold=({x:.3f}, {y:.3f}, {z:.3f}) m | '
                f'yaw={yaw:.3f} rad'
            )

        # In trạng thái mỗi giây.
        if self.heartbeat_counter % 10 == 0:
            self.get_logger().info(
                'Heartbeat 10 Hz | '
                f'arming_state={self.arming_state} | '
                f'nav_state={self.nav_state} | '
                f'preflight_ok={self.preflight_ok} | '
                f'failsafe={self.failsafe}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OffboardHoldNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Đang dừng offboard_hold...'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

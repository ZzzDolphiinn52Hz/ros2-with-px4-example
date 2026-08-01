#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class PX4StateReader(Node):
    """ROS 2 node chỉ đọc và in trạng thái PX4."""

    def __init__(self) -> None:
        super().__init__('px4_state_reader')

        # Khớp với QoS publisher mà PX4 tạo qua uXRCE-DDS.
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest_status = None
        self.latest_local_position = None

        self.status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            px4_qos,
        )

        self.local_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            px4_qos,
        )

        # In một lần mỗi giây thay vì in theo tốc độ 50 Hz của local position.
        self.print_timer = self.create_timer(1.0, self.print_state)

        self.get_logger().info(
            'Đang subscribe vehicle_status_v1 và vehicle_local_position_v1'
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.latest_status = msg

    def vehicle_local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ) -> None:
        self.latest_local_position = msg

    def print_state(self) -> None:
        if self.latest_status is None:
            self.get_logger().warning('Chưa nhận được vehicle_status_v1')
        else:
            status = self.latest_status

            self.get_logger().info(
                '[STATUS] '
                f'arming_state={status.arming_state}, '
                f'nav_state={status.nav_state}, '
                f'failsafe={status.failsafe}, '
                f'gcs_lost={status.gcs_connection_lost}, '
                f'preflight_ok={status.pre_flight_checks_pass}'
            )

        if self.latest_local_position is None:
            self.get_logger().warning(
                'Chưa nhận được vehicle_local_position_v1'
            )
        else:
            pos = self.latest_local_position

            self.get_logger().info(
                '[LOCAL POSITION - NED] '
                f'x={pos.x:.3f} m, '
                f'y={pos.y:.3f} m, '
                f'z={pos.z:.3f} m | '
                f'vx={pos.vx:.3f} m/s, '
                f'vy={pos.vy:.3f} m/s, '
                f'vz={pos.vz:.3f} m/s | '
                f'xy_valid={pos.xy_valid}, '
                f'z_valid={pos.z_valid}, '
                f'dead_reckoning={pos.dead_reckoning}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = PX4StateReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

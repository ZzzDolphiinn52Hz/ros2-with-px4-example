#!/usr/bin/env python3
"""
Publish ROS 2 TF + nav_msgs/Odometry from PX4 local position & attitude.

PX4 local frame: NED (x North, y East, z Down), body FRD.
ROS convention used here: ENU (x East, y North, z Up), base_link FLU-ish for 2D SLAM.

Publishes a planar odom -> base_footprint transform for 2D SLAM, plus the
full 3D base_footprint -> base_link pose and static base_link -> laser TF.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import (
    Quaternion,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def ned_to_enu_position(x_n: float, y_n: float, z_n: float) -> tuple[float, float, float]:
    # ENU: x=East, y=North, z=Up
    return y_n, x_n, -z_n


def ned_to_enu_velocity(vx_n: float, vy_n: float, vz_n: float) -> tuple[float, float, float]:
    return vy_n, vx_n, -vz_n


def ned_xy_reset_to_enu_continuity_offset(
    delta_x_n: float,
    delta_y_n: float,
) -> tuple[float, float]:
    """Offset to cancel an EKF position reset in the published ENU odom."""
    # PX4 delta_xy is new_estimate - old_estimate. ENU swaps N/E axes,
    # therefore continuity requires subtracting [delta_E, delta_N].
    return -delta_y_n, -delta_x_n


def quaternion_xyzw_to_yaw(q: np.ndarray) -> float:
    """Extract ENU yaw from a normalized geometry_msgs-style quaternion."""
    x, y, z, w = q
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_xyzw_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product for geometry_msgs-style [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def px4_quat_ned_frd_to_enu_flu(q_ned: np.ndarray) -> np.ndarray:
    """
    Convert PX4 ``vehicle_attitude.q`` to a ROS pose quaternion.

    PX4 stores [w, x, y, z] for the rotation FRD body -> NED earth.
    ROS geometry_msgs needs [x, y, z, w] for FLU body -> ENU earth.
    """
    norm = np.linalg.norm(q_ned)
    if norm == 0.0:
        raise ValueError('PX4 attitude quaternion has zero norm')
    q_ned_frd = q_ned / norm

    # Coordinate conversion matrices. Both are self-inverse, but their order
    # is significant: v_enu = R_enu_ned R_ned_frd R_frd_flu v_flu.
    r_enu_ned = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ])
    r_frd_flu = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ])

    def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def rot_to_quat_xyzw(r: np.ndarray) -> np.ndarray:
        tr = np.trace(r)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
        q = np.array([x, y, z, w], dtype=float)
        return q / np.linalg.norm(q)

    r_ned_frd = quat_wxyz_to_rot(q_ned_frd)
    r_enu_flu = r_enu_ned @ r_ned_frd @ r_frd_flu
    return rot_to_quat_xyzw(r_enu_flu)


class Px4OdomTf(Node):
    def __init__(self) -> None:
        super().__init__('px4_odom_tf')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('laser_frame', 'link')  # gz_frame_id of lidar_2d_v2
        self.declare_parameter('laser_xyz', [0.12, 0.0, 0.26])
        self.declare_parameter('publish_rate_hz', 30.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.laser_xyz = list(self.get_parameter('laser_xyz').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pos: Optional[VehicleLocalPosition] = None
        self._att: Optional[VehicleAttitude] = None
        self._xy_reset_counter: Optional[int] = None
        self._heading_reset_counter: Optional[int] = None
        self._enu_reset_offset = np.zeros(2, dtype=float)
        self._yaw_reset_offset = 0.0

        # PX4 v1.17 often exposes versioned topic names on ROS 2 (*_v1).
        self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter(
            'attitude_topic', '/fmu/out/vehicle_attitude')

        pos_topic = self.get_parameter('local_position_topic').value
        att_topic = self.get_parameter('attitude_topic').value

        self.create_subscription(
            VehicleLocalPosition, pos_topic, self._on_pos, qos)
        self.create_subscription(
            VehicleAttitude, att_topic, self._on_att, qos)
        self.get_logger().info(f'Subscribing pos={pos_topic} att={att_topic}')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self._publish_static_laser_tf()

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._publish)

        self.get_logger().info(
            'px4_odom_tf ready: NED→ENU, TF '
            f'{self.odom_frame}->{self.base_frame}->{self.body_frame}, '
            f'static {self.body_frame}->{self.laser_frame}'
        )

    def _publish_static_laser_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.body_frame
        t.child_frame_id = self.laser_frame
        t.transform.translation = Vector3(
            x=float(self.laser_xyz[0]),
            y=float(self.laser_xyz[1]),
            z=float(self.laser_xyz[2]),
        )
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self._static_tf.sendTransform(t)

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        if self._xy_reset_counter is None:
            self._xy_reset_counter = int(msg.xy_reset_counter)
        elif int(msg.xy_reset_counter) != self._xy_reset_counter:
            dx, dy = ned_xy_reset_to_enu_continuity_offset(
                float(msg.delta_xy[0]), float(msg.delta_xy[1]))
            self._enu_reset_offset += np.array([dx, dy])
            self.get_logger().warning(
                'PX4 EKF XY reset detected: '
                f'{self._xy_reset_counter}->{msg.xy_reset_counter}, '
                f'continuity offset ENU += [{dx:.3f}, {dy:.3f}] m')
            self._xy_reset_counter = int(msg.xy_reset_counter)

        if self._heading_reset_counter is None:
            self._heading_reset_counter = int(msg.heading_reset_counter)
        elif int(msg.heading_reset_counter) != self._heading_reset_counter:
            # ROS ENU yaw = pi/2 - PX4 NED heading. If PX4 heading jumps by
            # delta_heading, add that delta to the ROS correction to cancel it.
            self._yaw_reset_offset += float(msg.delta_heading)
            self.get_logger().warning(
                'PX4 EKF heading reset detected: '
                f'{self._heading_reset_counter}->{msg.heading_reset_counter}, '
                f'continuity yaw += {math.degrees(msg.delta_heading):.2f} deg')
            self._heading_reset_counter = int(msg.heading_reset_counter)

        self._pos = msg

    def _on_att(self, msg: VehicleAttitude) -> None:
        self._att = msg

    def _publish(self) -> None:
        if self._pos is None or self._att is None:
            return
        if not (self._pos.xy_valid and self._pos.z_valid):
            return

        stamp = self.get_clock().now().to_msg()
        x, y, z = ned_to_enu_position(self._pos.x, self._pos.y, self._pos.z)
        x += float(self._enu_reset_offset[0])
        y += float(self._enu_reset_offset[1])
        vx, vy, vz = ned_to_enu_velocity(self._pos.vx, self._pos.vy, self._pos.vz)

        q_ned = np.array(self._att.q, dtype=float)  # wxyz
        q_xyzw = px4_quat_ned_frd_to_enu_flu(q_ned)
        q_reset = np.array([
            0.0,
            0.0,
            math.sin(self._yaw_reset_offset / 2.0),
            math.cos(self._yaw_reset_offset / 2.0),
        ])
        q_xyzw = quaternion_xyzw_multiply(q_reset, q_xyzw)

        # 2D SLAM must not consume the drone's z/roll/pitch. Publish a planar
        # base_footprint like mobile robots do, then retain the full attitude
        # in the child base_link for visualization and other 3D consumers.
        yaw = quaternion_xyzw_to_yaw(q_xyzw)
        q_yaw = np.array([
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        ])

        # TF odom -> base_footprint (x/y/yaw only)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation = Vector3(x=x, y=y, z=0.0)
        tf.transform.rotation = Quaternion(
            x=0.0,
            y=0.0,
            z=float(q_yaw[2]),
            w=float(q_yaw[3]),
        )
        self._tf_broadcaster.sendTransform(tf)

        # TF base_footprint -> base_link (z/roll/pitch relative to yaw).
        q_yaw_inverse = np.array([0.0, 0.0, -q_yaw[2], q_yaw[3]])
        q_tilt = quaternion_xyzw_multiply(q_yaw_inverse, q_xyzw)
        body_tf = TransformStamped()
        body_tf.header.stamp = stamp
        body_tf.header.frame_id = self.base_frame
        body_tf.child_frame_id = self.body_frame
        body_tf.transform.translation = Vector3(x=0.0, y=0.0, z=z)
        body_tf.transform.rotation = Quaternion(
            x=float(q_tilt[0]),
            y=float(q_tilt[1]),
            z=float(q_tilt[2]),
            w=float(q_tilt[3]),
        )
        self._tf_broadcaster.sendTransform(body_tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self._odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OdomTf()
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

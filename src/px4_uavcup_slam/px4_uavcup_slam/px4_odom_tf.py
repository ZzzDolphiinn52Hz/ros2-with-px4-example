#!/usr/bin/env python3
"""
Publish ROS 2 TF + nav_msgs/Odometry from PX4 local position & attitude.

PX4 local frame: NED (x North, y East, z Down), body FRD.
ROS convention used here: ENU (x East, y North, z Up), base_link FLU-ish for 2D SLAM.

Also publishes static TF base_link -> laser (Gazebo lidar link name often "link").
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


def px4_quat_ned_frd_to_enu_flu(q_ned: np.ndarray) -> np.ndarray:
    """
    PX4 vehicle_attitude.q = [w, x, y, z] rotation from NED to FRD body.
    Convert to ROS ENU->FLU style quaternion [x, y, z, w] for geometry_msgs.
    """
    # q_ned as wxyz
    q_frd_ned = q_ned / np.linalg.norm(q_ned)

    # R_flu_frd: 180 deg about X  (FRD -> FLU)
    # R_enu_ned: swap axes NED -> ENU
    # Composition commonly used in PX4-ROS bridges:
    # q_enu = q_rot * q_ned * q_rot_inv  with fixed transforms
    r_enu_ned = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ])
    r_flu_frd = np.array([
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

    r_frd_ned = quat_wxyz_to_rot(q_frd_ned)
    # R_flu_enu = R_flu_frd * R_frd_ned * R_ned_enu
    # R_ned_enu = R_enu_ned.T
    r_flu_enu = r_flu_frd @ r_frd_ned @ r_enu_ned.T
    return rot_to_quat_xyzw(r_flu_enu)


class Px4OdomTf(Node):
    def __init__(self) -> None:
        super().__init__('px4_odom_tf')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'link')  # gz_frame_id of lidar_2d_v2
        self.declare_parameter('laser_xyz', [0.12, 0.0, 0.26])
        self.declare_parameter('publish_rate_hz', 30.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
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
            f'{self.odom_frame}->{self.base_frame}, static '
            f'{self.base_frame}->{self.laser_frame}'
        )

    def _publish_static_laser_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        t.transform.translation = Vector3(
            x=float(self.laser_xyz[0]),
            y=float(self.laser_xyz[1]),
            z=float(self.laser_xyz[2]),
        )
        t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self._static_tf.sendTransform(t)

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
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
        vx, vy, vz = ned_to_enu_velocity(self._pos.vx, self._pos.vy, self._pos.vz)

        q_ned = np.array(self._att.q, dtype=float)  # wxyz
        q_xyzw = px4_quat_ned_frd_to_enu_flu(q_ned)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation = Vector3(x=x, y=y, z=z)
        tf.transform.rotation = Quaternion(
            x=float(q_xyzw[0]),
            y=float(q_xyzw[1]),
            z=float(q_xyzw[2]),
            w=float(q_xyzw[3]),
        )
        self._tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()

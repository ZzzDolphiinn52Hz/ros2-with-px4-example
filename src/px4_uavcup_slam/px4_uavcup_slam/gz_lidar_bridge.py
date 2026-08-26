#!/usr/bin/env python3
from __future__ import annotations

""" about version compatibility
Gazebo Harmonic (gz-transport13 / gz-msgs10) → ROS 2 LaserScan + Clock.

Why not ros_gz_bridge?
  ros-humble-ros-gz-bridge links ignition-msgs8 / transport11 (Fortress).
  PX4 (v1.15+) SITL uses Gazebo Harmonic (gz-msgs10 / transport13).
  Result: bridge starts but prints "Unknown message type [8/9]" and /scan stays empty.

This node uses the system Python bindings that match Harmonic.
"""

""" overview
This node is the one brigde between Gazebo and ROS2.

- recv LaserScan & Clock from Gazebo
- pub to ROS2 sensor_msgs/LaserScan (/scan) & rosgraph_msgs/Clock (/clock)
- filter LaserScan by tilt and altitude from PX4 for SLAM 2D
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock as RosClock
from builtin_interfaces.msg import Time
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from gz.transport13 import Node as GzNode
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.clock_pb2 import Clock as GzClock


# calc body tilt from earth vertical, independent of yaw
def px4_quat_tilt_rad(q_wxyz) -> float:
    w, x, y, z = (float(value) for value in q_wxyz)
    norm_sq = w * w + x * x + y * y + z * z
    if norm_sq == 0.0:
        raise ValueError('PX4 attitude quaternion has zero norm')
    r33 = 1.0 - 2.0 * (x * x + y * y) / norm_sq
    return math.acos(max(-1.0, min(1.0, r33)))


class GzLidarBridge(Node):
    def __init__(self) -> None:
        super().__init__('gz_lidar_bridge')

        # declare parameters
        self.declare_parameter(
            'gz_scan_topic',
            '/world/urban_uavcup/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
        )
        self.declare_parameter('gz_clock_topic', '/clock')
        self.declare_parameter('ros_scan_topic', 'scan')
        self.declare_parameter('frame_id', 'link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('max_tilt_deg', 5.0)
        self.declare_parameter('min_mapping_altitude_m', 0.5)
        self.declare_parameter(
            'attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter(
            'local_position_topic', '/fmu/out/vehicle_local_position_v1')

        # get parameters
        self._gz_scan_topic = self.get_parameter('gz_scan_topic').value
        self._gz_clock_topic = self.get_parameter('gz_clock_topic').value
        self._ros_scan_topic = self.get_parameter('ros_scan_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value)
        self._max_tilt_rad = math.radians(
            float(self.get_parameter('max_tilt_deg').value))
        self._min_mapping_altitude_m = float(
            self.get_parameter('min_mapping_altitude_m').value)
        self._attitude_topic = self.get_parameter('attitude_topic').value
        self._local_position_topic = self.get_parameter(
            'local_position_topic').value

        # initialize shared state
        # locking protects latest_* and *_count variables
        # which are updated by subscription callbacks and read by timer callback
        self._lock = threading.Lock()
        # latest_*: updated by subscription callbacks
        self._latest_scan: Optional[GzLaserScan] = None
        self._latest_clock: Optional[GzClock] = None
        self._latest_tilt_rad: Optional[float] = None
        self._latest_altitude_m: Optional[float] = None
        # *_count: hearbeat each 2s, reset to 0 after heartbeat
        self._scan_received_count = 0
        self._scan_published_count = 0
        self._scan_rejected_count = 0
        self._scan_altitude_rejected_count = 0

        # pub ROS topics
        # laser scan
        self._scan_pub = self.create_publisher(
            LaserScan, self._ros_scan_topic, 10)
        # clock
        self._clock_pub = self.create_publisher(RosClock, '/clock', 10)

        # sub PX4 topics
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleAttitude,
            self._attitude_topic,
            self._on_attitude,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self._local_position_topic,
            self._on_local_position,
            px4_qos,
        )

        # sub Gazebo topics
        self._gz = GzNode()
        # Python binding may return None even when subscription succeeds.
        self._gz.subscribe(GzLaserScan, self._gz_scan_topic, self._on_gz_scan)
        self._gz.subscribe(GzClock, self._gz_clock_topic, self._on_gz_clock)
        self.get_logger().info(f'Subscribed GZ scan: {self._gz_scan_topic}')
        self.get_logger().info(f'Subscribed GZ clock: {self._gz_clock_topic}')

        # This node is the source of /clock. Its own timers must therefore use
        # wall time; a ROS-time timer would wait for /clock and deadlock before
        # it could publish the first clock or scan message.
        self._wall_clock = RclpyClock(clock_type=ClockType.SYSTEM_TIME)
        period = 1.0 / self._publish_rate_hz
        self.create_timer(period, self._publish, clock=self._wall_clock)
        self.create_timer(2.0, self._heartbeat, clock=self._wall_clock)

    def _on_gz_scan(self, msg: GzLaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._scan_received_count += 1

    def _on_gz_clock(self, msg: GzClock) -> None:
        with self._lock:
            self._latest_clock = msg

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        # The angle between body-down (FRD z) and earth-down (NED z) is the
        # total roll/pitch tilt, independent of yaw. For a normalized
        # quaternion R_ned_frd[2, 2] = 1 - 2(x^2 + y^2).
        try:
            tilt = px4_quat_tilt_rad(msg.q)
        except ValueError:
            return
        with self._lock:
            self._latest_tilt_rad = tilt

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        if not msg.z_valid:
            return
        # PX4 local position is NED: negative z means height above local origin.
        with self._lock:
            self._latest_altitude_m = max(0.0, -float(msg.z))

    def _heartbeat(self) -> None:
        with self._lock:
            received = self._scan_received_count
            published = self._scan_published_count
            rejected = self._scan_rejected_count
            altitude_rejected = self._scan_altitude_rejected_count
            tilt = self._latest_tilt_rad
            altitude = self._latest_altitude_m
            self._scan_received_count = 0
            self._scan_published_count = 0
            self._scan_rejected_count = 0
            self._scan_altitude_rejected_count = 0
        tilt_text = 'unknown' if tilt is None else f'{math.degrees(tilt):.1f}deg'
        altitude_text = 'unknown' if altitude is None else f'{altitude:.2f}m'
        self.get_logger().info(
            'LaserScan last 2s: '
            f'GZ={received} ROS={published} tilt_rejected={rejected} '
            f'altitude_rejected={altitude_rejected} '
            f'tilt={tilt_text} altitude={altitude_text}')

    # node publishes /scan and /clock at a fixed rate
    # using the latest received messages
    # pub /scan to SLAM ToolBox, AMCL, Nav2, Rviz
    # pub /clock to all node for correct time sync, including this node itself
    def _publish(self) -> None:
        with self._lock:
            scan = self._latest_scan
            clk = self._latest_clock
            tilt = self._latest_tilt_rad
            altitude = self._latest_altitude_m

        scan_stamp = None
        if clk is not None:
            c = RosClock()
            # Prefer sim time
            sec = int(clk.sim.sec)
            nsec = int(clk.sim.nsec)
            if sec == 0 and nsec == 0:
                sec = int(clk.system.sec)
                nsec = int(clk.system.nsec)
            scan_stamp = Time(sec=sec, nanosec=nsec)
            c.clock = scan_stamp
            self._clock_pub.publish(c)

        if scan is None:
            return

        if (
            self._min_mapping_altitude_m > 0.0
            and (
                altitude is None
                or altitude < self._min_mapping_altitude_m
            )
        ):
            with self._lock:
                self._scan_altitude_rejected_count += 1
            return

        if (
            self._max_tilt_rad > 0.0
            and (
                tilt is None
                or tilt > self._max_tilt_rad
            )
        ):
            with self._lock:
                self._scan_rejected_count += 1
            return

        out = LaserScan()
        # Stamp directly from Gazebo time so scan and /clock are coherent even
        # before the ROS clock subscription has processed its first message.
        out.header.stamp = (
            scan_stamp if scan_stamp is not None
            else self.get_clock().now().to_msg()
        )
        # Gazebo commonly reports a scoped sensor path here, while the ROS TF
        # tree intentionally uses the short frame configured by `frame_id`.
        # Publishing the scoped Gazebo name makes RViz/slam_toolbox reject the
        # scan because no matching TF exists.
        out.header.frame_id = self._frame_id

        out.angle_min = float(scan.angle_min)
        out.angle_max = float(scan.angle_max)
        out.angle_increment = float(scan.angle_step)
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = float(scan.range_min)
        out.range_max = float(scan.range_max)
        out.ranges = [float(r) for r in scan.ranges]
        out.intensities = [float(i) for i in scan.intensities]

        self._scan_pub.publish(out)
        with self._lock:
            self._scan_published_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzLidarBridge()
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

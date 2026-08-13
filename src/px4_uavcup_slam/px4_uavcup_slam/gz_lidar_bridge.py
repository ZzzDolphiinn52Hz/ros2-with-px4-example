#!/usr/bin/env python3
"""
Gazebo Harmonic (gz-transport13 / gz-msgs10) → ROS 2 LaserScan + Clock.

Why not ros_gz_bridge?
  ros-humble-ros-gz-bridge links ignition-msgs8 / transport11 (Fortress).
  PX4 SITL uses Gazebo Harmonic (gz-msgs10 / transport13).
  Result: bridge starts but prints "Unknown message type [8/9]" and /scan stays empty.

This node uses the system Python bindings that match Harmonic.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

from gz.transport13 import Node as GzNode
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.clock_pb2 import Clock as GzClock


class GzLidarBridge(Node):
    def __init__(self) -> None:
        super().__init__('gz_lidar_bridge')

        self.declare_parameter(
            'gz_scan_topic',
            '/world/urban_uavcup/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
        )
        self.declare_parameter('gz_clock_topic', '/clock')
        self.declare_parameter('ros_scan_topic', 'scan')
        self.declare_parameter('frame_id', 'link')
        self.declare_parameter('publish_rate_hz', 30.0)

        self._gz_scan_topic = self.get_parameter('gz_scan_topic').value
        self._gz_clock_topic = self.get_parameter('gz_clock_topic').value
        self._frame_id = self.get_parameter('frame_id').value

        self._scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('ros_scan_topic').value, 10)
        self._clock_pub = self.create_publisher(Clock, '/clock', 10)

        self._lock = threading.Lock()
        self._latest_scan: Optional[GzLaserScan] = None
        self._latest_clock: Optional[GzClock] = None
        self._scan_count = 0

        self._gz = GzNode()
        # Python binding may return None even when subscription succeeds.
        self._gz.subscribe(GzLaserScan, self._gz_scan_topic, self._on_gz_scan)
        self._gz.subscribe(GzClock, self._gz_clock_topic, self._on_gz_clock)
        self.get_logger().info(f'Subscribed GZ scan: {self._gz_scan_topic}')
        self.get_logger().info(f'Subscribed GZ clock: {self._gz_clock_topic}')

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._publish)
        self.create_timer(2.0, self._heartbeat)

    def _on_gz_scan(self, msg: GzLaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._scan_count += 1

    def _on_gz_clock(self, msg: GzClock) -> None:
        with self._lock:
            self._latest_clock = msg

    def _heartbeat(self) -> None:
        with self._lock:
            n = self._scan_count
            self._scan_count = 0
        self.get_logger().info(f'GZ LaserScan msgs in last 2s: {n}')

    def _publish(self) -> None:
        with self._lock:
            scan = self._latest_scan
            clk = self._latest_clock

        if clk is not None:
            c = Clock()
            # Prefer sim time
            sec = int(clk.sim.sec)
            nsec = int(clk.sim.nsec)
            if sec == 0 and nsec == 0:
                sec = int(clk.system.sec)
                nsec = int(clk.system.nsec)
            c.clock = Time(sec=sec, nanosec=nsec)
            self._clock_pub.publish(c)

        if scan is None:
            return

        out = LaserScan()
        # Prefer ROS clock (sim if use_sim_time)
        out.header.stamp = self.get_clock().now().to_msg()
        # frame_id: Gazebo sets msg.frame; fallback to param
        out.header.frame_id = scan.frame if scan.frame else self._frame_id

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

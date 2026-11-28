#!/usr/bin/env python3
"""Read-only readiness report for the manually gated SITL landing workflow."""

from __future__ import annotations

import argparse
import math
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from px4_uavcup_control.landing.capture_clock import CaptureClock, stamp_ns
from px4_msgs.msg import (
    VehicleAttitude,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock


def _diagnostic_values(message: DiagnosticArray) -> dict[str, str]:
    if not message.status:
        return {}
    return {item.key: item.value for item in message.status[0].values}


class LandingReadiness(Node):
    """Collect the minimum PX4 and ArUco state needed for a go/no-go check."""

    def __init__(self) -> None:
        super().__init__('sim_landing_readiness')
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_status = None
        self.local_position = None
        self.attitude = None
        self.land_detected = None
        self.aruco_status = None
        self.landing_status = None
        self.capture_clock = CaptureClock()
        self.create_subscription(
            Clock, '/uav/sim/clock',
            lambda message: self.capture_clock.update(
                stamp_ns(message.clock), time.monotonic()), 1)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            lambda message: setattr(self, 'vehicle_status', message),
            px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            lambda message: setattr(self, 'local_position', message),
            px4_qos)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            lambda message: setattr(self, 'attitude', message),
            px4_qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            lambda message: setattr(self, 'land_detected', message),
            px4_qos)
        self.create_subscription(
            DiagnosticArray, '/uav/aruco/status',
            lambda message: setattr(self, 'aruco_status', message), 10)
        self.create_subscription(
            DiagnosticArray, '/uav/aruco/landing_status',
            lambda message: setattr(self, 'landing_status', message), 10)

    def base_messages_ready(self) -> bool:
        return all(value is not None for value in (
            self.vehicle_status,
            self.local_position,
            self.attitude,
            self.land_detected,
            self.aruco_status,
            self.landing_status,
            self.capture_clock.latest_ns,
        ))


def _report(node: LandingReadiness, stage: str) -> bool:
    status = node.vehicle_status
    local = node.local_position
    land = node.land_detected
    aruco = _diagnostic_values(node.aruco_status)
    landing = _diagnostic_values(node.landing_status)

    checks = {
        'PX4 messages received': node.base_messages_ready(),
        'PX4 preflight checks passed': bool(status.pre_flight_checks_pass),
        'PX4 not in failsafe': not bool(status.failsafe),
        'PX4 local XY valid': bool(local.xy_valid),
        'PX4 local Z valid': bool(local.z_valid),
        'PX4 attitude finite': all(math.isfinite(float(value))
                                   for value in node.attitude.q),
    }
    if stage in ('takeover', 'offboard'):
        fresh = node.capture_clock.is_fresh(
            stamp_ns(node.aruco_status.header.stamp), time.monotonic(), 0.35)
        checks.update({
            'vehicle armed': int(status.arming_state)
            == int(VehicleStatus.ARMING_STATE_ARMED),
            'vehicle airborne': not bool(land.landed),
            'ArUco capture fresh': fresh,
            'ArUco target visible': fresh and aruco.get('target_visible') == 'true',
            'ArUco target is ID 0': aruco.get('target_marker_id') == '0',
        })
    if stage == 'offboard':
        checks['PX4 nav_state is OFFBOARD'] = (
            int(status.nav_state)
            == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD))

    print(f'=== SITL landing readiness: {stage} ===')
    for label, passed in checks.items():
        print(f'{"PASS" if passed else "FAIL":4}  {label}')
    print(
        f'PX4 arming_state={int(status.arming_state)} '
        f'nav_state={int(status.nav_state)} landed={bool(land.landed)}')
    print(
        f'ArUco visible={aruco.get("target_visible", "unknown")} '
        f'id={aruco.get("target_marker_id", "unknown")} '
        f'latency_ms={aruco.get("latency_ms", "unknown")}')
    print(
        f'Landing enabled={landing.get("enabled", "unknown")} '
        f'state={landing.get("state", "unknown")} '
        f'error_m={landing.get("horizontal_error_m", "unknown")}')
    return all(checks.values())


def main(args=None) -> None:
    cli = argparse.ArgumentParser()
    cli.add_argument(
        '--stage', choices=('startup', 'takeover', 'offboard'),
        default='takeover')
    cli.add_argument('--timeout', type=float, default=8.0)
    parsed = cli.parse_args(remove_ros_args(args=sys.argv)[1:])
    if parsed.timeout <= 0.0:
        cli.error('--timeout must be positive')

    rclpy.init(args=args)
    node = LandingReadiness()
    deadline = time.monotonic() + parsed.timeout
    try:
        while time.monotonic() < deadline and not node.base_messages_ready():
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.base_messages_ready():
            print('FAIL  timed out waiting for PX4/ArUco/landing status or Gazebo clock')
            raise SystemExit(1)
        raise SystemExit(0 if _report(node, parsed.stage) else 1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

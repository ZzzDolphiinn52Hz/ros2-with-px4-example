# Verification: Gazebo ArUco Landing End-to-End

**Date**: 2026-11-28
**Base commit**: `e84de55`
**Physical hardware/flight**: NOT RUN

- `./uavcup sim-setup`: 6 packages built. Host `colcon test` and
  `colcon test-result --verbose`: 109 tests, zero failures. `bash -n uavcup`
  and `shellcheck uavcup` passed.
- Docker `./uavcup unit-test` and `./uavcup doctor`: unavailable on this PC.
- `sim-landing-gazebo` (headless), `sim-landing-agent`, `sim-landing-px4`,
  `sim-qgc`, and `sim-landing-stack`: PX4 attached to the free-flight world,
  XRCE used UDP 8890 and domain 42, and PX4 reported a localhost MAVLink
  partner and Ready for takeoff.
- `sim-landing-check startup`: PASS with preflight, local XY/Z and attitude
  valid. PX4 reported arming state 1, nav state 4, and landed true.
- `sim-request-offboard` with adapter disabled and `sim-landing-start` with
  PID disabled returned `success=False`. Explicit PID enable left it
  enabled and idle. It was then disabled.
- `sim-landing-record`: startup bag
  `artifacts/gazebo_aruco_landing/landing_20261128_075201` contains 9,213
  messages over 54.4 seconds. Counts were zero for cmd_vel, heartbeat,
  trajectory setpoint, and vehicle command. Bridge publisher endpoints are
  present when the full stack is loaded. The detector-only profile is separate.
- Explicit adapter enable while disarmed produced zero horizontal velocity and
  an altitude target clamped to 0.20 m. Adapter disable stopped publication.
  PX4 was kept disarmed throughout these checks.
- NOT RUN: manual QGC arm, takeoff, approach, airborne Offboard takeover,
  descent, landing/disarm and flight rosbag. Follow
  `docs/GAZEBO_ARUCO_LANDING_E2E.md`.
- NOT RUN: Raspberry Pi synchronization and physical hardware checks.

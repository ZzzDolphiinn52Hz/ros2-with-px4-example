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

## Camera freshness follow-up

Base `bd0b7de`. Camera-only verification used `UAV_SIM_HEADLESS=1 ./uavcup
sim-gazebo` and the production detector with `input_mode:=ros_topic`,
without PX4, PID or the adapter running. The bridge probe instruments callback
entry, ROS publisher entry, ROS receipt and pose receipt. A Gazebo clock
subscriber in a separate process is the time reference. The local probe is
`artifacts/sitl_camera_latency_probe.py` and needs the host ROS/workspace setup
and the SITL domain 42 environment. Run it instead of a second camera bridge.

Median capture age per 5 s window, in seconds of Gazebo simulation time:

| Revision | Raw Gazebo receiver | Bridge callback entry | ROS Image publish | ROS Image receipt |
|---|---|---|---|---|
| Baseline | 0.586 / 1.360 / 2.148 | 0.520 / 1.300 / 2.074 | 0.598 / 1.372 / 2.154 | 0.604 / 1.360 / 2.148 |
| Patched | 0.008 / 0.008 / 0.008 | 0.008 / 0.008 / 0.008 | 0.016 / 0.016 / 0.016 | 0.020 / 0.020 / 0.020 |

The baseline raw-image observer shares the Python process with the instrumented
bridge. The clock observer is separate to avoid delayed clock callbacks masking
the increasing image age. The comparable 15 s runs observed 191 and 225 calls
at the ROS publisher. Worst publish age was 2.556 s and 0.064 s, respectively.
The earlier 199 s field observation was not reproduced at that magnitude.
The fixed-size 640×480 RGB conversion took 76.25 ms versus 0.62 ms in a
microbenchmark. The change avoids the per-byte ROS message setter validation
and moves conversion/publish out of transport callbacks.

The 60 s patched run with the independent clock recorded 900 image publish
calls (15 FPS). Publish age had a 20 ms median in each of the 12 windows and
an 88 ms maximum. Pose age peaked at 92 ms including startup. No accumulating
lag was observed.

`./uavcup sim-setup` built six packages. Host `colcon test` across perception,
control, bridge, bringup and simulation reported 114 tests and zero failures.
Regression tests verify latest-frame replacement, rejection of increasing
but stale timestamps in PID/readiness, and checks for pause/reset.
`./uavcup unit-test` and `./uavcup doctor` could not start because Docker
is unavailable on the PC.

No flight or Pi tests were performed for this patch. In a separate static
raw-image test around 1.7 m, the native detector recognized 0/24, versus 24/24
when the same frames were enlarged 3× offline. Upscaling is not in the patch.
The adapter altitude target on ABORT remains unchanged.

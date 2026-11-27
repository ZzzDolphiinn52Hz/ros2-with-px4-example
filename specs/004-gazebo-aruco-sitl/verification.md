# Verification: Gazebo ArUco PX4 SITL

**Date**: 2026-11-27
**Base commit**: `d7cb9fb`
**Implementation commit**: `6f931d6`
**Hardware/flight**: NOT RUN

## Result

Phase 1 and phase 2 PASS on the development PC. PX4 v1.17 SITL remained
disarmed throughout. No landing PID, PX4 adapter, arm command or Offboard
request was started by the simulation detector profile.

## Build and automated tests

```bash
./uavcup sim-doctor
./uavcup sim-setup
```

PASS:

- Gazebo Sim `8.15.0`, PX4 SITL, Micro XRCE-DDS Agent and
  `gz.transport13` were found.
- Six packages built successfully, including `px4_uavcup_simulation`.
- The pre-existing dirty state in the external PX4 checkout was observed but
  not modified (`Tools/simulation/gz`).

Host-side ROS test command:

```bash
set +u
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
colcon test --event-handlers console_direct+ --packages-select \
  px4_uavcup_perception px4_uavcup_control \
  px4_uavcup_px4_bridge px4_uavcup_bringup \
  px4_uavcup_simulation
colcon test-result --verbose
```

PASS: 104 tests, 0 errors, 0 failures, 0 skipped.

`bash -n uavcup` and `shellcheck uavcup` also PASS.

The standard Docker commands `./uavcup unit-test` and `./uavcup doctor` are
NOT RUN in their intended environment because Docker is not installed on this
development PC. They exited before starting tests. The host-side build and
equivalent package test suite above passed.

## Phase 1: PX4 SITL and ROS 2

The world, agent and PX4 were started with:

```bash
UAV_SIM_HEADLESS=1 ./uavcup sim-gazebo
./uavcup sim-agent
./uavcup sim-px4
```

PASS:

- PX4 attached to the existing Gazebo entity `x500_uavcup_aruco`.
- Micro XRCE-DDS synchronized over UDP `127.0.0.1:8888`.
- `/fmu/out/vehicle_status_v1`,
  `/fmu/out/vehicle_local_position_v1` and
  `/fmu/out/vehicle_attitude` each had one PX4 publisher.
- Vehicle status reported `arming_state: 1` (disarmed), `failsafe: false`.
- Local position reported `xy_valid: true`, `z_valid: true`,
  `v_xy_valid: true`, `v_z_valid: true`.
- `/fmu/in/trajectory_setpoint` had `Publisher count: 0` while the
  detector-only profile was running.

This run exposed an existing PX4 v1.17 contract mismatch. The real topic is
`/fmu/out/vehicle_local_position_v1`; the landing controller, bridge defaults,
YAML, contract documentation and regression tests were updated accordingly.

## Phase 2: simulated camera and production detector

The detector-only profile was started with:

```bash
./uavcup sim-aruco
./uavcup sim-status
```

PASS:

- Gazebo published 640x480 image and CameraInfo from the centred down camera.
- The ROS camera stream sustained approximately 8--11 Hz under the concurrent
  headless SITL/test workload; the requested sensor update rate is 15 Hz.
- `/uav/aruco/status` reported `marker_count=1`,
  `target_marker_id=0`, `target_visible=true`.
- `/uav/aruco/ids` contained `[0]`.
- `/uav/aruco/target_pose` reported approximately
  `x=-0.00109 m`, `y=-0.00161 m`, `z=0.90104 m`, consistent with the
  detector stand geometry.
- A repeated detector launch stopped both detector and camera adapter cleanly
  with `Ctrl+C`; Gazebo transport callbacks were unsubscribed before ROS
  publisher destruction.

The Gazebo Harmonic Ogre2 camera uses the measured IMX500 intrinsic matrix but
an ideal pinhole distortion vector (`D=0`). Real-camera distortion parameters
remain unchanged. This test validates software integration, not real-lens
accuracy or flight readiness.

## Not run / next boundary

- Gazebo GUI operator check: NOT RUN; the same world passed headless.
- PID, PX4 adapter enable, arm, Offboard and vehicle motion: NOT RUN and out of
  scope for phases 1--2.
- Raspberry Pi synchronization: NOT RUN because this feature is PC-only and
  excluded from the default Pi build.

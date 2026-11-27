# Feature Specification: Gazebo ArUco PX4 SITL

**Feature Branch**: `feature/gazebo-aruco-sitl`
**Created**: 2026-11-27
**Status**: Phase 1 and phase 2 complete

## Goal

Provide a PC-only Gazebo/PX4 SITL environment that replaces the physical
vehicle, IMX500 and UART while preserving the existing ROS 2 ArUco detector,
landing-controller and PX4 topic contracts.

## Phase 1 Acceptance Criteria

- PX4 v1.17 SITL attaches to a Gazebo X500 model without modifying the dirty
  PX4 Gazebo-model submodule.
- Micro XRCE-DDS Agent uses UDP port 8888 and exposes the expected
  `/fmu/in/*` and `/fmu/out/*` topics.
- The simulation remains manually armed and manually switched to Offboard.

## Phase 2 Acceptance Criteria

- A workspace-owned world contains a `DICT_5X5_50`, ID 0 marker whose black
  outer square is 80 mm.
- The X500 camera is 640x480 at 15 Hz, looks down from the vehicle center, and
  has image-top aligned with body forward.
- Gazebo image and camera-info messages reach the existing detector through
  ROS topics, without Picamera2 or a Unix camera socket.
- The existing detector publishes ID 0 and `/uav/aruco/target_pose` when the
  marker is visible.
- PID and PX4 adapter remain disabled; phase 2 publishes no motion command.

## Compatibility and Safety

- Real Raspberry Pi launch profiles and camera calibration remain available.
- Simulation-specific dependencies are optional and are not added to the
  Raspberry Pi default setup path.
- No command automatically arms, enables the landing controller, enables the
  PX4 adapter, or requests Offboard.
- Hardware and flight readiness remain NOT RUN.

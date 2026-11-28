# Feature Specification: Gazebo ArUco Landing End-to-End

**Feature Branch**: `feature/gazebo-aruco-landing-e2e`
**Created**: 2026-11-27
**Status**: Implemented and verified disarmed. Operator flight pending.

## Goal

Extend the verified Gazebo/PX4 SITL perception environment into a realistic,
operator-gated precision-landing workflow: QGroundControl performs manual arm,
takeoff and approach; the existing ROS 2 detector, landing controller and PX4
adapter then take over only after each explicit service gate is invoked.

## Acceptance Criteria

- A free-flight world starts the X500 on the ground away from the 80 mm ID 0
  target and contains no invisible detector stand.
- QGroundControl can connect to PX4 SITL and remains the only documented path
  for arm, takeoff and manual approach to the target.
- A simulation launch composes the Gazebo camera bridge, production detector,
  landing PID and production PX4 adapter.
- PID and adapter start disabled. Starting the stack creates no Offboard
  request and does not arm the vehicle.
- Enabling the landing controller does not start motion; a separate
  `start_landing` request is required.
- The documented operator sequence explicitly checks target visibility and
  PX4 health, enables the adapter, waits for setpoint heartbeat, requests
  Offboard, verifies ACK/nav state, enables PID, and starts landing.
- Abort, return-to-POSCTL and disable commands are documented as distinct
  operator actions.
- A ROS bag profile records pose, command, state, PX4 status/local position,
  setpoints and command acknowledgements for post-flight analysis.
- Software tests and a SITL end-to-end landing run provide evidence; physical
  flight remains NOT RUN.

## Safety Boundaries

- No command or launch file automatically arms, enables PID/adapter, or
  requests Offboard.
- Operator actions remain one gate per command; no bundled landing-start
  script bypasses intermediate verification.
- The detector-only phase-2 profile continues to publish no `/fmu/in/*`.
- The free-flight simulation is not evidence that the physical UAV is safe to
  fly.

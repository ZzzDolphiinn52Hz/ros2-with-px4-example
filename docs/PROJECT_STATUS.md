# Project Handoff Status

**Updated**: 2026-11-28

**Active feature**: `specs/005-gazebo-aruco-landing-e2e/`

**Branch**: `feature/gazebo-aruco-landing-e2e`
**State**: Free-flight stack implemented and verified disarmed. Operator flight pending.

## Last Completed

- Operator confirmed the phase 1–2 GUI check. Added the free-flight world,
  QGC entry point, production ROS pipeline, explicit service commands and a
  landing rosbag profile.
- Verified QGC connection, PX4 telemetry, disabled startup and distinct PID
  enable/start gates. See `specs/005-gazebo-aruco-landing-e2e/verification.md`.

- Added the optional PC-only `px4_uavcup_simulation` package with a Gazebo
  Harmonic X500/down-camera model, 80 mm ID 0 marker cargo and detector world.
- Added a Harmonic-native one-way camera adapter and reused the production
  ArUco detector through the ROS image/CameraInfo contract.
- Added safe `./uavcup sim-*` commands for setup, Gazebo, XRCE Agent, PX4 SITL,
  detector and combined status evidence.
- Verified PX4 v1.17 DDS, valid SITL local position, simulated camera, marker
  ID 0 and target pose while PX4 remained disarmed and `/fmu/in/trajectory_setpoint`
  had zero publishers.
- Corrected the PX4 v1.17 local-position topic contract to
  `/fmu/out/vehicle_local_position_v1` in bridge/control config and defaults.
- Updated the latest centred camera mount offset to `[0.0, 0.0, 0.0]` while
  retaining the separate 0.15 m lens-to-contact vertical clearance.

- Generated an A4 `DICT_5X5_50`, ID 0 target with an 80 mm black outer
  square and a matching 300 DPI PNG.
- Changed the ArUco runtime contract from the former 160 mm target to
  `marker_size_m: 0.08`, while leaving IMX500 intrinsics and mounting geometry
  unchanged.
- Added a regression test that keeps dictionary, target ID and physical size
  synchronized with the printed target.
- Recorded a 10-minute GPIO-powered Raspberry Pi 5 IMX500 + ArUco bench test
  with one-second CSV sampling and PNG plots.
- Stored the accepted test evidence and limits in
  `specs/002-pi-gpio-power-validation/verification.md` and the Pi artifact
  directory documented there.
- Added the repeatable manual operator procedure in
  `docs/PI_GPIO_POWER_TEST.md`.
- Preserved the first sampler attempt as invalid evidence because its voltage
  parser was corrected before the accepted run.
- Initialized Spec Kit 1.4.3 with the agent-neutral `none` integration.
- Added the project constitution and shared `AGENTS.md` entry point.
- Added architecture, ROS contract, and multi-agent workflow documentation.
- Added a compact, read-only `./uavcup context` handoff command.
- Added the mandatory Raspberry Pi SSH/push/pull/commit-comparison gate.
- Completed shell, Spec Kit, integration-key, and artifact verification.

## Last Evidence

Read `specs/004-gazebo-aruco-sitl/verification.md`. Host build passed and the
five project packages produced 104 passing tests. Headless end-to-end evidence
showed target ID 0, `target_visible=true`, pose z approximately 0.901 m, valid
PX4 local position, disarmed status and no trajectory-setpoint publisher.

## Working Tree

Phase 1–2 implementation is recorded in `6f931d6`. Phase 3 verification
covers the feature branch with the disarmed test procedure documented in
`specs/005-gazebo-aruco-landing-e2e/verification.md`. Raspberry Pi sync is
NOT RUN. The shared PID service contract now requires enable and start as
separate calls.

## Next Action

Run the operator QGC flight and ROS takeover procedure in
`docs/GAZEBO_ARUCO_LANDING_E2E.md` and record airborne Offboard, landing
state transitions, touchdown and disarm in the flight bag.

## Known Constraints

- Agent-specific integration files are intentionally not committed. Each
  checkout selects `codex`, `claude`, `grok`, or `agy` locally.
- Camera/UART/flight checks require Raspberry Pi hardware and remain separate
  from software-only verification.
- Gazebo Ogre2 does not render Brown lens distortion, so the simulation uses
  the measured IMX500 intrinsic matrix with `D=0`; real calibration is unchanged.
- The accepted power artifacts are on the Pi and are not committed because
  `artifacts/` is intentionally ignored. Their path and summary are recorded
  in the feature verification file.
- Chat history is not project state. Future agents must update this file and
  the active feature artifacts before handoff.

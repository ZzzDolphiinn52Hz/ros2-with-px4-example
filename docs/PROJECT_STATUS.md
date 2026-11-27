# Project Handoff Status

**Updated**: 2026-11-27

**Active feature**: `specs/004-gazebo-aruco-sitl/`

**Branch**: `feature/gazebo-aruco-sitl`
**State**: Gazebo/PX4 SITL phase 1 and phase 2 complete; operator GUI check next

## Last Completed

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

The completed feature is ready to commit on branch
`feature/gazebo-aruco-sitl`. The external PX4 checkout retained its prior dirty
`Tools/simulation/gz` state and was not modified. Raspberry Pi synchronization
is NOT RUN because the simulation package is intentionally PC-only.

## Next Action

Run the documented four-terminal workflow with the Gazebo GUI and confirm the
camera view and ID 0 pose. The next feature may then add a separate free-flight
world and manually gated PID/Offboard tests; phases 1--2 do not authorize arm
or flight.

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

# UAV Cup ROS 2 Workspace Constitution

## Core Principles

### I. Flight Safety Is Non-Negotiable

No software path may arm the vehicle, enable a controller, or request PX4
Offboard mode automatically. Landing PID and PX4 adapters MUST start disabled.
Test-only profiles MUST NOT create publishers to `/fmu/in/*`. Any change that
can affect motion MUST preserve bounded commands, stale-input handling, explicit
enable gates, and a safe disabled state. Hardware validation MUST be described
as a manual bench test with propellers removed; an AI agent may never represent
software-only tests as flight approval.

### II. ROS Contracts Are Explicit

Every changed ROS topic, service, parameter, frame, unit, QoS policy, timeout,
or launch profile MUST be reflected in `docs/ROS2_CONTRACTS.md`, the owning
package README/config, and tests where practical. Optical, body-FLU, and PX4-NED
frames MUST never be mixed implicitly. Relative ZipDepth output MUST NOT be
described or consumed as metric distance unless real-camera metric calibration
is enabled and verified.

### III. Ownership and Sources of Truth Stay Singular

Component configuration belongs to its package:
`px4_uavcup_perception/config`, `px4_uavcup_control/config`, or
`px4_uavcup_px4_bridge/config`. Bringup composes nodes and MUST NOT duplicate
component configuration. `src/px4_msgs` is the PX4 v1.17 submodule and MUST be
treated as an external dependency unless a specification explicitly changes
the message version. Neural-network models and calibration captures remain
external artifacts and MUST NOT be committed accidentally.

### IV. Completion Requires Reproducible Evidence

Every implementation task MUST name its affected paths and verification
command. At minimum, relevant Python tests and `./uavcup unit-test` MUST pass
when the ROS workspace is available. Build, launch, hardware, camera, UART, and
PX4 checks MUST be separated so unavailable hardware is reported as `NOT RUN`,
never as passed. Bug fixes MUST include a regression test when the behavior can
be reproduced without hardware.

### V. Handoffs Are Durable and Agent-Neutral

Git-tracked files are the shared memory between machines and AI agents. Each
bounded change MUST have a feature directory under `specs/` containing current
intent (`spec.md`), approach (`plan.md`), executable work (`tasks.md`), and
verification evidence. Before ending a work session, the agent MUST update task
checkboxes and `docs/PROJECT_STATUS.md`, record the last verified commit or
working-tree state, and identify the next concrete action. Specs reduce context
discovery but never authorize editing without inspecting the relevant source,
config, and tests.

When Raspberry Pi deployment is in scope, handoff is incomplete until the SSH
connection is checked, both working trees are inspected, the completed branch
is pushed to the shared Git remote, the Pi updates with a fast-forward-only
pull, and local/Pi commit plus submodule revisions match. Divergence, local Pi
edits, or unavailable credentials MUST stop synchronization and be recorded;
agents MUST NOT use force-push, hard reset, or bidirectional file copying to
hide the conflict.

## Platform and Runtime Constraints

- Target computer: Raspberry Pi 5 running Raspberry Pi OS 64-bit.
- ROS runtime: ROS 2 Humble in the repository Docker environment.
- Autopilot contract: PX4 v1.17 through Micro XRCE-DDS on TELEM2/UART.
- Camera host services use Picamera2 outside the ROS container and local Unix
  sockets for raw frames.
- IMX500 calibration is valid only for the documented sensor mode, resolution,
  and physical mount. A changed setup requires recalibration.
- The ZipDepth model is supplied outside Git. Relative outputs are the default;
  metric depth and point clouds remain blocked without trusted calibration.
- `report_thesis2/` and `report_thesis_final/` are reference material and do
  not participate in build or runtime behavior.

## Development and Review Workflow

1. Start from a clean, reviewable branch and select one bounded feature.
2. Read `AGENTS.md`, this constitution, `docs/PROJECT_STATUS.md`, and the active
   feature artifacts before opening source files.
3. Define observable behavior and compatibility boundaries in `spec.md`.
4. In `plan.md`, identify affected packages, interfaces, safety gates, and the
   verification matrix before implementation.
5. In `tasks.md`, use exact paths, dependencies, and commands; one agent owns a
   task at a time.
6. Inspect only the relevant implementation, configuration, launch, and tests,
   then make the smallest coherent change.
7. Update contracts and documentation in the same change as behavior.
8. Record commands and outcomes in `verification.md`; update project handoff
   state before switching machine, model, or agent.
9. For Raspberry Pi work, follow `docs/RASPI_SYNC.md` and record SSH, push,
   fast-forward pull, commit comparison, and hardware checks separately.

## Governance

This constitution overrides feature plans and agent suggestions. Amendments
require a reviewed change that explains the reason, compatibility impact, and
any migration needed for existing specs. Patch releases clarify wording;
minor releases add principles or materially expand constraints; major releases
remove or redefine existing guarantees. Reviews MUST explicitly check safety,
ROS contracts, test evidence, and handoff consistency.

**Version**: 1.0.0 | **Ratified**: 2026-11-22 | **Last Amended**: 2026-11-22

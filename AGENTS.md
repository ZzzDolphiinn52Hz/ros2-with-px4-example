# Agent Entry Point

This repository is designed to move between machines and AI coding agents.
Git-tracked specifications and documentation are the shared memory; chat
history is not.

## Read Order

Read these files before changing code:

1. `.specify/memory/constitution.md` — non-negotiable safety and quality rules.
2. `docs/PROJECT_STATUS.md` — active feature, last evidence, next action.
3. The active directory under `specs/` — intent, plan, tasks, verification.
4. `docs/ARCHITECTURE.md` — package and runtime boundaries.
5. `docs/ROS2_CONTRACTS.md` — ROS topics, services, frames, units, and profiles.
6. `docs/RASPI_SYNC.md` — required SSH/Git gate when Raspberry Pi is in scope.
7. Only then read the source, config, launch files, and tests named by the plan.

Run `./uavcup context` for a compact local checkout summary.

## Working Rules

- One bounded feature and one branch per agent/session. Do not let multiple
  agents edit the same checkout concurrently.
- Treat `spec.md` as intended behavior, code as implementation, and tests as
  evidence. Resolve disagreement explicitly; do not silently choose one.
- Update `tasks.md`, the feature `verification.md`, and
  `docs/PROJECT_STATUS.md` before handing work to another agent.
- A checked task requires evidence. Use `NOT RUN` for unavailable hardware.
- Inspect relevant code before editing even when the spec is detailed. The
  documentation is an index, not a replacement for implementation context.
- Preserve unrelated user changes and do not edit generated `build/`,
  `install/`, or `log/` trees.
- Do not modify `src/px4_msgs` unless the active spec explicitly requires a PX4
  message-version change.
- Use the shared Git remote to synchronize machines. Do not `rsync`, `scp`,
  force-push, or hard-reset one source tree over another.
- Raspberry Pi handoff requires SSH verification, clean/divergence checks,
  push, fast-forward-only pull, and matching commit/submodule revisions. If any
  gate cannot run, record it as `NOT RUN` or blocked rather than claiming sync.

## Safety Boundaries

- Never introduce automatic arm, automatic Offboard request, or automatic
  controller enable behavior.
- Landing PID and PX4 bridge start disabled.
- Perception test profiles must not publish `/fmu/in/*`.
- Do not call relative ZipDepth values metres.
- Hardware or flight readiness requires explicit human-run bench procedures;
  passing unit tests is not flight approval.

## Standard Verification

```bash
./uavcup context
./uavcup unit-test
./uavcup doctor
```

`unit-test` requires an existing ROS workspace build. `doctor` observes local
hardware and environment; it may legitimately report missing Pi-only devices
on a development machine.

## Spec Kit Across Agents

The repository is initialized with the agent-neutral `none` integration. On a
machine with Specify CLI installed, select the local agent without changing the
shared specifications:

```bash
specify integration switch codex
specify integration switch claude
specify integration switch grok
specify integration switch agy
```

Use only the integration needed on that checkout and review generated agent
files before committing. Shared project memory belongs in `.specify/`, `docs/`,
and `specs/`, not in model-specific chat notes.

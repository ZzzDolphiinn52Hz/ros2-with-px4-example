# Feature Specification: Multi-Agent Project Memory

**Feature Branch**: `001-multi-agent-project-memory`  
**Created**: 2026-11-22  
**Status**: Implemented

## Goal

Allow a developer or AI coding agent on a new machine to discover current
intent, architecture, progress, safety constraints, and the next action without
first reading the entire ROS 2 source tree.

This feature reduces context discovery; it does not permit an agent to modify
code without reading the relevant implementation and tests.

## User Scenarios and Acceptance

### US1 — Continue Work on Another Agent (P1)

Given a fresh clone, when an agent reads the documented entry points, then it
can identify the active feature, incomplete task, affected package, safety
constraints, and verification command before opening source files.

### US2 — Use Different AI Integrations (P1)

Given a supported coding agent, when a developer selects its Spec Kit
integration locally, then shared specs remain agent-neutral and no chat history
is required for handoff.

### US3 — Obtain Compact Checkout Context (P2)

Given a local checkout, when `./uavcup context` runs, then it reports branch,
commit, dirty state, submodule revision, ROS packages, active project status,
and available feature specs without requiring Docker or Raspberry Pi hardware.

## Functional Requirements

- **FR-001**: The repository MUST contain one obvious agent entry point.
- **FR-002**: Safety and quality rules MUST be versioned in the constitution.
- **FR-003**: Current architecture and core ROS contracts MUST be indexed in
  maintained documents.
- **FR-004**: Current progress MUST identify branch, feature, evidence location,
  next action, blockers, and unrun hardware checks.
- **FR-005**: Feature tasks MUST include exact paths and verification commands.
- **FR-006**: Spec Kit MUST use shared scaffolding by default; agent-specific
  integrations are selected per checkout.
- **FR-007**: The context command MUST be read-only and work without ROS build,
  Docker, camera, model, or UART.
- **FR-008**: Existing build/runtime behavior MUST remain unchanged.
- **FR-009**: Raspberry Pi handoff MUST define an SSH preflight, clean-tree
  check, push to the shared remote, fast-forward-only pull, submodule update,
  and commit/revision comparison without destructive conflict handling.

## Out of Scope

- Automatically summarizing arbitrary source changes.
- Replacing source inspection or tests with documentation.
- Installing every supported AI agent on every machine.
- Flight or hardware validation of the UAV stack.

## Success Criteria

- A new agent can follow a documented read order to one active task.
- `specify check` recognizes the initialized project and shared integration.
- `./uavcup context` exits successfully in the current checkout.
- `bash -n uavcup` passes.
- Raspberry Pi synchronization has one documented, auditable Git-based flow;
  unavailable SSH credentials are reported as `NOT RUN`, never as synchronized.
- Existing software-only ROS tests have no regressions, or any inability to run
  them is recorded explicitly with the reason.

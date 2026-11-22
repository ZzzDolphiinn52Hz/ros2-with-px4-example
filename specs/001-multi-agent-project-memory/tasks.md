# Tasks: Multi-Agent Project Memory

## Setup

- [x] **T001** Create branch `001-multi-agent-project-memory` from clean `main`.
- [x] **T002** Install Specify CLI and initialize shared scaffolding with
  `specify init --here --force --integration none`.

## Shared Memory

- [x] **T003** Replace the placeholder constitution at
  `.specify/memory/constitution.md` with UAV safety, contract, evidence, and
  handoff rules.
- [x] **T004** Add `AGENTS.md` and `docs/AI_WORKFLOW.md` with read order,
  multi-machine bootstrap, integration switching, and handoff protocol.
- [x] **T005** Add `docs/ARCHITECTURE.md` and `docs/ROS2_CONTRACTS.md` from the
  current package READMEs, configuration, launch profiles, and interfaces.
- [x] **T006** Add `docs/PROJECT_STATUS.md`, `specs/README.md`, and this retained
  feature artifact set.

## Tooling and Discovery

- [x] **T007** Add a read-only `context` command to `uavcup` that requires only
  Bash and Git.
- [x] **T008** Link the agent/spec workflow from root `README.md`.

## Verification

- [x] **T009** Run `bash -n uavcup` and `./uavcup context`; record results in
  `verification.md`.
- [x] **T010** Run Spec Kit health/integration checks and documentation checks;
  record results in `verification.md`.
- [x] **T011** Run `./uavcup unit-test` if the ROS build is available; otherwise
  record `NOT RUN` and the exact blocker.
- [x] **T012** Review `git diff`, update `docs/PROJECT_STATUS.md`, `spec.md`, and
  `verification.md` to their final state.
- [x] **T013** Add `docs/RASPI_SYNC.md` and make SSH preflight, shared-remote
  push, Pi fast-forward pull, submodule update, and SHA comparison mandatory in
  the constitution, agent entry point, workflow, status, and feature artifacts.

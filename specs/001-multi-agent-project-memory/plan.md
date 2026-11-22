# Implementation Plan: Multi-Agent Project Memory

**Branch**: `001-multi-agent-project-memory`  
**Date**: 2026-11-22  
**Spec**: `specs/001-multi-agent-project-memory/spec.md`

## Summary

Initialize Spec Kit in agent-neutral mode, add durable cross-agent project
memory, and expose a hardware-independent context command through the existing
operator CLI. No ROS nodes, launch behavior, parameters, or PX4 interfaces are
changed.

## Technical Context

- Specify CLI 1.4.3, shell integration, shared `none` agent target.
- ROS 2 Humble workspace with four project `ament_python` packages and one PX4
  message submodule.
- Documentation is Markdown tracked by Git.
- CLI change is Bash and must remain compatible with `set -euo pipefail`.

## Constitution Check

- Flight behavior: no runtime node or `/fmu/in/*` change — PASS.
- ROS contracts: indexed from existing package READMEs/config/source — PASS.
- Source ownership: no component config duplicated — PASS.
- Evidence: shell, Spec Kit, docs, and existing test commands planned — PASS.
- Handoff: active feature and next action are explicit — PASS.

## Files

```text
.specify/                         Spec Kit shared infrastructure
.specify/memory/constitution.md  workspace rules
AGENTS.md                         universal agent entry point
docs/AI_WORKFLOW.md               machine/agent onboarding and handoff
docs/ARCHITECTURE.md              package/runtime index
docs/ROS2_CONTRACTS.md            cross-package interface index
docs/RASPI_SYNC.md                SSH and Git synchronization gate
docs/PROJECT_STATUS.md            current handoff pointer
specs/001-.../                    intent, plan, tasks, evidence
uavcup                            read-only context command
README.md                         human-facing discovery link
```

## Verification Matrix

| Layer | Command | Expected |
|---|---|---|
| Bash syntax | `bash -n uavcup` | exit 0 |
| Context | `./uavcup context` | compact report, exit 0 |
| Spec Kit | `specify check` and integration status | initialized and healthy |
| Links/placeholders | repository checks recorded in verification | no broken local links or unresolved constitution placeholders |
| ROS regression | `./uavcup unit-test` | all project package tests pass |
| Hardware | `./uavcup doctor` on Pi | manual follow-up; not required for docs-only change |

## Risks and Mitigations

- Documentation drift: constitution requires contract updates with behavior.
- Conflicting agent files: repository defaults to `none`; select integrations
  per checkout and return to `none` before committing shared state.
- False confidence from specs: all entry points explicitly require relevant
  source/test inspection before edits.
- Concurrent state conflicts: separate clone/worktree and branch per agent.
- Raspberry Pi drift: require clean-tree checks, shared-remote push,
  fast-forward-only pull, submodule update, and matching revisions.

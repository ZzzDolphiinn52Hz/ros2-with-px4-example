# Multi-Agent Development Workflow

## Purpose

This workflow lets a new machine or AI agent continue a bounded change without
reconstructing the whole project from source. It deliberately does not promise
zero source reading: the next agent reads the relevant implementation and tests
identified by the active plan.

## Bootstrap a New Machine

```bash
git clone --recurse-submodules <repository-url> ~/ros2_ws
cd ~/ros2_ws
uv tool install specify-cli==1.4.3
specify check
./uavcup context
```

Install the integration for the local agent only:

```bash
specify integration switch codex   # Codex
specify integration switch claude  # Claude Code
specify integration switch grok    # Grok Build
specify integration switch agy     # Antigravity
```

To return the tracked project to shared scaffolding before committing
agent-specific generated files:

```bash
specify integration switch none
specify integration status
```

Do not use `--force` during switching unless generated-file changes have been
reviewed and replacement is intentional.

## Start or Continue Work

1. Run `./uavcup context` and confirm the branch/working tree.
2. Follow the read order in `AGENTS.md`.
3. Continue the first incomplete, unblocked item in the active `tasks.md`.
4. Inspect the exact source/config/test files named by that task.
5. Implement and run the listed verification.
6. Record the command and result in `verification.md`.
7. Update `docs/PROJECT_STATUS.md` before ending the session.

## Feature Artifact Policy

The project uses a hybrid living/flow-forward policy:

- The current behavior contract in a retained `spec.md` is maintained when the
  same feature evolves.
- Materially new behavior gets a new numbered feature directory that links to
  the earlier feature it supersedes or extends.
- `plan.md` and `tasks.md` describe the current delivery pass and may be revised
  while work is active.
- Completed verification evidence is retained. Do not rewrite a failed result;
  append the later successful run.
- Enduring cross-feature knowledge is promoted to `docs/ARCHITECTURE.md`,
  `docs/ROS2_CONTRACTS.md`, or the constitution.

## Handoff Contract

A handoff is usable only when it states:

- active feature and branch;
- last completed task;
- exact verification command and outcome;
- dirty files or commit SHA;
- blockers and assumptions;
- one concrete next action;
- hardware checks that remain `NOT RUN`.

Never mark work complete solely because code was generated. Completion means
spec acceptance criteria, source, tests, contracts, and recorded evidence agree.

## Parallel Agents

Use separate clones or Git worktrees and separate feature branches. Never run
multiple agents against one working tree. If several agents use Spec Kit in
parallel, each checkout must select its own feature; machine-local
`.specify/feature.json` is intentionally ignored by Git.

## Raspberry Pi Synchronization Gate

When work must run on the vehicle computer, follow `docs/RASPI_SYNC.md`. A
handoff is not synchronized merely because both machines contain similar files:
the branch must be pushed to the shared remote, pulled on the Pi with
fast-forward-only semantics, and verified by matching commit and submodule
revisions. Record the result in the active `verification.md` and
`docs/PROJECT_STATUS.md`.

# Feature Specifications

Each numbered directory is durable, agent-neutral project memory for one
bounded change. Read `docs/PROJECT_STATUS.md` to find the active feature.

Minimum retained artifacts:

```text
specs/NNN-short-name/
├── spec.md          # observable intent and acceptance criteria
├── plan.md          # affected architecture, contracts, and verification plan
├── tasks.md         # executable work with exact paths and commands
└── verification.md  # timestamped evidence; NOT RUN is explicit
```

Use a new directory for materially new behavior. If an existing contract is
being evolved, either update its living `spec.md` or create a new feature that
links to the earlier one; state the choice in the plan. Do not use a spec
directory as a private scratchpad or store secrets in it.

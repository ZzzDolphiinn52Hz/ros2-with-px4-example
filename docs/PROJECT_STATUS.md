# Project Handoff Status

**Updated**: 2026-11-23

**Active feature**: `specs/002-pi-gpio-power-validation/`

**Branch**: `main`
**State**: bench evidence recorded; hardware follow-up remains

## Last Completed

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

Read `specs/002-pi-gpio-power-validation/verification.md` for exact commands,
measurements and limits. The accepted Pi test produced 600 samples, three
verified PNGs, and a final ArUco status of `ok`. No ROS source or runtime
configuration changed in this evidence task.

## Working Tree

The GPIO power evidence is documented locally but the raw CSV/PNG artifacts
remain on the Raspberry Pi under its ignored `artifacts/` directory. The local
checkout contains only the durable specification and handoff updates.

## Next Action

Repeat the power test at the lowest expected LiPo voltage and with representative
servo/load transients while propellers remain removed. Add an external current
sensor or DC watt meter before claiming XL4015 current headroom. Then review
and commit the durable specification updates; no source synchronization is
needed for this documentation-only change.

## Known Constraints

- Agent-specific integration files are intentionally not committed. Each
  checkout selects `codex`, `claude`, `grok`, or `agy` locally.
- Camera/UART/flight checks require Raspberry Pi hardware and remain separate
  from software-only verification.
- The accepted power artifacts are on the Pi and are not committed because
  `artifacts/` is intentionally ignored. Their path and summary are recorded
  in the feature verification file.
- Chat history is not project state. Future agents must update this file and
  the active feature artifacts before handoff.

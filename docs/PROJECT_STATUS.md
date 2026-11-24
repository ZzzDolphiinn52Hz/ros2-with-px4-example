# Project Handoff Status

**Updated**: 2026-11-24

**Active feature**: `specs/003-aruco-80mm-target/`

**Branch**: `main`
**State**: 80 mm target implemented; print and camera validation pending

## Last Completed

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

Read `specs/003-aruco-80mm-target/verification.md`. The configuration test
passed, the PDF is A4, and the 300 DPI marker region measures approximately
80.01 x 80.01 mm after pixel rounding. Printing and camera validation are
still NOT RUN.

## Working Tree

The 80 mm target work started from commit `6a1b6cd`. Raspberry Pi deployment
has not run yet and must follow `docs/RASPI_SYNC.md` after commit/push.

## Next Action

Print the new PDF at Actual size / 100%, measure both outer black edges, then
deploy the matching commit to the Raspberry Pi with a fast-forward-only pull.
Repeat static pose tests to determine the reliable range of the smaller marker
before any flight test.

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

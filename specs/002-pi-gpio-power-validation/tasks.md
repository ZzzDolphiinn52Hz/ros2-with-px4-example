# Tasks: Raspberry Pi GPIO Power Validation

## Preparation

- [x] Confirm SSH access to `pi@192.168.1.120` and identify Raspberry Pi 5.
- [x] Confirm IMX500 is camera index 0 and no conflicting perception process
  is running.
- [x] Confirm the existing Pi checkout has local `docs/log.md` and an
  untracked screenshot; preserve both without staging or modifying them.

## Workload and Measurement

- [x] Start `./uavcup aruco-camera` for IMX500 640x480 at 15 FPS.
- [x] Start `./uavcup aruco-test` with ArUco debug profile.
- [x] Keep the existing XRCE Agent running without enabling control.
- [x] Run a corrected 600-sample sampler at one sample per second.
- [x] Save CSV, JSON summary and PNG plots under the Pi `artifacts/` path.
- [x] Save final ArUco status and verify the detector remains healthy.
- [x] Stop the test camera/container and leave only the pre-existing XRCE
  Agent running.

## Follow-up

- [ ] Repeat at the lowest expected LiPo voltage.
- [ ] Repeat with representative servo/load transients, propellers removed.
- [ ] Add an external current sensor or DC watt meter and extend the CSV with
  input current and power.
- [ ] Copy the accepted artifacts to the thesis evidence location manually
  or through a reviewed, non-source artifact workflow; do not commit ignored
  raw artifacts to Git.

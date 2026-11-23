# Verification: Raspberry Pi GPIO Power Validation

**Date**: 2026-11-23  
**Host**: Raspberry Pi 5 at `192.168.1.120`  
**Power path**: LiPo → XL4015 → GPIO 5 V/GND header  
**Accepted session**: `gpio_aruco_20261123_222253`

## Preflight

- SSH connection — PASS.
- Model — PASS: Raspberry Pi 5 Model B Rev 1.0.
- IMX500 discovery — PASS: camera index 0, 640x480 RGB stream opened.
- Initial power state — PASS: `throttled=0x0`.
- Existing XRCE Agent — PASS: remained running; no configuration change.
- Repository source — PASS: Pi was on commit `b40023c`; pre-existing local
  `docs/log.md` and screenshot were preserved and not changed.

## Accepted Test

Commands/profiles used:

```text
./uavcup aruco-camera
./uavcup aruco-test
```

The corrected sampler ran for 600 samples at a one-second interval from
`2026-11-23T22:22:53.461+07:00` to
`2026-11-23T22:32:52.460+07:00`. Workload:

```text
IMX500 640x480 15 FPS + ArUco detector debug profile + XRCE Agent
```

Result summary:

| Quantity | Result |
|---|---:|
| Samples | 600 |
| Duration | 599 s |
| EXT5V_V mean | 5.0002 V |
| EXT5V_V minimum | 4.8923 V |
| EXT5V_V maximum | 5.0384 V |
| EXT5V_V median | 5.0002 V |
| CPU temperature mean / max | 70.4 / 74.1 °C |
| PMIC temperature mean / max | 65.2 / 66.5 °C |
| CPU usage mean / max | 27.9 / 30.1 % |
| CPU frequency | 2400 MHz |
| New undervoltage events | 0 |
| Samples with nonzero throttled flag | 0 |
| Current measured | NO |

Only 1/600 samples was below 4.90 V and 4/600 samples were below 4.95 V.
These are PMIC `EXT5V_V` readings, not a direct measurement at the Pi header
or at the XL4015 output connector.

Final ArUco status — PASS:

```text
message: ok
marker_count: 0
target_visible: false
latency_ms: 146.51
```

`marker_count=0` is expected because no marker was placed for this power test;
the detector and camera pipeline remained healthy.

## Artifact Integrity

The CSV had 601 lines (header plus 600 samples). All three PNGs opened and
verified as 1600x900 RGB images. Accepted artifact directory:

```text
/home/pi/ros2_ws/artifacts/power_test/gpio_aruco_20261123_222253/
```

The first run was invalid and retained separately as:

```text
gpio_aruco_20261123_221623_INVALID_voltage_parser/
```

It is excluded because its voltage parser read the channel-name digit rather
than the measured value.

## Interpretation and Limits

- GPIO power passed this 10-minute static ArUco bench test without a new
  undervoltage event or throttling.
- The 4.8923 V minimum is a transient margin observation, not a guarantee for
  motor/servo transients or low battery voltage.
- Temperature reached 74.1 °C; cooling and airflow remain relevant for flight.
- Input current and power were not measured, so XL4015 current headroom is not
  yet quantified.
- Flight readiness — NOT RUN.
- Low-LiPo and dynamic-load validation — NOT RUN.

## Re-run Instructions

The operator runbook is maintained at `docs/PI_GPIO_POWER_TEST.md`. It
contains the exact SSH, camera, ArUco, CSV, status-check and safe-stop
commands. The accepted 2026-11-23 session used a more complete sampler that
also recorded PMIC temperature, CPU usage, CPU frequency and memory, then
generated the three PNG plots. The documented manual sampler is intentionally
smaller and does not claim to measure input current.

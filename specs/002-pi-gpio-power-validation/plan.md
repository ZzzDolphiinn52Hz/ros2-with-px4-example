# Implementation Plan: Raspberry Pi GPIO Power Validation

**Branch**: `main`  
**Date**: 2026-11-23  
**Spec**: `specs/002-pi-gpio-power-validation/spec.md`

## Approach

This is a hardware-evidence and documentation feature; no ROS source or power
configuration is changed. The workload is started using the existing operator
profiles, while a host-side Python sampler on the Pi writes the CSV and plots
to the ignored `artifacts/` directory. Pillow is used for PNG generation
because Matplotlib was not installed on the Pi and no package installation was
needed.

The first sampling attempt is retained only as a diagnostic record with the
suffix `INVALID_voltage_parser`; its voltage parser captured the `5` in the
channel name `EXT5V_V` rather than the value after `=`. It must not be used in
the thesis. The second run corrected the parser and is the accepted dataset.

## Evidence Location

Accepted data on the Raspberry Pi:

```text
/home/pi/ros2_ws/artifacts/power_test/gpio_aruco_20261123_222253/
```

Files:

```text
power_log.csv
summary.json
voltage_over_time.png
temperature_over_time.png
cpu_usage_over_time.png
aruco_status_final.yaml
```

## Safety Boundaries

- `aruco-test` is perception-only and does not create PX4 `/fmu/in/*`
  publishers.
- The XRCE Agent was not modified; no Offboard request or adapter enable was
  performed.
- Camera and detector were stopped after the test. The XRCE Agent was left
  running.
- This evidence is a propeller-off bench result, not flight approval.

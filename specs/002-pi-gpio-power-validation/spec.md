# Feature Specification: Raspberry Pi GPIO Power Validation

**Feature Branch**: `main`  
**Created**: 2026-11-23  
**Status**: Bench test completed; hardware follow-up remains

## Goal

Record reproducible evidence that the Raspberry Pi 5 can be powered through
the 5 V GPIO header from the LiPo-to-XL4015 path while running the intended
IMX500 ArUco workload. Record voltage, temperature, throttling and CPU load so
the results can be used in the thesis and compared with a later current
measurement.

## Scope

- Raspberry Pi 5, powered through GPIO 5 V/GND header.
- LiPo-to-XL4015 power path.
- IMX500 host camera server at 640x480, 15 FPS.
- ArUco detector debug profile in the ROS Docker container.
- Existing Micro XRCE-DDS Agent left running.
- No landing PID, PX4 bridge enable, Offboard request, arming, or flight.
- Input current was not measured in this session.

## Acceptance Criteria

- A one-second CSV log is produced for the complete test interval.
- The log contains timestamp, elapsed time, EXT5V voltage, CPU/PMIC
  temperature, throttled state, CPU usage, load, CPU frequency and memory use.
- A JSON summary and PNG plots are produced beside the CSV.
- The test records whether new kernel undervoltage events occurred.
- The result explicitly separates bench evidence from flight readiness.

## Out of Scope

- Measuring XL4015 input/output current or power.
- Proving operation during motor/servo transients or at minimum LiPo voltage.
- Changing XL4015 voltage/current settings.
- Automatic startup, arming, Offboard mode or landing behavior.

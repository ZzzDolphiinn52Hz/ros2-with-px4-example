# Tasks: Gazebo ArUco Landing End-to-End

- [x] Add the free-flight world and validate its geometry.
- [x] Add the full simulation landing launch with disabled defaults.
- [x] Separate PID enable from landing start and add regression tests.
- [x] Add explicit operator commands and read-only readiness/status checks.
- [x] Add rosbag recording for landing analysis.
- [x] Document QGC/manual approach and ROS takeover step by step.
- [x] Build and run automated tests.
- [x] Verify safe startup publishes no setpoint messages before adapter enable.
- [ ] Run and record an end-to-end SITL landing.
- [x] Update contracts, architecture and project handoff state.
- [x] Measure raw Gazebo, bridge callback and ROS camera timing with the
  detector-only setup and compare before/after.
- [x] Check capture age in readiness and PID and regression-test an
  increasing backlog, pause, reset and duplicate poses.

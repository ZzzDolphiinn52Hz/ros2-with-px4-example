# PX4 UAV Cup Simulation

This optional PC-only package replaces the real vehicle, IMX500 and UART with
Gazebo Sim, a simulated down camera and PX4 SITL. The production ArUco detector
is reused in `ros_topic` mode. It is not part of the Raspberry Pi default build.

The camera path uses a small one-way `gz.transport13` adapter instead of the
Ubuntu 22.04 `ros_gz_bridge` package. The latter targets an older Gazebo
transport ABI than the Gazebo Harmonic installation used by PX4 v1.17 on this
machine. The adapter publishes standard ROS 2 `sensor_msgs/Image` and
`sensor_msgs/CameraInfo` with sensor-data QoS.

The phase-2 `aruco_detector.sdf` world places a disarmed X500 on an invisible
collision stand above a `DICT_5X5_50`, ID 0, 80 mm marker. The stand prevents
the vehicle from falling without introducing an automatic arm path.

Run the commands documented in `docs/GAZEBO_ARUCO_SITL.md` from separate
terminals. These profiles do not enable the PID, enable the PX4 adapter, arm,
or request Offboard.

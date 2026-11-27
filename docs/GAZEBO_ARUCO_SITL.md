# Gazebo ArUco + PX4 SITL (Phase 1 and Phase 2)

This procedure runs the existing ArUco detector against a Gazebo Harmonic
camera and connects PX4 v1.17 SITL to ROS 2 through Micro XRCE-DDS UDP. It does
not launch the landing PID or PX4 command adapter, and it never arms or requests
Offboard.

## Scope

- Host: Ubuntu 22.04, ROS 2 Humble, Gazebo Sim 8 (Harmonic).
- Vehicle: PX4 X500 with a centred, downward camera.
- Camera: 640x480, 15 Hz, image-top aligned with the vehicle nose.
- Marker: `DICT_5X5_50`, ID 0, 80 mm black outer square on a 120 mm white top.
- Transport: PX4 SITL to Micro XRCE-DDS Agent over UDP port 8888.

The detector world supports the disarmed vehicle on an invisible collision
stand. The stand has no visual geometry, so it does not occlude the marker.
It is only for phase-2 perception checks and is not a free-flight world.

## One-time setup

The default PX4 path is:

```text
~/Documents/px4-myself/firmware/PX4-Autopilot
```

Override it when necessary:

```bash
export PX4_AUTOPILOT_DIR=/absolute/path/to/PX4-Autopilot
```

Check prerequisites and build the optional host-side package:

```bash
cd ~/ros2_ws
./uavcup sim-doctor
./uavcup sim-setup
```

`sim-setup` sets `PYTHONNOUSERSITE=1` only for the host ROS process. This keeps
the user's NumPy 2 installation untouched while allowing Ubuntu OpenCV 4.5,
which was compiled against NumPy 1, to load correctly.

## Run in four terminals

### Terminal 1: Gazebo world

```bash
cd ~/ros2_ws
./uavcup sim-gazebo
```

Set `UAV_SIM_HEADLESS=1` to run without the GUI:

```bash
UAV_SIM_HEADLESS=1 ./uavcup sim-gazebo
```

### Terminal 2: Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws
./uavcup sim-agent
```

Expected first line includes `port: 8888`.

### Terminal 3: PX4 SITL

Start this only after Gazebo is ready:

```bash
cd ~/ros2_ws
./uavcup sim-px4
```

Expected evidence includes:

```text
PX4 will attach to existing model
uxrce_dds_client ... UDP agent IP:127.0.0.1, port:8888
uxrce_dds_client ... synchronized with time offset
```

Warnings about no GCS connection are expected when QGroundControl is closed.
Do not arm for phase 1 or phase 2.

### Terminal 4: camera bridge and ArUco detector

```bash
cd ~/ros2_ws
./uavcup sim-aruco
```

This launch contains only the camera adapter and detector. It creates no
publisher to `/fmu/in/*`.

## Collect the combined evidence

In a fifth terminal:

```bash
cd ~/ros2_ws
./uavcup sim-status
```

Expected results:

- `/fmu/out/vehicle_status_v1` type is `px4_msgs/msg/VehicleStatus`.
- `/fmu/out/vehicle_local_position_v1` type is
  `px4_msgs/msg/VehicleLocalPosition`.
- Simulated camera frames are present.
- `/uav/aruco/status` reports `target_visible=true` and marker ID 0.
- `/uav/aruco/ids` contains `0`.
- `/uav/aruco/target_pose.pose.position.z` is approximately 0.90 m in the
  detector-stand world.

Stop each foreground process with `Ctrl+C` in reverse order.

## Camera-model limitation

Gazebo Sim 8 with Ogre2 does not implement the Brown lens-distortion render
pass. The simulation therefore uses the measured IMX500 intrinsic matrix but
publishes an ideal pinhole image with `D=[0,0,0,0,0]`. The real Raspberry Pi
profile retains the measured distortion coefficients. Simulated pose results
validate integration and control logic, not real-lens calibration accuracy.

## PX4 topic-version note

PX4 v1.17 publishes local position on:

```text
/fmu/out/vehicle_local_position_v1
```

Both the landing controller and PX4 adapter use that exact topic. The attitude
topic remains `/fmu/out/vehicle_attitude`, and vehicle status is
`/fmu/out/vehicle_status_v1`.

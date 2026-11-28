# Workspace Architecture

## System Boundary

The product is a Raspberry Pi 5 perception, local-control, and PX4 Offboard
adapter stack. Raspberry Pi OS owns the CSI cameras; ROS 2 Humble runs in the
Docker `ros` service. Micro XRCE-DDS carries PX4 v1.17 messages over TELEM2/UART.

```text
IMX219 front ─┐
              ├─ Picamera2 host servers ─ Unix sockets ─ perception
IMX500 down ──┘                                      │
                                                    ├─ relative obstacle data
                                                    └─ ArUco target pose
                                                              │
                                              control (disabled by default)
                                                              │ body-FLU Twist
                                              PX4 bridge (disabled by default)
                                                              │ explicit gate
                                                     /fmu/in/* via XRCE-DDS
```

## Package Map

| Package | Responsibility | Configuration | Primary tests |
|---|---|---|---|
| `px4_uavcup_perception` | Picamera2 socket input, ArUco pose, relative ZipDepth/corridor | `config/aruco.yaml`, `config/zipdepth.yaml`, `config/pi_cameras.yaml` | `test/` geometry, socket, camera, depth |
| `px4_uavcup_control` | Obstacle advisories/gate and ArUco landing state machine | `config/landing.yaml`, corridor/shadow YAML | `test/` controller and landing behavior |
| `px4_uavcup_px4_bridge` | Bounded body-FLU velocity to PX4 NED setpoints and explicit mode services | `config/px4_bridge.yaml` | `test/test_cmd_vel_to_px4.py` |
| `px4_uavcup_bringup` | Compose existing nodes into test/bench/vehicle launch profiles | no duplicated component config | launch composition |
| `px4_msgs` | PX4 v1.17 ROS interfaces | Git submodule | upstream package checks |
| `px4_uavcup_simulation` | Optional PC-only Gazebo Harmonic/PX4 SITL assets and camera bridge | `config/gz_bridge.yaml`, `models/`, `worlds/` | simulation asset/bridge tests |

The root `uavcup` command is the supported operator interface. Direct launch
commands are useful for development but must not create a second production
workflow.

## Runtime Flows

### ArUco landing

```text
down_camera.sock
  -> aruco_detector_node
  -> /uav/aruco/target_pose
  -> aruco_landing_pid (starts disabled)
  -> /aruco_land/cmd_vel
  -> cmd_vel_to_px4 (starts disabled)
  -> PX4 OffboardControlMode + TrajectorySetpoint
```

The landing controller never creates its own PX4 publisher path. Starting
landing does not request Offboard. The adapter requires explicit enable and a
separate explicit Offboard request.

### Relative obstacle guidance

```text
front_camera.sock
  -> zipdepth_node
  -> /uav/depth/relative_free_space or /uav/depth/relative_corridor
  -> shadow controller
  -> advisory velocity
  -> optional velocity gate
```

Relative outputs are normalized scene cues, not metres. The shadow controller
does not command PX4 directly.

## Launch Profiles

| Profile | Scope | PX4 output path |
|---|---|---|
| `aruco_test.launch.py` | detector/debug | none |
| `zipdepth_test.launch.py` | ZipDepth only | none |
| `pi_perception_test.launch.py` | both perception pipelines | none |
| `pi_aruco_vehicle.launch.py` | ArUco + landing + bridge | present, controllers disabled |
| `pi_corridor_px4_bench.launch.py` | corridor + safety gates | present, disabled |
| `pi_vehicle.launch.py` | complete vehicle stack | present, disabled |
| `sim_aruco_detector.launch.py` | simulated detector only | none |
| `sim_aruco_landing.launch.py` | simulated camera + production detector/PID/bridge | present, PID and bridge disabled |

## Environment and Artifacts

- `Dockerfile` and `docker-compose.yml` define ROS runtime and XRCE Agent.
- `build/`, `install/`, and `log/` are generated and ignored.
- ZipDepth model files live outside Git, normally under `~/models`.
- Camera calibration evidence lives under ignored `artifacts/`; accepted
  calibration values are copied into the owning YAML and documented.
- Thesis directories are references only and are excluded from runtime scope.

## Where to Read for a Change

| Change type | Start here |
|---|---|
| Camera or ArUco | perception README, `config/aruco.yaml`, detector tests |
| ZipDepth/corridor | `config/zipdepth.yaml`, depth modules/tests |
| Landing behavior | control README, `config/landing.yaml`, landing tests |
| PX4 output/modes | bridge README/config, bridge tests, then implementation |
| Launch composition | bringup README and the named launch profile |
| Operator workflow | root README and `uavcup` |

Update this file when package ownership or runtime data flow changes.

## Optional PC Simulation

Gazebo Harmonic and PX4 SITL run on the development host. The optional
`px4_uavcup_simulation` package publishes the simulated down camera through the
same ROS image/CameraInfo contract used by the detector. Micro XRCE-DDS uses
UDP port 8888 instead of the Raspberry Pi UART. This package is intentionally
excluded from the Raspberry Pi default setup command.

The free-flight `aruco_landing` world mirrors the real operator boundary: QGC
owns manual arm, takeoff and target approach. ROS takeover remains four
separate gates: adapter enable, Offboard request, PID enable and landing start.

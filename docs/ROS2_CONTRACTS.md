# ROS 2 Contracts

This file is the cross-package interface index. The owning YAML and source are
authoritative for exact defaults; any interface change must update both this
index and its tests in the same feature.

## Frames and Units

| Name | Convention |
|---|---|
| Camera optical | `x` right, `y` down, `z` forward |
| Body FLU | `x` forward, `y` left, `z` up |
| PX4 local NED | `x` north, `y` east, `z` down |
| `down_camera_optical_frame` | IMX500; image top points toward vehicle nose |
| `front_camera_optical_frame` | front IMX219 optical frame |
| `base_link` | body-FLU command/advisory frame |

Distances and velocities with `_m` or `_m_s` suffixes are metric. Values on
`relative_*` topics are normalized/affine-relative and MUST NOT be interpreted
as metres.

## Perception Outputs

| Topic | Type | Contract |
|---|---|---|
| `/uav/aruco/target_pose` | `geometry_msgs/PoseStamped` | Target marker pose in the configured camera optical frame, metric translation |
| `/uav/aruco/status` | `diagnostic_msgs/DiagnosticArray` | Visibility, count, latency, and detector/input health |
| `/uav/aruco/ids` | `std_msgs/Int32MultiArray` | Debug only; disabled in flight profile |
| `/uav/aruco/rvecs` | `std_msgs/Float64MultiArray` | Debug only |
| `/uav/aruco/tvecs` | `std_msgs/Float64MultiArray` | Debug only; metres |
| `/uav/aruco/debug_image` | `sensor_msgs/Image` | Debug only; sensor-data QoS |
| `/uav/depth/relative_free_space` | `std_msgs/Float32MultiArray` | `[left, center, right, nearest, valid]`, relative values |
| `/uav/depth/relative_corridor` | `std_msgs/Float32MultiArray` | Relative opening/corridor summary; not metric |
| `/uav/depth/status` | `diagnostic_msgs/DiagnosticArray` | Backend, FPS, latency, input and calibration state |
| `/uav/depth/free_space` | `std_msgs/Float32MultiArray` | Metric mode only; must not be trusted when calibration is disabled |
| `/camera/depth/image` | `sensor_msgs/Image` | Optional metric output; blocked unless metric calibration is enabled |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Optional metric point cloud; blocked unless calibration is enabled |

Camera image topics use sensor-data/best-effort semantics. Control and status
topics currently use bounded in-process queue depths defined by their nodes.

The optional Gazebo profile publishes `/camera/down/image_raw` and
`/camera/down/camera_info` with the same frame and sensor-data QoS. It uses the
measured IMX500 intrinsic matrix but an ideal pinhole model (`D=0`) because the
Gazebo Harmonic Ogre2 renderer does not implement Brown lens distortion. This
does not change the real-camera calibration contract.

## Control Contracts

| Interface | Type | Contract |
|---|---|---|
| `/aruco_land/cmd_vel` | `geometry_msgs/Twist` | Bounded body-FLU velocity; landing controller starts disabled |
| `/uav/aruco/landing_state` | `std_msgs/String` | Landing state-machine state |
| `/uav/aruco/landing_status` | `diagnostic_msgs/DiagnosticArray` | State, abort reason, pose/landing health |
| `/aruco_landing_pid/enable` | `std_srvs/SetBool` | Explicitly prepares/disables controller; enabling remains `IDLE` and publishes no command |
| `/aruco_landing_pid/start_landing` | `std_srvs/Trigger` | Requires enabled controller, then starts `SEARCH`; does not request Offboard |
| `/aruco_landing_pid/abort` | `std_srvs/Trigger` | Aborts landing and returns to safe behavior |
| `/uav/local_controller/advisory_velocity` | `geometry_msgs/TwistStamped` | Shadow-only body-FLU advisory |
| `/uav/local_controller/cmd_vel` | `geometry_msgs/Twist` | Output of the optional bounded velocity gate |

Loss of the ArUco marker during visual approach produces zero motion commands.
Blind descent is allowed only after the bridge confirms latched XY/yaw and the
configured alignment/confirmation gates have passed.

## PX4 Bridge Contract

Inputs:

- `/aruco_land/cmd_vel` (`geometry_msgs/Twist` by current configuration).
- `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_attitude`,
  `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_land_detected`, and
  `/fmu/out/vehicle_command_ack` using PX4 v1.17 messages.

Outputs:

- `/fmu/in/offboard_control_mode` (`px4_msgs/OffboardControlMode`).
- `/fmu/in/trajectory_setpoint` (`px4_msgs/TrajectorySetpoint`).
- `/fmu/in/vehicle_command` (`px4_msgs/VehicleCommand`).

PX4 subscriptions and publishers use best-effort, transient-local, keep-last
depth 1 QoS to match the current bridge implementation.

Services:

| Service | Type | Safety contract |
|---|---|---|
| `/cmd_vel_to_px4/enable` | `std_srvs/SetBool` | Reject enable without valid local position and attitude |
| `/cmd_vel_to_px4/request_offboard` | `std_srvs/Trigger` | Separate explicit action after adapter heartbeat/setpoints |
| `/cmd_vel_to_px4/request_posctl` | `std_srvs/Trigger` | Explicit return request |
| `/cmd_vel_to_px4/latch_xy_yaw` | `std_srvs/Trigger` | Hold current local XY/yaw for blind descent |
| `/cmd_vel_to_px4/release_xy_yaw` | `std_srvs/Trigger` | Release latched hold |
| `/cmd_vel_to_px4/request_disarm` | `std_srvs/Trigger` | Allowed only when `VehicleLandDetected.landed` is true |

The bridge starts disabled, never arms automatically, never changes flight mode
automatically, clamps velocity/acceleration, and brakes or holds on stale
`cmd_vel` according to the current mode.

## Configuration Ownership

- Perception: `src/px4_uavcup_perception/config/`.
- Control: `src/px4_uavcup_control/config/`.
- PX4 bridge: `src/px4_uavcup_px4_bridge/config/px4_bridge.yaml`.
- Bringup may pass launch arguments but must not copy parameter sets.

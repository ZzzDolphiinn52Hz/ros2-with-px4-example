# Autonomous Urban Drone Delivery — ROS 2 workspace

Workspace ROS 2 Humble cho drone PX4 v1.17 SITL + Gazebo Harmonic, hướng tới
bay tự động trên sa bàn UAV Cup.

Pipeline localization và Navigation2 trên saved map đã chạy end-to-end trong
SITL. Bước tiếp theo trên branch `feat/waypoint-mission` là waypoint mission.

```text
Gazebo 2D LiDAR + PX4 local state
  -> /scan + /clock + /odom + TF
  -> AMCL trên saved map
  -> Nav2 planner / costmaps / controller / velocity smoother
  -> /cmd_vel
  -> PX4 Offboard adapter
  -> x500 giữ cao độ 0.7 m và bay tới Nav2 Goal
```

## Các package

| Package | Vai trò |
| --- | --- |
| `px4_uavcup_slam` | Bridge LiDAR Gazebo Harmonic, odom/TF, SLAM Toolbox, adapter `/cmd_vel` sang PX4 |
| `px4_uavcup_nav` | AMCL + Navigation2 trên saved map |
| `px4_state_reader` | Đọc và quan sát state PX4 qua uXRCE-DDS |
| `px4_offboard_baseline` | Các bài Offboard sớm: hold, takeoff/hover, tiến rồi dừng |
| `px4_msgs` | Message interface khớp PX4 v1.17 |

Nhật ký kỹ thuật, checkpoint và xử lý lỗi nằm tại
[`src/px4_uavcup_slam/README.md`](src/px4_uavcup_slam/README.md).
Context đầy đủ cho AI/engineer tiếp tục project nằm tại
[`AI_PROJECT_HANDOFF.md`](AI_PROJECT_HANDOFF.md).

## Tiến độ

Đã hoàn thành trong SITL:

- custom world `urban_uavcup` và LiDAR 2D;
- ROS 2 Offboard baseline;
- SLAM Toolbox và map đã lưu tại `maps/uavcup_map.{pgm,yaml}`;
- AMCL trên saved map, cây TF `map -> odom -> base_footprint -> base_link -> link`;
- Nav2 Goal: cùng heading, đổi yaw, tránh vật cản tĩnh;
- drone giữ cao độ 0.7 m, `/cmd_vel` về 0 và hover khi tới goal.

Chưa làm: waypoint mission bằng code, dynamic obstacle, failsafe mất scan,
takeoff/landing tự động, nhận/thả kiện, thử trên drone thật.

## Cây TF

```text
map -> odom -> base_footprint -> base_link -> link
```

- Mapping: chỉ `slam_toolbox` publish `map -> odom`.
- Navigation trên saved map: chỉ AMCL publish `map -> odom`.
- Robot-only: không có frame `map`.
- Không chạy SLAM Toolbox và AMCL cùng lúc.
- `base_footprint` là pose phẳng `(x, y, yaw)` cho SLAM/Nav2.
- `base_link` giữ `z`, roll và pitch thật của drone.
- Laser static: `base_link -> link` = `[0.12, 0.0, 0.26]`.

Bridge lọc scan khi cao độ `< 0.5 m` hoặc tilt `> 5°`. AMCL/SLAM cần drone
hover khoảng 0.7 m mới nhận scan hợp lệ.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select px4_uavcup_slam px4_uavcup_nav
source install/setup.bash
```

Không dùng path cũ `/home/dolphiinn/Documents/ros2_ws/install/setup.bash`.

Unit test phía robot:

```bash
python3 -m pytest -q src/px4_uavcup_slam/test
```

## Chạy navigation (đã kiểm chứng)

Bốn terminal, theo thứ tự:

```bash
# 1. PX4 SITL + Gazebo
make px4_sitl gz_x500_lidar_2d_urban_uavcup

# 2. Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 3. Robot-side: /clock, /scan, /odom, TF, adapter disabled
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_slam uavcup_robot.launch.py

# 4. AMCL + Nav2 + RViz trên saved map
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_nav uavcup_navigation.launch.py
```

Map mặc định của launch là `~/ros2_ws/maps/uavcup_map.yaml`. Trước Initial
Pose, lỗi thiếu `map -> base_footprint` là bình thường.

Enable adapter, arm trong QGroundControl, rồi vào Offboard:

```bash
ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: true}"

ros2 service call /cmd_vel_to_px4/request_offboard \
  std_srvs/srv/Trigger "{}"
```

Khi drone hover khoảng 0.7 m, dùng **2D Pose Estimate** trong RViz cho đến khi
laser trùng vật cản trên map, rồi gửi **Nav2 Goal**.

Dừng an toàn: Cancel Nav2 → chuyển PX4 sang Position/Loiter → mới disable
adapter. Không disable khi PX4 còn Offboard vì mất heartbeat có thể kích hoạt
failsafe `COM_OF_LOSS_T`.

Adapter mặc định disabled, không tự arm và không tự đổi mode.

## Mapping

Không chạy cùng lúc với navigation launch.

```bash
ros2 launch px4_uavcup_slam uavcup_slam.launch.py
```

Lưu map khi `/map` ổn định:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/uavcup_team1 \
  --ros-args -p use_sim_time:=true
```

Bản map đang dùng và bản backup nằm tại `maps/` và `maps_backup/`.

## Ghi chú học tập

Course ROS 2/Nav2: `/data/obsidian-vaults/Study/01-Courses/00-ROS2-humble/ros2_nav2/Lessons/`
(lesson 35, 40–44 đã dùng; waypoint tiếp theo bám 45–51).

Nhật ký project: `~/ObsidianVaults/Study/03-Projects/a-Drone-Sep1st/`.

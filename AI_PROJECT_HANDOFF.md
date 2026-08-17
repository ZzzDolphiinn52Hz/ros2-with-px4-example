# AI handoff — PX4 UAV Cup SLAM và Navigation2

> Cập nhật: 2026-08-14, Asia/Ho_Chi_Minh  
> Workspace: `/home/dolphiinn/ros2_ws`  
> Mục đích: cung cấp context khởi đầu đầy đủ cho một AI/engineer tiếp tục project mà không cần đọc lại toàn bộ session trước.

## 1. Trạng thái ngắn gọn

Project là drone giao hàng đô thị chạy PX4 v1.17 SITL, Gazebo Harmonic và ROS 2 Humble. Pipeline SLAM 2D và Navigation2 trên saved map đã chạy end-to-end thành công trong SITL:

```text
Gazebo 2D LiDAR + PX4 local state
  -> /scan + /clock + /odom + TF
  -> AMCL trên saved map
  -> Nav2 planner/costmaps/controller/velocity smoother
  -> /cmd_vel
  -> PX4 Offboard adapter
  -> x500 giữ cao độ 0.7 m và bay tới Nav2 Goal
```

Đã kiểm chứng thực tế trong session:

- `/scan` ổn định khoảng 30 Hz;
- TF đầy đủ `map -> odom -> base_footprint -> base_link -> link`;
- AMCL recorrect sai số odometry tốt, không nhảy sang cụm cột khác;
- nhiều Nav2 Goal cùng heading hoàn thành tốt;
- goal có thay đổi yaw hoạt động đúng, `angular.z` không vượt `±0.3 rad/s`;
- drone giữ cao độ, không nghiêng quá `5°` kéo dài;
- `/cmd_vel` về 0 và drone hover khi tới goal;
- khi có vật cản tĩnh giữa drone và goal, Nav2 tự lập path vòng tránh thành công.

Chưa thực hiện: waypoint mission bằng code, dynamic-obstacle test có kiểm soát, mất scan/failsafe behavior, hoặc thử nghiệm trên drone thật.

## 2. Trạng thái Git và tài liệu nguồn

Tại thời điểm tạo handoff:

```text
branch: feat/waypoint-mission
HEAD:   a536264 complete nav2
```

`main`, `origin/main` và `origin/feat/waypoint-mission` cũng đang trỏ tới commit này. Trước khi thêm file handoff, `git status --short` chỉ báo:

```text
?? src/px4_msgs/
```

Không xóa `src/px4_msgs/`: đây là message interface cần khớp PX4 v1.17, nhưng hiện toàn thư mục chưa được Git track. Hãy kiểm tra kỹ trước khi add/commit vì `.gitignore` có rule toàn cục `*.msg`, `*.srv`, `*.action`.

Root `README.md` đang lỗi thời: nó nói project dừng ở Phase 3-2 SLAM, trong khi Nav2 đã hoàn tất trong SITL. Tài liệu vận hành chi tiết và mới hơn nằm ở:

```text
src/px4_uavcup_slam/README.md
```

Course ROS 2/Nav2 mà user muốn bám theo nằm tại:

```text
/data/obsidian-vaults/Study/01-Courses/00-ROS2-humble/ros2_nav2/Lessons/
```

Các lesson đã dùng làm cơ sở là lesson 35 và 40–44. Bước waypoint tiếp theo nên bám lesson 45–51, đặc biệt:

- lesson 45: Simple Commander;
- lesson 46: initial pose và NavigateToPose interfaces;
- lesson 47: set initial pose bằng Python;
- lesson 48: gửi navigation goal;
- lesson 49: follow waypoints;
- lesson 50–51: patrol activity và solution.

## 3. Package và file quan trọng

### `px4_uavcup_slam`

Package này hiện chứa cả robot-side interfaces và mapping:

- `launch/uavcup_robot.launch.py`: robot-only bringup, không chạy SLAM/AMCL/RViz;
- `launch/uavcup_slam.launch.py`: include robot bringup rồi chạy SLAM Toolbox và RViz;
- `px4_uavcup_slam/gz_lidar_bridge.py`: bridge Gazebo Harmonic sang ROS `/scan` và `/clock`;
- `px4_uavcup_slam/px4_odom_tf.py`: PX4 NED/FRD sang ROS ENU/FLU, `/odom` và TF;
- `px4_uavcup_slam/cmd_vel_to_px4.py`: adapter `/cmd_vel` sang PX4 Offboard trajectory setpoint;
- `config/mapper_params_online_async.yaml`: SLAM Toolbox mapping config;
- `rviz/uavcup_slam.rviz`: RViz mapping;
- `test/test_frame_conversions.py` và `test/test_cmd_vel_to_px4.py`: unit tests;
- `README.md`: nhật ký kỹ thuật và các checkpoint đã làm.

### `px4_uavcup_nav`

Package mới cho localization và Navigation2:

- `launch/uavcup_navigation.launch.py`: static map + AMCL + Nav2 + RViz;
- `config/nav2_params.yaml`: toàn bộ AMCL, costmaps, planner, DWB controller, behaviors và velocity smoother;
- `rviz/uavcup_nav.rviz`: map, scan, particle cloud, costmaps, path và TF;
- `package.xml`, `setup.py`, `setup.cfg`: ament Python package.

Default map của launch hiện được tạo bằng `~/ros2_ws/maps/uavcup_map.yaml`; đây là một default path phụ thuộc home/workspace và nên được refactor nếu cần portability.

### Map

Map đang dùng:

```text
maps/uavcup_map.yaml
maps/uavcup_map.pgm
```

Thông số YAML:

```yaml
resolution: 0.05
origin: [-16.3, -27.2, 0]
mode: trinary
occupied_thresh: 0.65
free_thresh: 0.25
```

User đã sửa/clean map thủ công bằng GIMP theo lesson 35. Bản trước khi chỉnh và SLAM serialization được giữ trong `maps_backup/`; không ghi đè hoặc xóa nếu chưa được user yêu cầu.

## 4. Kiến trúc runtime và quyền sở hữu TF

Robot-only stack publish:

```text
/clock
/scan
/odom
odom -> base_footprint -> base_link -> link
```

Ba node mong đợi khi chỉ chạy robot bringup:

```text
/cmd_vel_to_px4
/gz_lidar_bridge
/px4_odom_tf
```

Quyền sở hữu `map -> odom`:

- mapping mode: chỉ `slam_toolbox` publish;
- saved-map navigation: chỉ AMCL publish;
- robot-only mode: không tồn tại `map` và `map -> odom`.

Không bao giờ chạy SLAM Toolbox và AMCL đồng thời vì hai node sẽ tranh quyền publish cùng transform.

`base_footprint` cố ý là planar `(x, y, yaw)` cho SLAM/Nav2. `base_link` là child chứa `z`, roll và pitch thật của drone. `link` là laser frame với static transform:

```text
base_link -> link: xyz = [0.12, 0.0, 0.26], quaternion identity
```

Thiết kế này ngăn SLAM 2D nhận trực tiếp z/roll/pitch của multicopter.

## 5. Các lỗi chính đã tìm ra và cách sửa

### Gazebo bridge không có dữ liệu

ROS Humble `ros_gz_bridge` từ apt dùng Fortress/ignition (`ignition-msgs8`, transport11), còn PX4 SITL hiện dùng Gazebo Harmonic (`gz-msgs10`, transport13). Bridge cũ có thể chạy nhưng báo unknown message type và `/scan` rỗng.

Giải pháp là node Python `gz_lidar_bridge.py` dùng trực tiếp:

```python
gz.transport13
gz.msgs10
```

Node này là nguồn `/clock`, vì vậy timer publish bắt buộc dùng system/wall clock. Nếu timer dùng simulated time, nó sẽ chờ chính `/clock` mà nó chưa publish và gây deadlock.

Gazebo LaserScan có thể chứa scoped frame name dài. Bridge luôn ghi đè `LaserScan.header.frame_id = link` để khớp TF; không dùng scoped Gazebo sensor path trong ROS TF.

### Map bị xé/nan và drift khi drone nghiêng

LiDAR 2D gắn cứng vào x500; scan không còn là cùng một lát cắt khi drone roll/pitch hoặc thay đổi cao độ. Bridge hiện loại scan khi:

```text
altitude < 0.5 m
tilt > 5 deg
```

Các parameter tương ứng:

```text
min_mapping_altitude_m = 0.5
max_tilt_deg = 5.0
```

Do đó khi drone đang nằm dưới đất, topic `/scan` có thể tồn tại nhưng không có message mới. AMCL/SLAM cần drone ở khoảng 0.7 m để nhận scan.

### PX4 quaternion/Yaw sai hướng

`VehicleAttitude.q` của PX4 là quaternion `[w,x,y,z]` cho phép quay body FRD sang earth NED. Thứ tự đúng:

```text
R_enu_flu = R_enu_ned @ R_ned_frd @ R_frd_flu
```

Thứ tự cũ làm yaw ROS lệch 180° và scan xé ngược hướng chuyển động. Logic đúng nằm trong `px4_odom_tf.py` và có unit test.

### PX4 EKF reset làm odom nhảy

`px4_odom_tf.py` theo dõi `xy_reset_counter` và `heading_reset_counter`, dùng `delta_xy`/`delta_heading` để cộng continuity offset. `cmd_vel_to_px4.py` theo dõi `z_reset_counter` và bù target Z. Không bỏ cơ chế này khi refactor.

### RViz báo không có `map`

Trong robot-only mode đây là trạng thái đúng. Khi Nav2/AMCL vừa start, map server và AMCL có thể active nhưng `map -> odom` chỉ xuất hiện sau khi AMCL nhận **2D Pose Estimate** và có scan hợp lệ. Dòng `Invalid frame ID map` đầu tiên của `tf2_echo` cũng có thể chỉ là discovery delay nếu ngay sau đó transform xuất hiện.

### Joystick/QGroundControl

Controller là Nintendo Switch Pro USB `057e:2009`. HID ID thay đổi mỗi lần cắm; một lần xác định được `0003:057E:2009.0006`. Cách bind generic driver đã tạo được `/dev/input/js0`:

```bash
controller_hid_id=$(
  basename "$(find /sys/bus/hid/devices -maxdepth 1 \
    -name '0003:057E:2009.*' -print -quit)"
)

echo 1 | sudo tee /sys/module/hid/parameters/ignore_special_drivers
printf '%s' "$controller_hid_id" | sudo tee /sys/bus/hid/drivers_probe
echo 0 | sudo tee /sys/module/hid/parameters/ignore_special_drivers
```

Tuy nhiên axes/buttons trong QGroundControl vẫn từng nhảy loạn và calibration không đáng tin. Đây chưa được giải quyết dứt điểm; Nav2 testing không phụ thuộc joystick. Dialog `MAV_CMD_START_RX_PAIR` là RX/radio pairing, không phải cách kết nối Linux joystick.

## 6. Offboard adapter và safety invariants

`cmd_vel_to_px4` nhận Twist trong ROS base FLU:

```text
linear.x = forward
linear.y = left
angular.z = CCW yaw rate
```

Node dùng PX4 attitude hiện tại để xoay vận tốc body sang earth NED, đổi dấu yaw rate sang convention NED, giữ Z bằng position setpoint và điều khiển XY bằng velocity setpoint.

Default:

| Parameter | Giá trị |
| --- | ---: |
| target altitude | 0.7 m |
| adapter max XY speed | 0.4 m/s |
| adapter max yaw rate | 0.3 rad/s |
| adapter XY acceleration | 0.3 m/s² |
| adapter yaw acceleration | 0.5 rad/s² |
| `/cmd_vel` timeout | 0.5 s |
| Offboard heartbeat/setpoint | 20 Hz |

Safety invariants:

- adapter mặc định disabled;
- node không tự arm;
- node không tự đổi mode khi enable;
- enable thất bại nếu PX4 local position hoặc attitude không hợp lệ;
- `/cmd_vel` timeout đưa XY/yaw về 0 nhưng tiếp tục giữ cao độ;
- service request Offboard từ chối khi PX4 failsafe hoặc preflight check fail;
- phải kiểm tra PX4 ACK và `nav_state=14`;
- không disable adapter khi PX4 vẫn ở Offboard, vì mất heartbeat có thể kích hoạt `COM_OF_LOSS_T` failsafe.

Services:

```text
/cmd_vel_to_px4/enable            std_srvs/srv/SetBool
/cmd_vel_to_px4/request_offboard  std_srvs/srv/Trigger
```

PX4 status tốt đã quan sát:

```text
arming_state: 2
nav_state: 14
failsafe: false
```

## 7. Nav2 configuration quan trọng

AMCL:

```text
base_frame_id: base_footprint
odom_frame_id: odom
global_frame_id: map
scan_topic: scan
robot_model_type: nav2_amcl::OmniMotionModel
laser range: 0.1–12.0 m
max_beams: 90
update_min_d/update_min_a: 0.1
```

Controller/DWB:

```text
controller_frequency: 10 Hz
vx: -0.3 .. 0.3 m/s
vy: -0.3 .. 0.3 m/s
vtheta: -0.3 .. 0.3 rad/s
XY accel/decel: ±0.3 m/s²
yaw accel/decel: ±0.5 rad/s²
XY/yaw goal tolerance: 0.25 m / 0.25 rad
```

Costmaps:

```text
robot_radius: 0.5 m
inflation_radius: 0.8 m
local rolling window: 6 x 6 m, 0.05 m resolution
local VoxelLayer consumes /scan
global StaticLayer + ObstacleLayer consume map and /scan
raytrace max: 10 m
obstacle max: 8 m
global planner allow_unknown: false
```

Velocity smoother giới hạn `[x,y,yaw]` ở `±[0.3,0.3,0.3]`, acceleration `[0.3,0.3,0.5]`, open-loop 20 Hz.

AMCL covariance trong test vẫn khoảng `0.26–0.27 m²` cho XY và `0.08 rad²` cho yaw, nhưng particle cloud giữ một cụm và `map -> odom` ổn định qua các phép quay/dịch chuyển. Không tuning chỉ dựa vào covariance nếu runtime alignment vẫn tốt; đo lại khi thay map/sensor/model.

## 8. Quy trình chạy navigation đã kiểm chứng

Đúng workspace setup là:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Không dùng path cũ `/home/dolphiinn/Documents/ros2_ws/install/setup.bash`.

### Terminal PX4/Gazebo

Trong PX4-Autopilot, chạy SITL target/world đã cấu hình, tương đương prerequisite được ghi trong launch:

```bash
make px4_sitl gz_x500_lidar_2d_urban_uavcup
```

### Terminal Micro XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal robot-side stack

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_slam uavcup_robot.launch.py
```

### Terminal localization/Nav2

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_nav uavcup_navigation.launch.py
```

Map phải hiện trong RViz. Trước Initial Pose, lỗi thiếu `map -> base_footprint` là bình thường.

### Enable và vào Offboard

```bash
ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: true}"
```

Chờ heartbeat ít nhất một giây, Arm bằng QGroundControl, rồi:

```bash
ros2 service call /cmd_vel_to_px4/request_offboard \
  std_srvs/srv/Trigger "{}"
```

Kiểm tra:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 --once | \
  grep -E 'arming_state:|nav_state:|failsafe:'
```

### Initial localization

Khi drone hover khoảng 0.7 m, dùng **2D Pose Estimate** trong RViz, click đúng vị trí trên saved map và kéo theo heading của drone. Đặt lại nếu laser chưa trùng vật cản.

```bash
timeout 5 ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /amcl_pose --once
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
```

Sau khi các node active và scan/map khớp, dùng **Nav2 Goal**. Goal đầu tiên nên ngắn, nằm giữa free space và tránh vùng inflation màu cyan/hồng.

### Dừng an toàn

1. Cancel Nav2 action trong panel Navigation 2.
2. Chuyển PX4 sang Position/Loiter trong QGroundControl.
3. Sau khi PX4 rời Offboard mới disable adapter:

```bash
ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: false}"
```

## 9. Mapping và lưu map

Mapping launch:

```bash
ros2 launch px4_uavcup_slam uavcup_slam.launch.py
```

Launch này include robot-side stack, chạy SLAM Toolbox và mapping RViz. Không chạy navigation launch cùng lúc.

Giữ drone ở cao độ gần cố định, bay chậm, hover khi đổi hướng và tránh tilt lớn. Scan khi dưới 0.5 m hoặc tilt quá 5° sẽ bị lọc. Một map đã xé vì scan sai/tilt không tự phục hồi hoàn toàn; nên restart SLAM session rồi quét lại.

Lưu map khi `/map` ổn định:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/uavcup_team1 \
  --ros-args -p use_sim_time:=true
```

## 10. Build, test và diagnostic commands

Build hai package project:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select px4_uavcup_slam px4_uavcup_nav
source install/setup.bash
```

Unit tests đã chạy tại thời điểm handoff:

```bash
python3 -m pytest -q src/px4_uavcup_slam/test
# 22 passed
```

Các diagnostic hữu ích:

```bash
ros2 node list | sort
ros2 topic list | grep -E '^/(clock|scan|odom|map)$'
timeout 5 ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
timeout 5 ros2 run tf2_ros tf2_echo map odom
ros2 topic info /cmd_vel -v
ros2 action list | grep navigate_to_pose
```

Robot-only expected: `/clock`, `/scan`, `/odom`, không có `/map`. Navigation expected: thêm `/map`, `/amcl_pose`, Nav2 actions và `map -> odom` sau Initial Pose.

## 11. Kết quả định lượng đã quan sát

- `/scan`: khoảng `29.97–30.00 Hz`;
- adapter heartbeat: khoảng 20 Hz;
- manual `/cmd_vel x=0.1 m/s` thử nghiệm ban đầu di chuyển khoảng 0.28 m rồi tự dừng, altitude thay đổi chỉ cỡ millimeter;
- phép test quay giữ `map -> odom` gần như cố định;
- phép tiến ngắn làm odom đổi khoảng 0.12 m, trong khi `map -> odom` chỉ recorrect khoảng 6 mm và dưới 1° rồi ổn định;
- PX4 Offboard ACK accepted, `nav_state=14`, armed và không failsafe;
- nhiều goal, goal đổi yaw và static obstacle avoidance đều thành công trong Gazebo.

## 12. Hạn chế và việc tiếp theo được khuyến nghị

### Hạn chế hiện tại

- Chỉ kiểm thử SITL/Gazebo, chưa đủ điều kiện áp dụng lên hardware thật.
- Navigation là 2D ở cao độ cố định 0.7 m; chưa phải 3D planner.
- LiDAR gắn cứng, không có gimbal; scan bị bỏ khi tilt quá 5°.
- Obstacle avoidance đã xác nhận với vật cản tĩnh; chưa xác nhận dynamic obstacles.
- Chưa có automatic takeoff/landing orchestration trong Nav2 pipeline.
- Chưa có mission state machine cho nhận/thả kiện hàng.
- Joystick Switch Pro vẫn có vấn đề calibration/input nhảy trong QGroundControl.
- Default map path trong nav launch phụ thuộc `~/ros2_ws`.
- Root README cần cập nhật vì vẫn ghi project dừng ở SLAM.

### Nhiệm vụ tiếp theo hợp lý: waypoint mission

Branch đã có tên `feat/waypoint-mission`. Nên tạo một node/package mission nhỏ dùng `nav2_simple_commander.BasicNavigator`, theo lesson 45–51:

1. nhận hoặc cấu hình initial pose;
2. đợi Nav2 active;
3. định nghĩa waypoint trong frame `map`;
4. gọi `followWaypoints()` hoặc `goThroughPoses()`;
5. log feedback/current waypoint;
6. xử lý cancel/timeout/failure;
7. không tự arm/Offboard nếu chưa thiết kế một state machine safety rõ ràng;
8. trước tiên thử 2–3 waypoint ngắn trong free space, sau đó mới gắn logic nhận/thả kiện.

Giữ adapter và các safety invariant hiện tại. Không mở rộng quyền điều khiển PX4, tự arm hoặc tự bypass preflight chỉ để làm demo mission.

## 13. Quy tắc khi AI tiếp tục chỉnh project

- Đọc `src/px4_uavcup_slam/README.md` và file này trước khi sửa.
- Đọc lesson liên quan trong local course; user muốn triển khai từng bước và hiểu mỗi thay đổi.
- Không chạy đồng thời SLAM Toolbox và AMCL.
- Không thay planar `base_footprint` bằng full 3D pose.
- Không bỏ NED/ENU + FRD/FLU conversions hoặc EKF reset compensation.
- Không hạ scan filters mà không giải thích tác động lên SLAM/AMCL.
- Không tự arm PX4 hoặc tự động request Offboard ngoài flow được user chấp thuận.
- Không disable adapter khi PX4 còn Offboard.
- Preserve map và `maps_backup/`.
- Preserve untracked `src/px4_msgs/`; hỏi trước khi thực hiện cleanup/destructive Git actions.
- Sau thay đổi Python/conversion/control, chạy lại 22 unit tests và build package liên quan.
- Với thay đổi Nav2, xác minh lifecycle, TF, scan-map alignment, `/cmd_vel`, altitude và failsafe trước khi tăng quãng đường/speed.


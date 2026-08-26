# PX4 UAV Cup SLAM

Package nối LiDAR 2D của Gazebo Harmonic và odometry PX4 vào ROS 2 Humble để tạo map bằng `slam_toolbox`.

## Trạng thái

Đang làm **Phase 3-2**: `Gazebo LaserScan -> ROS 2 -> TF/odom -> SLAM Toolbox -> RViz2 -> map .pgm/.yaml`.

- Đã có bridge riêng cho Gazebo Harmonic (`gz-msgs10`, `transport13`).
- Đã có chuyển đổi PX4 NED/FRD sang ROS ENU/FLU và cây TF `map -> odom -> base_footprint -> base_link -> link`.
- Đã cấu hình SLAM Toolbox online async và RViz2.
- Bridge bỏ qua scan khi drone nghiêng quá 5° hoặc thấp hơn 0,5 m để tránh LiDAR 2D quét xuống đất và trộn nhiều lát cắt cao độ.
- Đã lưu map và đang chuyển sang tích hợp Navigation2 theo workflow custom robot.

## Chạy

Mở ba terminal đầu:

```bash
# Terminal 1
cd ~/Documents/px4-myself/firmware/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d_urban_uavcup

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select px4_msgs px4_uavcup_slam
source install/setup.bash
ros2 launch px4_uavcup_slam uavcup_slam.launch.py
```

Launch mặc định mở luôn RViz2 với `/map`, `/scan` và TF. Chạy headless bằng `rviz:=false`. Nếu Gazebo spawn tên model khác, tìm bằng `gz topic -l | grep -E 'lidar|scan'`, rồi truyền ví dụ `model:=x500_lidar_2d_1`.

## Nếu RViz2 không thấy dữ liệu

Chạy theo thứ tự sau, không bỏ qua checkpoint:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic hz /scan
ros2 topic echo /scan --once | sed -n '1,20p'
ros2 topic echo /clock --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link link
ros2 topic hz /map
```

Kỳ vọng quan trọng:

| Check | Giá trị đúng | Nếu sai |
| --- | --- | --- |
| `/scan` | Có tần số, `frame_id: link` | Kiểm tra đúng Gazebo model/topic |
| `/clock` | Thời gian tăng | Bridge chưa nhận clock Gazebo |
| PX4 local position | Có message | Kiểm tra Micro XRCE-DDS Agent và topic có/không có hậu tố `_v1` |
| TF | `odom -> base_footprint -> base_link -> link` tồn tại | `px4_odom_tf` chưa nhận PX4 data |
| `/map` | Có message sau khi scan + TF hợp lệ | Xem log `slam_toolbox` |

Log bridge phải in `GZ LaserScan msgs in last 2s: N` mỗi 2 giây. Nếu `N = 0`, topic Gazebo đang sai hoặc LiDAR chưa chạy; nếu hoàn toàn không có dòng heartbeat, hãy chắc chắn package vừa được build lại và terminal đã source đúng `~/ros2_ws/install/setup.bash`.

Ở phiên bản hiện tại heartbeat có dạng:

```text
LaserScan last 2s: GZ=60 ROS=60 tilt_rejected=0 altitude_rejected=0 tilt=1.2deg altitude=2.50m
```

Scan chỉ được đưa vào SLAM khi drone cao từ `0.5 m` và `tilt` không vượt
quá 5°. Nhờ vậy lidar vẫn quét được các vật cản và điểm đặt kiện hàng thấp,
nhưng bỏ qua dữ liệu khi drone còn sát mặt đất hoặc nghiêng mạnh. Có thể đổi
hai ngưỡng bằng parameter `min_mapping_altitude_m` và `max_tilt_deg`; giá trị
`0` tắt bộ lọc tương ứng.

Scan lúc cất/hạ cánh dưới 0,5 m cũng bị bỏ qua. SLAM 2D chỉ hợp lệ khi LiDAR quét một lát cắt cao độ gần như cố định; không được trộn map lúc drone nằm dưới đất với map lúc đang bay.

Nếu RViz báo `Frame [map] does not exist`, đặt Fixed Frame tạm thành `odom`: nếu scan hiện thì phần bridge + TF đã chạy, lỗi còn lại nằm ở SLAM Toolbox. Đừng đổi Fixed Frame thành tên scoped dài của Gazebo.

Nếu mở RViz thủ công, bắt buộc dùng simulated time:

```bash
rviz2 \
  -d $(ros2 pkg prefix px4_uavcup_slam)/share/px4_uavcup_slam/rviz/uavcup_slam.rviz \
  --ros-args -p use_sim_time:=true
```

`use_sim_time` là ROS parameter truyền lúc chạy; RViz2 không có nút tương ứng trong Global Options.

## Lưu map

Khi `/map` đã ổn định và SLAM vẫn chạy:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/uavcup_team1 \
  --ros-args -p use_sim_time:=true
```

Đầu ra là `~/maps/uavcup_team1.pgm` và `~/maps/uavcup_team1.yaml`.

## Quét lại map sau khi sửa TF

Map đã méo không tự phục hồi hoàn toàn. Sau khi build code mới, dừng launch cũ bằng `Ctrl+C`, launch lại để tạo session SLAM trắng rồi mới bay quét lại. Giữ cao độ ổn định, bay chậm và dừng hover khi đổi hướng; hạn chế roll/pitch lớn vì LiDAR hiện gắn cứng vào thân drone, chưa có gimbal cân bằng.

## Lỗi frame đã sửa

Gazebo có thể đặt `LaserScan.frame` thành một scoped sensor path dài. Cây TF của project chỉ khai báo laser frame là `link`, nên RViz2 và SLAM Toolbox không tìm được transform nếu bridge sao chép nguyên frame Gazebo. Bridge hiện luôn publish frame ROS đã cấu hình (`link`) để khớp static TF `base_link -> link`.

Bridge cũng dùng system-time riêng cho timer publish. Đây là bắt buộc vì bridge là nguồn tạo `/clock`: nếu timer đó dùng simulated time, nó sẽ chờ chính `/clock` mà nó chưa kịp publish và toàn pipeline đứng ở 0 message.

Quaternion PX4 được hiểu đúng theo message definition là phép quay **FRD body → NED earth**. Thứ tự đổi hệ trục phải là `R_enu_ned × R_ned_frd × R_frd_flu`; thứ tự cũ làm yaw ROS lệch 180°, khiến scan nằm ngược hướng chuyển động và xé map thành các nan dài.

SLAM Toolbox là 2D nên không được nhận trực tiếp pose 3D của drone. Cây TF tách `base_footprint` phẳng (x/y/yaw) cho SLAM và `base_link` chứa z/roll/pitch thật của thân drone. Cách này tương đương cấu trúc thường thấy trên TurtleBot3 và ngăn `map -> odom` bị nghiêng khi drone pitch/roll.

PX4 EKF có thể reset local position hoặc heading khi đổi nguồn fusion. Node theo dõi `xy_reset_counter` và `heading_reset_counter`, rồi cộng offset ngược với `delta_xy`/`delta_heading` để ROS `odom` vẫn liên tục. Log sẽ báo rõ mỗi lần reset được bù.

## Navigation2 — checkpoint 1: `/cmd_vel` sang PX4

Theo lesson 40–42 của course, Nav2 cần robot cung cấp TF, `/odom`, `/scan`
và một controller nhận `/cmd_vel`. Ba interface đầu đã có. Node
`cmd_vel_to_px4` hoàn thiện interface cuối bằng cách:

```text
Nav2 Twist (base FLU)
  -> giới hạn tốc độ/gia tốc
  -> xoay theo heading hiện tại
  -> PX4 velocity NED + position Z cố định
```

Giá trị mặc định:

| Parameter | Giá trị | Ý nghĩa |
| --- | ---: | --- |
| `target_altitude_m` | `0.7` | Giữ độ cao 0,7 m so với local origin |
| `max_xy_speed_m_s` | `0.4` | Giới hạn độ lớn vận tốc ngang |
| `max_yaw_rate_rad_s` | `0.3` | Giới hạn tốc độ quay |
| `max_xy_accel_m_s2` | `0.3` | Ramp lệnh ngang để giảm tilt |
| `cmd_timeout_s` | `0.5` | Mất lệnh thì phanh XY/yaw về 0 |

Node **không tự arm, không tự takeoff và không tự chuyển Offboard**. Nó mặc
định disabled và chỉ publish heartbeat/setpoint sau khi enable bằng service.

Sau khi Gazebo, PX4 và Micro XRCE-DDS Agent đã chạy:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run px4_uavcup_px4_bridge cmd_vel_to_px4 \
  --ros-args -p use_sim_time:=true
```

Terminal khác, xác nhận PX4 state hợp lệ rồi enable:

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once

ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: true}"
```

Kỳ vọng service trả `success: true`. Chờ ít nhất một giây để PX4 nhận
Offboard heartbeat, sau đó nhấn Arm trong QGroundControl. QGroundControl có
thể không liệt kê Offboard trong menu; khi đó yêu cầu mode bằng service:

```bash
ros2 service call /cmd_vel_to_px4/request_offboard \
  std_srvs/srv/Trigger "{}"
```

Node sẽ log ACK của PX4. Chỉ tiếp tục khi ACK là `ACCEPTED` và
`vehicle_status.nav_state` bằng `14` (Offboard).

Thử lệnh rất nhỏ trong SITL:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Drone chỉ tiến trong thời gian timeout rồi tự đưa lệnh ngang về 0 và giữ
độ cao. Trước khi kết thúc, chuyển PX4 từ Offboard về Position/Loiter trong
QGroundControl, sau đó mới disable adapter:

```bash
ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: false}"
```

Không disable adapter khi PX4 vẫn đang ở Offboard, vì mất heartbeat sẽ kích
hoạt xử lý failsafe theo `COM_OF_LOSS_T`.

## Navigation2 — checkpoint 2: tách robot stack và localization

Course lesson 43 tách custom robot stack khỏi SLAM/Nav2. Project có hai launch:

```text
uavcup_robot.launch.py
  -> /clock, /scan, /odom
  -> odom -> base_footprint -> base_link -> link
  -> /cmd_vel sang PX4 (mặc định disabled)

uavcup_slam.launch.py
  -> include uavcup_robot.launch.py
  -> slam_toolbox sở hữu map -> odom
  -> RViz
```

Chạy mapping như trước:

```bash
ros2 launch px4_uavcup_slam uavcup_slam.launch.py
```

Chạy robot-only để chuẩn bị static-map localization:

```bash
ros2 launch px4_uavcup_slam uavcup_robot.launch.py
```

Robot-only mode không được có `/slam_toolbox`, `/map` hoặc frame `map`. Kiểm tra:

```bash
ros2 node list | sort
ros2 topic list | grep -E '^/(clock|scan|odom|map)$'
ros2 run tf2_ros tf2_echo odom base_footprint
timeout 3 ros2 run tf2_ros tf2_echo map odom
```

Kỳ vọng ba node robot-side:

```text
/cmd_vel_to_px4
/gz_lidar_bridge
/px4_odom_tf
```

Lệnh `map -> odom` phải thất bại trong robot-only mode. Đây là chủ ý: khi
mapping, `slam_toolbox` publish transform này; khi navigation bằng saved map,
AMCL sẽ publish nó. Không bao giờ chạy cả `slam_toolbox` và AMCL cùng lúc.

## Navigation2 — checkpoint 3: static map và AMCL

Gói `px4_uavcup_nav` chạy map server, AMCL và các server của Nav2 trên map
đã lưu. AMCL là node duy nhất sở hữu transform `map -> odom`; PX4 odometry
vẫn sở hữu `odom -> base_footprint`.

Giữ `uavcup_robot.launch.py` chạy ở terminal thứ nhất. Ở terminal thứ hai:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_nav uavcup_navigation.launch.py
```

Trước khi đặt initial pose, RViz có thể báo chưa có transform từ `map` đến
`base_footprint`, và Nav2 có thể đang chờ global costmap activate. Đây là
trạng thái bình thường: AMCL chưa biết drone đang ở đâu trên saved map.

Đưa drone lên độ cao quét `0.7 m` nhưng **chưa gửi Navigation Goal**. Trong
RViz chọn **2D Pose Estimate**, bấm vào đúng vị trí drone trên map rồi kéo mũi
tên theo hướng đầu drone. Laser trắng cần chồng lên đúng các tường/vật cản
đen; nếu chưa khớp, đặt lại 2D Pose Estimate chính xác hơn.

Kiểm tra ở terminal thứ ba:

```bash
timeout 5 ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /amcl_pose --once
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
```

Checkpoint đạt khi:

- `map -> odom` trả về transform liên tục;
- `/amcl_pose` có dữ liệu;
- map server, AMCL và planner đều báo `active`;
- laser scan nằm trùng biên vật cản trên map khi drone hover.

Chưa bấm **Nav2 Goal** ở checkpoint này. Nếu `map -> odom` vẫn không xuất
hiện, kiểm tra `/scan` có message hay không; bridge cố ý loại scan khi drone
thấp hơn `0.5 m` hoặc nghiêng quá `5°`.

## Navigation2 — checkpoint 4: localization và goal đầu tiên

Trước khi giao quyền điều khiển cho Nav2, kiểm tra AMCL bằng một phép quay và
một phép dịch chuyển ngắn qua `/cmd_vel`. Kết quả đạt khi `/scan` liên tục,
particle cloud giữ thành một cụm và `map -> odom` chỉ hiệu chỉnh nhỏ rồi ổn
định. Offset giữa `map` và `odom` là correction của AMCL, không tự động có
nghĩa là hệ thống đang drift.

Xác nhận toàn bộ navigation stack sẵn sàng:

```bash
for node in controller_server planner_server bt_navigator behavior_server; do
  ros2 lifecycle get "/$node"
done

ros2 action list | grep navigate_to_pose
ros2 topic info /cmd_vel -v
```

Các lifecycle node phải là `active [3]`. `/cmd_vel` phải có Nav2 velocity
smoother làm publisher và `cmd_vel_to_px4` làm subscriber.

Goal đầu tiên được đặt bằng **Nav2 Goal** trong RViz, cách drone khoảng
`0.5–0.7 m`, nằm giữa vùng free space và giữ cùng heading hiện tại. Project
đã xác nhận nhiều goal loại này hoạt động tốt; sai số odometry được AMCL
correct lại theo saved map.

Khi cần dừng khẩn cấp:

1. Nhấn **Cancel** trong panel Navigation 2.
2. Chuyển PX4 sang Position/Loiter trong QGroundControl.
3. Sau khi PX4 đã rời Offboard, disable adapter:

```bash
ros2 service call /cmd_vel_to_px4/enable \
  std_srvs/srv/SetBool "{data: false}"
```

Không disable adapter khi PX4 còn ở Offboard vì thao tác đó làm mất heartbeat.

## Navigation2 — checkpoint 5: yaw và tránh vật cản

Các bài kiểm tra cuối trong SITL đã đạt:

- drone quay đúng chiều goal và `angular.z` không vượt `±0.3 rad/s`;
- độ cao giữ ổn định, không nghiêng quá `5°` trong thời gian dài;
- laser tiếp tục khớp saved map sau khi quay;
- `/cmd_vel` trở về 0 và drone hover sau khi hoàn thành goal;
- AMCL không nhảy sang một cụm vật cản giống nhau khác;
- khi vật cản tĩnh nằm giữa drone và goal, Nav2 tạo global path vòng qua vật
  cản và drone bám đường thành công.

Như vậy pipeline đã được kiểm chứng end-to-end:

```text
saved map + /scan + /odom
  -> AMCL: map -> odom
  -> Nav2 planner + costmaps
  -> controller + velocity smoother
  -> /cmd_vel
  -> cmd_vel_to_px4
  -> PX4 Offboard giữ cao độ 0.7 m
```

Phạm vi hiện đã xác nhận là localization và navigation 2D với vật cản tĩnh
trong Gazebo. Dynamic obstacles, waypoint mission, behavior khi mất scan và
thử nghiệm trên drone thật cần các checkpoint an toàn riêng; không nên xem
kết quả SITL hiện tại là đã xác nhận cho phần cứng thật.

# PX4 UAV Cup SLAM

Package nối LiDAR 2D của Gazebo Harmonic và odometry PX4 vào ROS 2 Humble để tạo map bằng `slam_toolbox`.

## Trạng thái

Đang làm **Phase 3-2**: `Gazebo LaserScan -> ROS 2 -> TF/odom -> SLAM Toolbox -> RViz2 -> map .pgm/.yaml`.

- Đã có bridge riêng cho Gazebo Harmonic (`gz-msgs10`, `transport13`).
- Đã có chuyển đổi PX4 NED/FRD sang ROS ENU/FLU và cây TF `map -> odom -> base_footprint -> base_link -> link`.
- Đã cấu hình SLAM Toolbox online async và RViz2.
- Bridge bỏ qua scan khi drone nghiêng quá 5° hoặc thấp hơn 1,5 m để tránh LiDAR 2D quét xuống đất và trộn nhiều lát cắt cao độ.
- Chưa xác nhận trọn pipeline bằng một lượt bay quét map và chưa lưu map chính thức.

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

Scan lúc cất/hạ cánh dưới 1,5 m cũng bị bỏ qua. SLAM 2D chỉ hợp lệ khi LiDAR quét một lát cắt cao độ gần như cố định; không được trộn map lúc drone nằm dưới đất với map lúc bay 2–3 m.

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

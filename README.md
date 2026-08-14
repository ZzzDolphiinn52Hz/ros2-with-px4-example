# Autonomous Urban Drone Delivery — ROS 2 workspace

Workspace ROS 2 Humble cho drone PX4 v1.17 chạy SITL/Gazebo, hướng tới bay tự động trong sa bàn UAV Cup.

## Các package project

| Package | Vai trò |
| --- | --- |
| `px4_state_reader` | Đọc và quan sát state từ PX4 qua uXRCE-DDS |
| `px4_offboard_baseline` | Các bài bay Offboard: hold, takeoff/hover, tiến rồi dừng |
| `px4_uavcup_slam` | Bridge LiDAR Gazebo, publish odom/TF và chạy SLAM Toolbox |
| `px4_msgs` | Message interface khớp PX4 v1.17 |

## Tiến độ hiện tại

Các bước setup, SITL, LiDAR/collision prevention, ROS 2 Offboard và custom world `urban_uavcup` đã hoàn thành. Dự án đang dừng ở **Phase 3-2 — SLAM map**:

```text
Gazebo lidar_2d_v2
  -> /scan + /clock
  -> odom -> base_link -> link
  -> slam_toolbox (/map, map -> odom)
  -> RViz2
  -> uavcup_team1.pgm + uavcup_team1.yaml
```

Hướng dẫn chạy và xử lý lỗi RViz2 nằm tại [`src/px4_uavcup_slam/README.md`](src/px4_uavcup_slam/README.md).

Ghi chú học tập chi tiết bên ngoài repository: `~/ObsidianVaults/Study/03-Projects/a-Drone-Sep1st/`, đặc biệt `Practice-Log/3-2-slam-map-guide.md`.

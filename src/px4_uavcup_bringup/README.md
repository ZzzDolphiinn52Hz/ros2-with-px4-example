# PX4 UAV Cup bringup

Launch toàn bộ stack Pi. Config xe nằm trong `config/` của package này.
Launch test ZipDepth/ArUco dùng config cùng tên trong `px4_uavcup_perception`.

```bash
ros2 launch px4_uavcup_bringup pi_vehicle.launch.py
```

Test riêng:

```bash
ros2 launch px4_uavcup_bringup aruco_test.launch.py
ros2 launch px4_uavcup_bringup zipdepth_test.launch.py
ros2 launch px4_uavcup_bringup pi_perception_test.launch.py
```

`pi_perception_test.launch.py` chạy đồng thời USB ZipDepth và Pi Camera
ArUco để kiểm tra perception; launch này không tạo PID, controller hoặc
PX4 bridge node.

| File | Node |
| --- | --- |
| `config/pi_cameras.yaml` | USB camera + Pi camera |
| `config/zipdepth.yaml` | ZipDepth |
| `config/aruco.yaml` | ArUco detector |
| `config/landing.yaml` | ArUco landing PID |
| `config/px4_bridge.yaml` | `cmd_vel_to_px4` + landing-target |

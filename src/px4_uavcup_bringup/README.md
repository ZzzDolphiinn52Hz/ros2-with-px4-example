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

`pi_perception_test.launch.py` chạy đồng thời IMX219 ZipDepth và IMX500
ArUco để kiểm tra perception; launch này không tạo PID, controller hoặc
PX4 bridge node.

Topology cố định của Pi là:

| CSI port | Camera | Pipeline | Unix socket |
| --- | --- | --- | --- |
| CAM0 | IMX219 phía trước | ZipDepth/free-space | `run/front_camera.sock` |
| CAM1 | IMX500 nhìn xuống | ArUco pose | `run/down_camera.sock` |

Trên Raspberry Pi OS, khởi động cả hai frame server bằng một lệnh trước khi
launch ROS container:

```bash
cd ~/ros2_ws
PYTHONPATH=$PWD/src/px4_uavcup_perception:$PYTHONPATH \
/usr/bin/python3 \
  src/px4_uavcup_perception/scripts/picamera2_dual_server.py
```

Trong lúc chưa gắn CAM1, chỉ test nhánh CAM0 bằng:

```bash
ros2 launch px4_uavcup_bringup pi_perception_test.launch.py \
  enable_aruco:=false
```

| File | Node |
| --- | --- |
| `config/pi_cameras.yaml` | IMX500 camera nhìn xuống tại CAM1 |
| `config/zipdepth.yaml` | ZipDepth |
| `config/aruco.yaml` | ArUco detector |
| `config/landing.yaml` | ArUco landing PID |
| `config/px4_bridge.yaml` | `cmd_vel_to_px4` + landing-target |

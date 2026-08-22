# PX4 UAV Cup camera perception

Mốc đầu tiên của pipeline không dùng SLAM/Nav2:

```text
Gazebo RGB camera -> ROS Image -> Depth Anything V2 metric
                  -> depth 32FC1 -> left/center/right free-space summary
```

Package này chỉ cảm nhận môi trường. Nó không arm, đổi mode hay publish PX4
trajectory setpoint, vì vậy không xung đột với `px4_offboard_baseline`.

## Topic

| Topic | Type | Nội dung |
| --- | --- | --- |
| `/uav/front_camera/image_raw` | `sensor_msgs/Image` | Ảnh RGB 320x240 (downsample từ sensor 640x480) |
| `/uav/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics camera mô phỏng |
| `/uav/depth/image` | `sensor_msgs/Image` | Depth mét, encoding `32FC1` |
| `/uav/depth/visualization` | `sensor_msgs/Image` | Depth `mono8`, vật gần sáng hơn |
| `/uav/depth/free_space` | `std_msgs/Float32MultiArray` | `[left, center, right, nearest, valid_fraction]` |
| `/uav/depth/status` | `diagnostic_msgs/DiagnosticArray` | Model, device và latency |

## Cài Depth Anything V2

Không cài model vào Python hệ thống của ROS. Script tạo virtual environment có
`--system-site-packages`, clone repository chính thức và tải checkpoint metric
indoor Small. Installer dùng PyTorch CPU để không tải CUDA trên máy hiện tại:

```bash
cd ~/ros2_ws/src/px4_uavcup_perception
bash scripts/install_depth_anything_v2.sh
source ~/ros2_ws/.venv-depth/bin/activate
```

Máy hiện tại từng có NumPy 2.x trong user site trong khi OpenCV của Ubuntu được
build với NumPy 1.x. Virtual environment trên giữ `numpy<2` để tránh lỗi ABI.

## Build

Phải activate virtual environment trước lúc build để console script dùng đúng
Python có Torch/OpenCV:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv-depth/bin/activate
cd ~/ros2_ws
colcon build --symlink-install --packages-select px4_uavcup_perception
source install/setup.bash
```

## Chạy mô phỏng

Terminal 1, dùng model `x500_uavcup` với camera trước 640x480, 20 Hz:

```bash
cd ~/Documents/px4-myself/firmware/PX4-Autopilot
PX4_GZ_MODEL_POSE="1.5,-9,0.25,0,0,0" \
make px4_sitl gz_x500_uavcup_urban_uavcup
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch px4_uavcup_perception perception_sim.launch.py
```

Launch tự thêm `~/ros2_ws/.venv-depth` vào `PYTHONPATH` chỉ cho node depth;
không cần activate virtual environment ở terminal chạy sau khi package đã build.

`input_size` mặc định đang là 252 để benchmark trên CPU của máy mô phỏng. Khi
chạy trên GPU/NPU, đổi lại 518 trong `config/perception.yaml` và đo latency trước
khi nối kết quả vào local controller.

Chỉ kiểm tra camera/bridge mà chưa cài Torch hoặc checkpoint:

```bash
ros2 launch px4_uavcup_perception perception_sim.launch.py run_depth:=false
```

Kiểm tra dữ liệu:

```bash
ros2 topic hz /uav/front_camera/image_raw
ros2 topic hz /uav/depth/image
ros2 topic echo /uav/depth/free_space
ros2 topic echo /uav/depth/status
```

Không chạy đồng thời `gz_lidar_bridge` cũ vì cả hai bridge đều có thể publish
`/clock`. Pipeline perception không cần launch của SLAM hoặc Nav2.

# PX4 UAV Cup camera perception

Mốc đầu tiên của pipeline không dùng SLAM/Nav2:

```text
Gazebo RGB camera -> ROS Image -> Depth Anything V2 metric
                  -> depth 32FC1 -> left/center/right free-space summary
```

Package này chỉ cảm nhận môi trường. Nó không arm, đổi mode hay publish PX4
trajectory setpoint, vì vậy không xung đột với `px4_offboard_baseline`.

Hai môi trường được tách rõ:

- `perception_sim.launch.py`: bridge Gazebo và chạy Depth Anything V2 PyTorch.
- `perception_jetson.launch.py`: đọc camera, chạy TensorRT và tính trực tiếp
  `left/center/right` trên depth. PointCloud, ảnh depth và ảnh visualization đều
  tắt mặc định để giảm copy/serialization khi bay.

## Topic

| Topic | Type | Nội dung |
| --- | --- | --- |
| `/uav/front_camera/image_raw` | `sensor_msgs/Image` | Ảnh RGB 320x240 (downsample từ sensor 640x480) |
| `/uav/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics camera mô phỏng |
| `/uav/depth/image` | `sensor_msgs/Image` | Depth mô phỏng, encoding `32FC1` |
| `/camera/depth/image` | `sensor_msgs/Image` | Depth Jetson tùy chọn, encoding `32FC1` |
| `/uav/depth/visualization` | `sensor_msgs/Image` | Depth `mono8`, vật gần sáng hơn |
| `/uav/depth/free_space` | `std_msgs/Float32MultiArray` | `[left, center, right, nearest, valid_fraction]` |
| `/uav/depth/status` | `diagnostic_msgs/DiagnosticArray` | Model, device và latency |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Point cloud FLU tùy chọn để debug RViz |

`valid_fraction` không chỉ phản ánh depth hữu hạn. Trên Jetson, camera health
gate còn chặn ảnh quá tối, quá sáng hoặc thiếu texture. Khi camera bị che, node
không tin giá trị monocular depth và publish:

```text
[NaN, NaN, NaN, NaN, 0.0]
```

Diagnostic `/uav/depth/status` chuyển sang `ERROR` và chứa brightness,
contrast, gradient, dark/bright fraction cùng lý do fail-safe. Sau lỗi kéo dài,
camera phải có 5 frame tốt liên tiếp trước khi depth được sử dụng lại.

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

## Chạy trên Jetson ROS 2 Foxy

Build overlay riêng, không build các package Nav2/SLAM và không ghi đè workspace
`~/ros2_ws` đang chứa pipeline TensorRT:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/uavcup_ws
colcon --log-base log_foxy build \
  --build-base build_foxy \
  --install-base install_foxy \
  --symlink-install \
  --packages-select \
    px4_msgs \
    px4_state_reader \
    px4_offboard_baseline \
    px4_uavcup_perception
source ~/uavcup_ws/install_foxy/setup.bash
```

Chế độ bay chỉ chạy TensorRT và publish free-space/status:

```bash
ros2 launch px4_uavcup_perception perception_jetson.launch.py
```

Camera flight được center-crop `1280x720 -> 720x720` rồi resize về
`364x364`, thay vì kéo méo ảnh 16:9 thành hình vuông. Depth publish và L/C/R
đều áp dụng calibration tuyến tính cấu hình bằng
`depth_calibration_scale * raw + depth_calibration_bias_m`.

Kiểm tra output điều khiển và hiệu năng:

```bash
ros2 topic hz /uav/depth/free_space
ros2 topic echo /uav/depth/free_space
ros2 topic echo /uav/depth/status
```

Thử camera health gate trên bàn bằng cách che rồi mở camera. Khi che, log phải
có `Perception fail-safe`, L/C/R phải là `NaN` và `valid_fraction=0`; khi mở lại
phải có `Camera health gate recovered` trước khi khoảng cách hợp lệ xuất hiện.

Chế độ debug có thể bật từng output nặng khi cần:

```bash
ros2 launch px4_uavcup_perception perception_jetson.launch.py \
  publish_depth_image:=true \
  publish_visualization:=true \
  publish_pointcloud:=true
```

Không chạy đồng thời launch `depth_to_pointcloud/octomap_depth.launch.py` cũ vì
hai node sẽ tranh `/dev/video0`. Pipeline vận hành không khởi động OctoMap,
RTAB-Map, SLAM, Nav2, visual odometry tự viết hoặc TF `map -> camera_link`.

Point cloud debug dùng hệ FLU (`x` trước, `y` trái, `z` lên). Intrinsics mặc
định giữ giá trị thử nghiệm `fx=fy=300 px`; phải thay bằng calibration camera
trước khi dùng point cloud để đo hình học chính xác. Depth metric cũng phải được
đối chiếu với vài khoảng cách đo thật trước khi nối vào local controller.

### Hiệu chuẩn intrinsic camera thật

Luồng hiệu chuẩn capture MJPEG gốc `1280x720`, giữ nguyên FOV 16:9 và scale về
`640x360` để ROS 2 Foxy publish ổn định. Không chạy đồng thời với
`perception_jetson.launch.py` vì cả hai cùng mở `/dev/video0`:

```bash
ros2 launch px4_uavcup_perception camera_calibration.launch.py
```

Ảnh được publish tại `/camera/image_raw` ở 10 Hz. Dùng bảng checkerboard
9x6 góc trong, đo chính xác cạnh ô, và công cụ `camera_calibration` chuẩn của
ROS 2. Không hiệu chuẩn trên ảnh `364x364`: đó là ảnh đã bị scale không đồng
đều cho TensorRT.

Bảng chuẩn nằm tại `calibration/checkerboard_9x6_30mm_a3.svg`: in A3 ngang ở
100% / Actual size, tắt Fit to page, rồi đo lại một ô phải đúng `30.0 mm`.

# PX4 UAV Cup Jetson perception

Package này chỉ giữ pipeline bay thật trên Jetson Xavier NX:

```text
/dev/video0 -> center crop / resize -> camera health -> TensorRT
            -> linear depth calibration -> free-space
            -> /uav/depth/free_space
```

Package chỉ cảm nhận môi trường. Nó không arm, không đổi mode, không publish
PX4 trajectory setpoint và không tạo publisher `/fmu/in/*`.

Pipeline Gazebo/PyTorch trước đây (`gz_image_bridge`, `depth_anything_node`,
`free_space_node`, `perception_sim.launch.py`) đã bị loại bỏ. Mô phỏng không còn
là runtime được hỗ trợ của package này.

## Output

| Topic | Type | Nội dung |
| --- | --- | --- |
| `/uav/depth/free_space` | `std_msgs/Float32MultiArray` | `[left, center, right, nearest, valid_fraction]` |
| `/uav/depth/status` | `diagnostic_msgs/DiagnosticArray` | Health, calibration và latency |
| `/camera/depth/image` | `sensor_msgs/Image` | Depth `32FC1`, chỉ bật khi debug |
| `/uav/depth/visualization` | `sensor_msgs/Image` | Depth `mono8`, chỉ bật khi debug |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | PointCloud FLU, chỉ bật khi debug |

Camera health gate chặn ảnh quá tối, quá sáng hoặc thiếu texture. Khi perception
không đáng tin, `/uav/depth/free_space` là:

```text
[NaN, NaN, NaN, NaN, 0.0]
```

Sau lỗi, camera phải có 5 frame tốt liên tiếp trước khi depth được sử dụng lại.

## Build trên Jetson ROS 2 Foxy

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/uavcup_ws
colcon --log-base log_foxy build \
  --build-base build_foxy \
  --install-base install_foxy \
  --symlink-install \
  --packages-select px4_uavcup_perception px4_uavcup_control
source ~/uavcup_ws/install_foxy/setup.bash
```

Chỉ chạy perception:

```bash
ros2 launch px4_uavcup_perception perception_jetson.launch.py
```

Chạy perception cùng local controller shadow mode:

```bash
ros2 launch px4_uavcup_control jetson_perception_shadow.launch.py
```

Không chạy hai launch trên đồng thời vì chúng sẽ tranh `/dev/video0`.

Camera được center-crop `1280x720 -> 720x720`, sau đó resize `364x364` cho
TensorRT. Mọi output áp dụng:

```text
depth_corrected = depth_calibration_scale * raw + depth_calibration_bias_m
```

## Debug

```bash
ros2 launch px4_uavcup_perception perception_jetson.launch.py \
  publish_depth_image:=true \
  publish_visualization:=true \
  publish_pointcloud:=true
```

PointCloud dùng hệ FLU (`x` trước, `y` trái, `z` lên) và không được dùng cho
điều khiển khi intrinsic camera vẫn chỉ là giá trị xấp xỉ.

## Utility hiệu chuẩn camera

Node utility dưới đây chỉ publish ảnh camera; không thuộc đường runtime bay:

```bash
ros2 launch px4_uavcup_perception camera_calibration.launch.py
```

Nó không được chạy đồng thời với perception. Bảng A3 tùy chọn nằm tại
`calibration/checkerboard_9x6_30mm_a3.svg`.

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

## Raspberry Pi 5: ZipDepth + ArUco

Pipeline Pi dùng hai camera và hai namespace riêng:

```text
USB camera -> zipdepth_node (direct V4L2, không copy ảnh BGR qua DDS)
  └─ /uav/depth/zipdepth_raw
     └─ /camera/depth/image (sau hiệu chuẩn metric)

Pi Camera nhìn xuống -> /camera/down/image_raw + /camera/down/camera_info
  └─ aruco_detector_node -> /uav/aruco/ids + /uav/aruco/target_pose
     └─ aruco_landing_pid -> /aruco_land/cmd_vel
        └─ cmd_vel_to_px4 -> OffboardControlMode + TrajectorySetpoint
```

ZipDepth dùng checkpoint NPU được export ONNX 512x384 để giữ tỷ lệ 4:3
của USB camera. Đặt hai file ONNX
(graph và external weights) vào `/home/dolphiinn/models`; Docker mount thư mục
này read-only tại `/models`.

```bash
docker compose build ros
docker compose run --rm ros bash -lc \
  'source /opt/ros/humble/setup.bash && \
   colcon build --symlink-install --packages-select px4_msgs px4_uavcup_perception'
docker compose run --rm ros bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 launch px4_uavcup_perception perception_pi.launch.py'
```

`maximum_processing_rate_hz: 0.0` tắt giới hạn phần mềm, vì vậy node chạy
liên tục theo tốc độ inference thực tế. Topic raw vẫn là `32FC1` kích thước
512x384. Mặc định `zipdepth_node` mở `/dev/video0` trực tiếp để đường runtime
giống benchmark và tránh serialize ảnh BGR 640x480 qua DDS. Camera publisher
riêng trong launch được tắt; đặt `camera_device` rỗng và launch với
`front_usb_camera:=true` nếu cần quay lại subscriber mode. Khi pipeline đang
chạy, lưu một ảnh màu được tạo trực tiếp từ raw bằng:

```bash
docker compose run --rm ros bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   PYTHONPATH=/ros2_ws/src/px4_uavcup_perception:$PYTHONPATH \
   python3 src/px4_uavcup_perception/scripts/check_ros_depth_topic.py \
   --timeout 30 --save-color /ros2_ws/zipdepth_raw_color.png'
```

Ảnh `zipdepth_raw_color.png` chỉ là bản hiển thị percentile 2..98; đỏ là gần,
xanh là xa. Dữ liệu raw không bị sửa và vẫn không mang đơn vị mét trước hiệu
chuẩn metric.

`ZipDepth` cho inverse depth affine-invariant. Mặc định
`metric_calibration_enabled: false`, nên free-space được publish ở trạng thái
không hợp lệ cho tới khi fit `inverse_depth_scale` và
`inverse_depth_shift_per_m` trong không gian disparity bằng dữ liệu đo thật,
sau đó nghịch đảo kết quả sang mét.

Gateway `/uav/aruco/target_pose -> /fmu/in/landing_target_pose` cũng mặc định
`enabled: false`. Chỉ bật sau khi đã đo extrinsic camera-to-body, xác nhận đúng
hệ optical -> FRD -> NED, và bench-test với FC không gắn cánh quạt. Nếu ArUco
được dùng để external vision hoặc chỉ đọc ID nhiệm vụ thay vì precision landing,
phải dùng gateway khác; không tái sử dụng `LandingTargetPose` sai ngữ nghĩa.
Firmware PX4 v1.17 stock không liệt kê `LandingTargetPose` trong
`dds_topics.yaml`; precision landing qua topic này cần firmware tùy biến expose nó,
hoặc phải chọn bridge ROS/MAVLink khác.

Pipeline Offboard hiện tại không dùng `LandingTargetPose`. PID chỉ publish
`cmd_vel` body-FLU có giới hạn; adapter chuyển XY sang NED và tích phân lệnh Z
thành altitude setpoint có chặn `0.20..3.0 m`. Cả PID lẫn adapter đều tắt
mặc định, không tự arm/disarm. Mất marker quá `0.35 s` sẽ ra lệnh
zero và giữ độ cao.

PX4 v1.17 qua TELEM2 cần Micro XRCE-DDS Agent. Image agent được pin ở
v2.4.3 và chạy 921600 baud:

```bash
docker compose build xrce-agent
docker compose up -d xrce-agent
docker compose logs -f xrce-agent
```

Trước đó Pi phải enable UART hardware, disable Linux serial console và
reboot. Trên PX4: tắt MAVLink ở TELEM2, đặt `UXRCE_DDS_CFG=TELEM2` và
`SER_TEL2_BAUD=921600`.

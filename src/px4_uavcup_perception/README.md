# PX4 UAV Cup perception

Package này giữ pipeline perception cho Jetson Xavier NX và Raspberry Pi 5.
Đường Jetson dùng Depth Anything TensorRT; đường Pi dùng ZipDepth ONNX.

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
| `/uav/depth/relative_free_space` | `std_msgs/Float32MultiArray` | Relative clearance `[L,C,R,nearest,valid]`, chỉ dùng shadow |
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

Nó không được chạy đồng thời với perception. Bộ file in A4 nằm trong
`calibration/`: checkerboard 9x6 ô 25 mm và năm ArUco `DICT_5X5_50` ID 0-4.
Bảng A3 ô 30 mm cũ vẫn được giữ lại.

USB camera Pi được calibrate đúng mode runtime 640x480. Checkerboard A4 mặc
định có 9x6 góc trong và ô 0.025 m. Pi chỉ publish
`/camera/front/image_raw`; GUI
`cameracalibrator` chạy trên PC với `--no-service-check` vì publisher utility
không ghi calibration trực tiếp vào driver.

Trên Raspberry Pi có desktop, có thể chạy capture và GUI ngay trong cùng
container để ảnh raw không phải đi qua DDS/Wi-Fi:

```bash
xhost +si:localuser:root
docker compose run --rm ros bash -lc \
  'source /opt/ros/humble/setup.bash && \
   colcon build --symlink-install --packages-select px4_uavcup_perception && \
   source install/setup.bash && \
   ros2 launch px4_uavcup_perception camera_calibration_gui.launch.py'
```

Nếu dùng lại bảng A3 ô 30 mm, thêm `square_size_m:=0.03` vào lệnh launch.

Chỉ cấp quyền X11 cục bộ cho root trong container. Sau khi calibration xong,
có thể thu hồi bằng `xhost -si:localuser:root`.

## Raspberry Pi 5: ZipDepth + ArUco

Pipeline Pi dùng hai camera và hai namespace riêng:

```text
USB camera -> zipdepth_node (direct V4L2, không copy ảnh BGR qua DDS)
  ├─ /uav/depth/free_space -> local_controller_shadow
  ├─ /uav/depth/zipdepth_raw (debug tùy chọn)
  ├─ /uav/depth/visualization (debug tùy chọn)
  ├─ /camera/depth/image (debug, sau hiệu chuẩn metric)
  └─ /camera/depth/points (debug, sau hiệu chuẩn metric)

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
liên tục theo tốc độ inference thực tế. Mặc định `zipdepth_node` mở
`/dev/video0` trực tiếp và chỉ publish free-space/status nhỏ, giống pipeline
Jetson. Raw `32FC1` 512x384, visualization, metric depth và pointcloud đều là
debug output tùy chọn để không làm giảm FPS khi bay.

Để bật raw và visualization trong lúc kiểm tra:

```bash
ros2 launch px4_uavcup_perception perception_pi.launch.py \
  publish_raw_output:=true \
  publish_visualization:=true
```

Camera publisher riêng trong launch được tắt; đặt `camera_device` rỗng và
launch với `front_usb_camera:=true` nếu cần quay lại subscriber mode. Khi raw
đang bật, lưu một ảnh màu bằng:

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

Vì controller shadow dùng ngưỡng theo mét, khi calibration chưa bật thì
`/uav/depth/free_space` cố ý là `[NaN, NaN, NaN, NaN, 0.0]` và controller ở
`FAILSAFE`. Không bật `publish_metric_depth` hoặc `publish_pointcloud` trước
khi đã fit calibration và camera intrinsic. Sau hiệu chuẩn, chuỗi L/C/R dùng
lại trực tiếp `summarize_free_space` và `local_controller_shadow` của Jetson.

Thử nghiệm thực tế `0.5 m` và `1.0 m` xác nhận output ZipDepth không hỗ trợ
một global scale/shift cố định. Utility dưới đây chỉ dùng để thu dữ liệu và
kiểm tra giả thuyết calibration; không được lấy kết quả của một scene để bật
metric khi bay. Muốn metric cần một metric anchor ở từng frame hoặc đổi sang
model/sensor metric.

Thu calibration bằng một mặt phẳng đặt vuông góc camera, chiếm toàn bộ ROI giữa.
Khoảng cách được đo từ mặt kính/lens camera tới mặt phẳng. Khi launch đang bật
`publish_raw_output:=true`, thêm từng mẫu vào cùng dataset bằng:

```bash
python3 src/px4_uavcup_perception/scripts/calibrate_zipdepth_metric.py collect \
  --distance-m 1.0 --samples 20 \
  --output /ros2_ws/artifacts/zipdepth_metric_samples.json
```

Sau ít nhất ba khoảng cách khác nhau, fit calibration bằng:

```bash
python3 src/px4_uavcup_perception/scripts/calibrate_zipdepth_metric.py fit \
  --input /ros2_ws/artifacts/zipdepth_metric_samples.json
```

Khi chưa có metric anchor, node publish thêm
`/uav/depth/relative_free_space`. Giá trị 0..1 chỉ biểu diễn khoảng trống tương
đối trong cùng frame, không phải mét và không thể phát hiện an toàn một bức
tường phẳng chiếm toàn ảnh. Scene thiếu contrast tạo NaN để shadow controller
vào FAILSAFE.

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

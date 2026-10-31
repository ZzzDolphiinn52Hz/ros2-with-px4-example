# px4_uavcup_perception

Perception dành riêng cho Raspberry Pi 5 với hai camera CSI:

```text
CAM0 IMX219 -> front_camera.sock -> ZipDepth 512x384
CAM1 IMX500 -> down_camera.sock  -> ArUco pose
```

Picamera2 chạy trên Raspberry Pi OS host; ROS 2 Humble chạy trong container.
Raw frame đi qua Unix socket, không truyền ảnh qua Wi-Fi/DDS trong profile bay.

## Output chính

| Topic | Nội dung |
| --- | --- |
| `/uav/depth/relative_free_space` | `[left, center, right, nearest, valid]` tương đối |
| `/uav/depth/relative_corridor` | corridor tránh vật cản tương đối |
| `/uav/depth/status` | FPS, latency và trạng thái ZipDepth |
| `/uav/aruco/target_pose` | pose marker ID mục tiêu trong optical frame |
| `/uav/aruco/status` | marker count, target visibility và latency |

Raw depth, visualization, pointcloud, ArUco ID/rvec/tvec và debug image đều là
publisher tùy chọn để tránh giảm FPS khi bay.

## Chạy

Ưu tiên giao diện ở root repository:

```bash
./uavcup cameras
./uavcup aruco-test
./uavcup zipdepth-test
./uavcup perception-test
```

Các profile test không khởi động PID, PX4 bridge hoặc topic `/fmu/in/*`.

## Config và calibration

- `config/zipdepth.yaml`: model/socket/topic ZipDepth.
- `config/aruco.yaml`: dictionary, marker size và intrinsic IMX500.
- `config/pi_cameras.yaml`: raw IMX500 socket publisher để debug/calibration.
- `calibration/`: checkerboard và marker A4 có kích thước in xác định.

IMX500 hiện dùng 640x480 với calibration trong `aruco.yaml`. Detector giữ ảnh
gốc để calibration còn đúng; phép biến đổi camera-to-body nằm trong package
control. Nếu đổi sensor, mode ảnh hoặc gá camera thì phải calibrate lại.

Chạy GUI calibration IMX500 sau khi camera host server đã hoạt động:

```bash
xhost +si:localuser:root
docker compose run --rm -e DISPLAY="$DISPLAY" ros bash -lc \
  'source /opt/ros/humble/setup.bash && source install/setup.bash && \
   ros2 launch px4_uavcup_perception camera_calibration_gui.launch.py'
```

## ZipDepth metric

ZipDepth hiện được dùng ở relative mode. Không coi
`/uav/depth/relative_free_space` là mét. Metric depth/pointcloud chỉ được bật
khi `metric_calibration_enabled` có calibration đáng tin cậy cho camera thật.

Model mặc định nằm ngoài Git tại:

```text
~/models/zipdepth_base_npu_512x384.onnx
```

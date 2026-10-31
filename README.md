# Raspberry Pi 5 UAV perception and landing stack

Repository bàn giao cho Raspberry Pi 5, gồm hai pipeline đang dùng trên drone:

- CAM0 IMX219 phía trước: ZipDepth 512x384 và corridor/free-space tương đối.
- CAM1 IMX500 nhìn xuống: ArUco `DICT_5X5_100`, pose ID mục tiêu và landing PID.
- PX4 v1.17 qua TELEM2/UART: Micro XRCE-DDS Agent và Offboard adapter có khóa an toàn.

Các package SLAM/Gazebo, Nav2, Jetson/Depth Anything và node Offboard thử nghiệm
không thuộc sản phẩm Pi 5 đã được loại khỏi repository. Hai thư mục
`report_thesis2/` và `report_thesis_final/` được giữ làm tài liệu luận văn,
không tham gia build/runtime.

## Cấu trúc

```text
src/
  px4_msgs/                 PX4 v1.17 message definitions (Git submodule)
  px4_uavcup_perception/    camera, ArUco, ZipDepth
  px4_uavcup_control/       obstacle guidance và ArUco landing
  px4_uavcup_px4_bridge/    cmd_vel -> PX4 Offboard
  px4_uavcup_bringup/       launch ghép toàn hệ thống
Dockerfile                  hai target: ros-runtime và xrce-agent
docker-compose.yml          runtime container definitions
uavcup                      giao diện setup/test/run duy nhất
```

Config chỉ có một nguồn cho mỗi component:

- perception: `src/px4_uavcup_perception/config/`
- control: `src/px4_uavcup_control/config/`
- PX4 bridge: `src/px4_uavcup_px4_bridge/config/`
- bringup chỉ ghép node, không giữ bản sao config.

## Chuẩn bị máy Pi mới

Raspberry Pi OS 64-bit cần có Docker Compose, Git, Picamera2 và `rpicam-apps`.
Clone kèm submodule:

```bash
git clone --recurse-submodules <repository-url> ~/ros2_ws
cd ~/ros2_ws
./uavcup setup
./uavcup doctor
```

Nếu clone không dùng `--recurse-submodules`, `./uavcup setup` sẽ tự lấy
`px4_msgs` release 1.17.

Model ZipDepth không được commit vì kích thước/quyền phân phối. Chép model đã
kiểm thử của team vào:

```text
~/models/zipdepth_base_npu_512x384.onnx
```

Nếu model lưu nơi khác, export `UAV_MODELS_DIR=/duong/dan/models` trước khi
chạy Compose. Giữ các file external-data đi cùng ONNX trong cùng thư mục nếu
bản model sử dụng chúng.

## Test perception không cần FC/UART

Terminal 1 chạy hai camera trên Raspberry Pi OS host:

```bash
cd ~/ros2_ws
./uavcup cameras
```

Terminal 2 chọn một profile:

```bash
./uavcup aruco-test
./uavcup zipdepth-test
./uavcup perception-test
```

Ba profile này không tạo PX4 adapter, không request Offboard và không cần cắm
UART. `Ctrl+C` dừng profile, sau đó dừng camera server ở terminal 1.

## Kết nối PX4

Sau khi bench-test, khởi động agent và stack xe:

```bash
./uavcup agent
./uavcup agent-logs
```

Trong terminal khác:

```bash
./uavcup vehicle
```

`vehicle` khởi động perception, landing PID và adapter nhưng cả PID lẫn adapter
vẫn disabled. Nó không tự arm và không tự chuyển Offboard. Chỉ gọi các service
enable/request mode sau khi kiểm tra PX4 local position, attitude và tháo cánh
quạt trong bench test.

## Camera và gá lắp đã cấu hình

- CAM0: IMX219 phía trước, ảnh bị roll 180 độ; ZipDepth xoay lại trước inference.
- CAM1: IMX500 nhìn xuống, yaw 180 độ so với đầu drone.
- IMX500 nằm sau tâm cơ cấu gắp 0.10 m.
- Camera cao hơn điểm tiếp xúc cơ cấu gắp 0.15 m.
- ArUco marker mặc định: ID 0, cạnh đen ngoài 0.16 m.

Nếu thay camera, độ phân giải hoặc gá lắp, phải hiệu chuẩn và cập nhật config
trước khi dùng landing.

## Lệnh bảo trì

```bash
./uavcup unit-test
./uavcup shell
./uavcup stop
./uavcup --help
```

`./uavcup doctor` cũng báo model, camera, `/dev/ttyAMA0`, nhiệt độ và cờ
undervoltage của Pi. Không tiếp tục bay nếu `vcgencmd get_throttled` cho thấy
undervoltage hoặc hệ thống tự chuyển LED từ xanh sang đỏ.

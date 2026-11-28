# Raspberry Pi 5 UAV perception and landing stack

Repository bàn giao cho Raspberry Pi 5, gồm hai pipeline đang dùng trên drone:

- CAM0 IMX219 phía trước: ZipDepth 512x384 và corridor/free-space tương đối.
- CAM1 IMX500 nhìn xuống: ArUco `DICT_5X5_50`, pose ID mục tiêu và landing PID.
- PX4 v1.17 qua TELEM2/UART: Micro XRCE-DDS Agent và Offboard adapter có khóa an toàn.

Các package SLAM, Nav2, Jetson/Depth Anything và node Offboard thử nghiệm
không thuộc sản phẩm Pi 5 đã được loại khỏi repository. Package Gazebo/PX4
SITL mới là tùy chọn chỉ dành cho PC và không nằm trong build mặc định của Pi.
Hai thư mục
`report_thesis2/` và `report_thesis_final/` được giữ làm tài liệu luận văn,
không tham gia build/runtime.

## Làm việc với nhiều máy và AI

Repository dùng GitHub Spec Kit ở chế độ agent-neutral để Codex, Claude, Grok,
Antigravity hoặc người phát triển khác có thể tiếp tục công việc từ trạng thái
đã lưu trong Git. Điểm đọc đầu tiên là `AGENTS.md`; tiến độ hiện tại nằm tại
`docs/PROJECT_STATUS.md`, còn từng thay đổi được lưu dưới `specs/`.

Lấy context ngắn của checkout hiện tại mà không cần Docker/phần cứng:

```bash
./uavcup context
```

Quy trình cài đặt/chuyển agent và bàn giao được mô tả trong
`docs/AI_WORKFLOW.md`. Specs giúp thu hẹp source cần đọc nhưng không thay thế
việc kiểm tra code và test liên quan trước khi sửa.

Khi triển khai sang Raspberry Pi, bắt buộc theo `docs/RASPI_SYNC.md`: kiểm tra
SSH và working tree hai bên, push branch lên `origin`, pull trên Pi bằng
`--ff-only`, cập nhật submodule rồi so sánh commit SHA trước khi chạy test.

## Cấu trúc

```text
src/
  px4_msgs/                 PX4 v1.17 message definitions (Git submodule)
  px4_uavcup_perception/    camera, ArUco, ZipDepth
  px4_uavcup_control/       obstacle guidance và ArUco landing
  px4_uavcup_px4_bridge/    cmd_vel -> PX4 Offboard
  px4_uavcup_bringup/       launch ghép toàn hệ thống
  px4_uavcup_simulation/    Gazebo/PX4 SITL tùy chọn, chỉ dùng trên PC
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

## Profile ArUco-only cho Đồ án 2

Khi chỉ lắp một IMX500 tại CAM0, không cần IMX219 hoặc model ZipDepth. Terminal
1 chạy đúng một camera host server:

```bash
cd ~/ros2_ws
./uavcup aruco-camera
```

Để xem ID/rvec/tvec và debug image mà không kết nối PX4, Terminal 2 chạy:

```bash
./uavcup aruco-debug
```

Khi thay camera hoặc thay gá lắp, hiệu chuẩn lại IMX500 theo
`docs/IMX500_CALIBRATION.md`. Lệnh `./uavcup aruco-calibrate` lưu riêng log,
ảnh minh chứng và `ost.yaml` của từng phiên trong `artifacts/`.

Khi bench-test với FC, chạy XRCE Agent rồi khởi động stack landing tối giản:

```bash
./uavcup agent
./uavcup agent-logs       # tùy chọn, để xem log UART
./uavcup aruco-vehicle
```

`aruco-vehicle` chỉ chạy detector, landing PID và PX4 bridge. PID và bridge
đều disabled lúc khởi động; profile không tự arm hoặc request Offboard. Nó
không kiểm tra và không nạp model ZipDepth.

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

## Mô phỏng Gazebo/PX4 SITL trên PC

Giai đoạn 1–2 của môi trường mô phỏng thay UAV, camera và UART thật bằng PX4
SITL, Gazebo Harmonic và camera nhìn xuống. Thuật toán nhận dạng vẫn dùng nguyên
node ArUco của hệ thống thật; marker mô phỏng là `DICT_5X5_50`, ID 0, cạnh
80 mm. Các profile này không tự arm, không request Offboard và không khởi động
PID hay PX4 bridge.

Kiểm tra và build một lần:

```bash
./uavcup sim-doctor
./uavcup sim-setup
```

Sau đó chạy Gazebo, XRCE Agent, PX4 SITL và detector ở bốn terminal riêng.
Toàn bộ trình tự cùng kết quả mong đợi nằm trong
[`docs/GAZEBO_ARUCO_SITL.md`](docs/GAZEBO_ARUCO_SITL.md).

Quy trình free-flight tiếp theo — mở QGC, arm/takeoff thủ công, tiếp cận marker,
chuyển Offboard qua từng gate và ghi rosbag landing — nằm trong
[`docs/GAZEBO_ARUCO_LANDING_E2E.md`](docs/GAZEBO_ARUCO_LANDING_E2E.md).

## Camera và gá lắp đã cấu hình

- Profile ArUco-only: một IMX500 tại CAM0.
- Profile hai camera: CAM0 IMX219 cho ZipDepth và CAM1 IMX500 cho ArUco.
- IMX500 nhìn xuống, mép trên ảnh hướng về mũi drone.
- IMX500 nằm cùng tâm XY với cơ cấu gắp.
- Camera cao hơn điểm tiếp xúc cơ cấu gắp 0.15 m.
- ArUco marker mặc định: `DICT_5X5_50`, ID 0, cạnh đen ngoài 0.08 m.

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

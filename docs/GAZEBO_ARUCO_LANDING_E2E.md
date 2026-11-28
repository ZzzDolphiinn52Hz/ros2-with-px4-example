# Gazebo ArUco Landing End-to-End

Quy trình này mô phỏng đúng luồng vận hành dự kiến trên UAV thật:

1. QGroundControl kết nối PX4, arm, takeoff và đưa UAV đến gần marker.
2. ROS 2 chỉ tiếp quản sau khi operator kiểm tra PX4 và ArUco.
3. Adapter, Offboard, PID và landing được kích hoạt bằng bốn thao tác riêng.
4. PID căn ngang, hạ theo marker, chốt XY/yaw ở khoảng 0,30 m AGL, sau đó hạ
   mù thẳng đứng đến khi PX4 báo landed và mới gửi yêu cầu disarm.

Không lệnh nào dưới đây tự arm. Không launch file nào tự enable adapter/PID hay
tự request Offboard.

## 1. Chuẩn bị một lần

```bash
cd ~/ros2_ws
./uavcup sim-doctor
./uavcup sim-setup
```

World `aruco_landing` không có giá đỡ vô hình. X500 xuất phát trên pad màu xanh
tại `x=-1 m`; marker `DICT_5X5_50`, ID 0, cạnh 80 mm nằm trên kiện hàng tại
`x=+1 m`. UAV phải được cất cánh và đưa từ pad đến vùng marker như ngoài thực
tế.

## 2. Khởi động sáu terminal

### Terminal 1 — Gazebo free-flight

```bash
cd ~/ros2_ws
./uavcup sim-landing-gazebo
```

### Terminal 2 — Micro XRCE-DDS Agent

```bash
cd ~/ros2_ws
./uavcup sim-landing-agent
```

### Terminal 3 — PX4 SITL

Chỉ chạy sau khi world Gazebo đã xuất hiện:

```bash
cd ~/ros2_ws
./uavcup sim-landing-px4
```

### Terminal 4 — QGroundControl

```bash
cd ~/ros2_ws
./uavcup sim-qgc
```

Chờ QGC báo `Ready To Fly`. QGC là đường duy nhất trong quy trình này dùng để
arm, takeoff và điều khiển tiếp cận marker.
Chọn đúng X500 SITL trong QGC. ROS của bài thử dùng domain 42 và XRCE UDP 8890.

### Terminal 5 — pipeline ROS 2 đầy đủ

```bash
cd ~/ros2_ws
./uavcup sim-landing-stack
```

Launch này chạy camera bridge, detector, landing PID và PX4 adapter. Hai node
điều khiển phải báo `disabled`/`IDLE`; chưa có setpoint nào được phát khi
adapter còn disabled.

### Terminal 6 — rosbag minh chứng

```bash
cd ~/ros2_ws
./uavcup sim-landing-record
```

Rosbag được lưu dưới `artifacts/gazebo_aruco_landing/`. Dừng bằng `Ctrl+C` sau
khi hạ cánh để metadata được đóng đúng cách.

## 3. Kiểm tra trước khi cất cánh

Trong một terminal thao tác riêng:

```bash
cd ~/ros2_ws
./uavcup sim-landing-check startup
```

Chỉ tiếp tục khi local XY/Z, attitude, preflight và failsafe đều PASS. Ở bước
này UAV vẫn disarmed và marker chưa cần xuất hiện.

## 4. Arm, takeoff và đưa UAV đến marker bằng QGC

Trên QGC:

1. Arm thủ công.
2. Chọn Takeoff. Nếu thanh độ cao của QGC bắt đầu ở 2 m, cất cánh ở mức đó rồi
   điều chỉnh độ cao khi đã giữ vị trí.
3. Dùng joystick hoặc [virtual joystick của QGC](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/settings_view/virtual_joystick.html)
   và nhìn Gazebo để bay từ pad xanh đến phía trên marker. Hai vị trí chỉ cách
   nhau 2 m nên khó căn chính xác bằng bản đồ vệ tinh.
4. Giữ UAV trong Hold/POSCTL ở độ cao khoảng `0,8–1,2 m`.

Khi UAV đã gần marker, chạy:

```bash
./uavcup sim-landing-check takeover
```

Tất cả mục phải PASS, đặc biệt:

- `vehicle armed`;
- `vehicle airborne`;
- `ArUco target visible`;
- `ArUco target is ID 0`.

Nếu marker chưa visible, tiếp tục chỉnh UAV bằng QGC; không enable adapter.
Mục `ArUco capture fresh` cũng phải PASS. Readiness so timestamp chụp với
đồng hồ Gazebo và yêu cầu đồng hồ tiếp tục tiến. Nếu reset world, khởi động
lại stack mô phỏng trước khi thử tiếp.

## 5. Chuyển quyền điều khiển theo từng gate

### Gate 1 — enable adapter, vẫn giữ mode hiện tại

```bash
./uavcup sim-adapter-enable
```

Response phải `success=True` và target altitude phải xấp xỉ độ cao hiện tại.
Chờ ít nhất 2 giây để PX4 nhận heartbeat/setpoint zero. QGC lúc này vẫn ở
Hold/POSCTL.

### Gate 2 — request Offboard

```bash
./uavcup sim-request-offboard
```

Kiểm tra log adapter có ACK `ACCEPTED`, QGC hiển thị `Offboard`, rồi chạy:

```bash
./uavcup sim-landing-check offboard
```

Không tiếp tục nếu `PX4 nav_state is OFFBOARD` chưa PASS.

### Gate 3 — enable PID, nhưng chưa bắt đầu landing

```bash
./uavcup sim-pid-enable
```

Response phải ghi `waiting for explicit start_landing`. Landing status vẫn là
`enabled=true`, `state=idle`; enable đơn lẻ không được tạo chuyển động.

### Gate 4 — bắt đầu landing

```bash
./uavcup sim-landing-start
```

State mong đợi:

```text
SEARCH -> LOCK -> VISUAL_APPROACH -> BLIND_FINAL_DESCENT -> COMPLETE
```

Ở `VISUAL_APPROACH`, PID căn XY và giảm cao. Khi AGL khoảng 0,30 m và sai số
ngang đủ nhỏ trong đủ số frame, controller gọi latch XY/yaw. Từ đó marker
không còn bắt buộc. Khi PX4 xác nhận landed, bridge mới chấp nhận yêu cầu
disarm.

Theo dõi trực tiếp trong terminal đã source ROS và workspace:

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /uav/aruco/landing_status
```

hoặc:

```bash
ros2 topic echo /fmu/out/vehicle_status_v1
```

## 6. Abort và trả quyền điều khiển

Nếu kết quả không đúng, thực hiện từng lệnh theo thứ tự:

```bash
./uavcup sim-landing-abort
./uavcup sim-request-posctl
```

`abort` đưa `cmd_vel` về 0. `request_posctl` gửi yêu cầu chuyển mode. Nếu
PX4 từ chối, dùng QGC chọn Hold hoặc chế độ điều khiển phù hợp. Xác nhận QGC
đã rời Offboard rồi mới dừng controller và heartbeat bằng hai lệnh dưới đây.

```bash
./uavcup sim-pid-disable
./uavcup sim-adapter-disable
```

## 7. Kết thúc

1. Dừng rosbag bằng `Ctrl+C`.
2. Xác nhận UAV disarmed trong QGC.
3. Dừng Terminal 5, 3, 2, 1 bằng `Ctrl+C`.
4. Đóng QGC.

SITL pass không phải là phê duyệt bay thật. Trước hardware flight vẫn phải
thực hiện bench test tháo cánh, kiểm tra nguồn, UART, local position, failsafe
và từng gate service trên UAV thật.

# PX4 UAV Cup local control

Package này chứa local controller không phụ thuộc Nav2/SLAM, cùng PID hạ
cánh ArUco. PID chỉ publish `/aruco_land/cmd_vel`; nó không tạo publisher
PX4 `/fmu/in/*`. Phiên đầu chạy
hoàn toàn ở **shadow mode**: đọc `/uav/depth/free_space`, nhưng chỉ publish vận
tốc đề xuất và không tạo publisher PX4 `/fmu/in/*`.

PID ArUco bù lever-arm của camera trước khi tính sai số ngang. Trên bộ gá hiện
tại, IMX500 nằm sau cơ cấu gắp ở tâm thân 10 cm nên
`camera_position_body_flu_m: [-0.10, 0.0, 0.0]`. Khoảng cách hạ cuối vẫn được
đo dọc trục quang camera và điều chỉnh riêng bằng `final_marker_distance_m`.

```bash
ros2 launch px4_uavcup_control shadow_controller.launch.py
```

Trên Jetson có thể chạy perception và shadow controller cùng một lệnh:

```bash
ros2 launch px4_uavcup_control jetson_perception_shadow.launch.py
```

Trên Pi, ZipDepth tìm một corridor 2D đủ rộng/cao theo pixel và publish tâm vùng
trống tương đối. Launch dưới đây nối topic đó vào `corridor_controller_shadow`.
Controller xuất tư vấn body-FLU ba trục: `x` tiến, `y` trái và `z` lên. Launch
không khởi tạo ArUco, PX4 adapter hoặc topic lệnh `/fmu/in/*`:

```bash
ros2 launch px4_uavcup_control pi_zipdepth_shadow.launch.py
```

Bật ảnh depth debug có hình chữ nhật và dấu tâm corridor được chọn:

```bash
ros2 launch px4_uavcup_control pi_zipdepth_shadow.launch.py \
  publish_visualization:=true
```

Relative mode không suy ra được khoảng cách phanh hay kích thước khe theo mét.
Ảnh invalid, scene thiếu contrast hoặc không có cửa sổ đạt độ thoáng tối thiểu
luôn chuyển sang `FAILSAFE/BRAKE`. Output này chỉ dùng để bench-test.

Không chạy thêm `perception_jetson.launch.py` riêng trong trường hợp này, vì
hai tiến trình perception sẽ tranh `/dev/video0`.

Output:

- `/uav/local_controller/advisory_velocity` (`geometry_msgs/TwistStamped`),
  hệ body FLU: `x` tiến, `y` trái, `z` lên.
- `/uav/local_controller/state` (`std_msgs/String`).
- `/uav/local_controller/status` (`diagnostic_msgs/DiagnosticArray`).

Các trạng thái corridor: `TRACK_CORRIDOR`, `CORRIDOR_CENTERED`, `BRAKE`,
`FAILSAFE`. Mất depth, depth invalid hoặc quá timeout luôn tạo advisory bằng
zero. Target ảnh được EMA và giới hạn bước nhảy để giảm đổi hướng liên tục.

## PX4 bench gate

`pi_corridor_px4_bench.launch.py` thêm hai khóa độc lập giữa corridor và PX4:
`corridor_velocity_gate` cùng `cmd_vel_to_px4`. Cả hai luôn khởi động ở trạng
thái disabled, không tự arm và không tự request Offboard. Gate quên lệnh cũ
mỗi lần enable/disable, kiểm tra frame `base_link`, giới hạn vận tốc và phát
zero nếu advisory stale. Trong bench launch, trục Z corridor bị khóa.

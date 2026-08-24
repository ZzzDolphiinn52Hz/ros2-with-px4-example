# PX4 UAV Cup local control

Package này chứa local controller không phụ thuộc Nav2/SLAM. Phiên đầu chạy
hoàn toàn ở **shadow mode**: đọc `/uav/depth/free_space`, nhưng chỉ publish vận
tốc đề xuất và không tạo publisher PX4 `/fmu/in/*`.

```bash
ros2 launch px4_uavcup_control shadow_controller.launch.py
```

Trên Jetson có thể chạy perception và shadow controller cùng một lệnh:

```bash
ros2 launch px4_uavcup_control jetson_perception_shadow.launch.py
```

Trên Pi, ZipDepth chỉ cung cấp relative clearance. Launch dưới đây nối topic
riêng vào shadow controller. Launch chỉ tạo `zipdepth_node` và
`local_controller_shadow`; không khởi tạo ArUco, PX4 adapter hoặc topic lệnh
`/fmu/in/*`:

```bash
ros2 launch px4_uavcup_control pi_zipdepth_shadow.launch.py
```

Không chạy thêm `perception_jetson.launch.py` riêng trong trường hợp này, vì
hai tiến trình perception sẽ tranh `/dev/video0`.

Output:

- `/uav/local_controller/advisory_velocity` (`geometry_msgs/TwistStamped`),
  hệ body FLU: `x` tiến, `y` trái.
- `/uav/local_controller/state` (`std_msgs/String`).
- `/uav/local_controller/status` (`diagnostic_msgs/DiagnosticArray`).

Các trạng thái: `CLEAR`, `AVOID_LEFT`, `AVOID_RIGHT`, `BRAKE`, `FAILSAFE`.
Mất depth, depth invalid hoặc quá timeout luôn tạo advisory bằng zero.

Ngưỡng shadow hiện tại: emergency `0.35 m`, vào tránh dưới `0.45 m`, và chỉ
chuyển về `CLEAR` khi khoảng trống đạt ít nhất `0.50 m`.

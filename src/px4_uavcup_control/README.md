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

Không chạy thêm `perception_jetson.launch.py` riêng trong trường hợp này, vì
hai tiến trình perception sẽ tranh `/dev/video0`.

Output:

- `/uav/local_controller/advisory_velocity` (`geometry_msgs/TwistStamped`),
  hệ body FLU: `x` tiến, `y` trái.
- `/uav/local_controller/state` (`std_msgs/String`).
- `/uav/local_controller/status` (`diagnostic_msgs/DiagnosticArray`).

Các trạng thái: `CLEAR`, `AVOID_LEFT`, `AVOID_RIGHT`, `BRAKE`, `FAILSAFE`.
Mất depth, depth invalid hoặc quá timeout luôn tạo advisory bằng zero.

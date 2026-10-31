# px4_uavcup_control

Hai nhóm controller dùng trên Raspberry Pi 5:

- `obstacle/`: đọc relative corridor/free-space từ ZipDepth và tạo advisory
  velocity. Shadow/gate mặc định không điều khiển PX4 trực tiếp.
- `landing/`: PID ArUco, bù lever arm camera và chuyển sang blind descent sau
  khi căn giữa ở độ cao chuyển pha.

Landing PID chỉ publish `/aruco_land/cmd_vel`. Việc đổi hệ tọa độ và publish
PX4 setpoint do `px4_uavcup_px4_bridge` đảm nhiệm.

Config duy nhất của landing là `config/landing.yaml`. Gá hiện tại:

- IMX500 yaw 180 độ so với đầu drone.
- Camera nằm sau tâm cơ cấu gắp 0.10 m.
- Camera cao hơn điểm tiếp xúc cơ cấu gắp 0.15 m.
- Chuyển blind descent ở gripper AGL xấp xỉ 0.30 m khi sai số XY đủ nhỏ.

PID khởi động disabled, không arm và không tự request Offboard. Mất marker ở
visual phase tạo lệnh zero. Blind phase chỉ bắt đầu sau khi PX4 adapter xác nhận
đã latch XY/yaw.

Test ZipDepth shadow không cần FC:

```bash
ros2 launch px4_uavcup_control pi_zipdepth_shadow.launch.py
```

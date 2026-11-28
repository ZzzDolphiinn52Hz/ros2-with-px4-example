# px4_uavcup_control

Hai nhóm controller dùng trên Raspberry Pi 5:

- `obstacle/`: đọc relative corridor/free-space từ ZipDepth và tạo advisory
  velocity. Shadow/gate mặc định không điều khiển PX4 trực tiếp.
- `landing/`: PID ArUco, bù lever arm camera và chuyển sang blind descent sau
  khi căn giữa ở độ cao chuyển pha.

Landing PID chỉ publish `/aruco_land/cmd_vel`. Việc đổi hệ tọa độ và publish
PX4 setpoint do `px4_uavcup_px4_bridge` đảm nhiệm.

Config duy nhất của landing là `config/landing.yaml`. Gá hiện tại:

- IMX500 nhìn xuống, mép trên ảnh hướng về mũi drone.
- Camera nằm cùng tâm XY với cơ cấu gắp.
- Camera cao hơn điểm tiếp xúc cơ cấu gắp 0.15 m.
- Chuyển blind descent ở gripper AGL xấp xỉ 0.30 m khi sai số XY đủ nhỏ.

PID khởi động disabled, không arm và không tự request Offboard. Service
`enable` chỉ chuẩn bị controller ở trạng thái `IDLE`; operator phải gọi riêng
`start_landing` thì state machine mới vào `SEARCH` và bắt đầu xuất lệnh. Mất
marker ở visual phase tạo lệnh zero. Blind phase chỉ bắt đầu sau khi PX4
adapter xác nhận đã latch XY/yaw.

Test ZipDepth shadow không cần FC:

```bash
ros2 launch px4_uavcup_control pi_zipdepth_shadow.launch.py
```

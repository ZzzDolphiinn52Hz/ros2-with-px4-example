# px4_uavcup_px4_bridge

Package chỉ giữ adapter đang dùng:

```text
/aruco_land/cmd_vel -> cmd_vel_to_px4 -> PX4 OffboardControlMode
                                      -> PX4 TrajectorySetpoint
```

Adapter mặc định disabled và không tự arm. Enable bị từ chối nếu PX4 local
position (`xy_valid`, `z_valid`) hoặc attitude không hợp lệ. Request Offboard là
một service riêng, chỉ hoạt động sau khi adapter đã phát heartbeat/setpoint.

Các service chính:

- `/cmd_vel_to_px4/enable`
- `/cmd_vel_to_px4/request_offboard`
- `/cmd_vel_to_px4/request_posctl`
- `/cmd_vel_to_px4/latch_xy_yaw`
- `/cmd_vel_to_px4/release_xy_yaw`
- `/cmd_vel_to_px4/request_disarm`

Disarm chỉ được gửi khi PX4 `VehicleLandDetected.landed == true`. Config duy
nhất nằm tại `config/px4_bridge.yaml`.

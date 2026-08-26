# PX4 UAV Cup PX4 bridge

Package này chỉ chuyển lệnh ROS sang giao tiếp PX4. Nó không cảm nhận môi
trường và không tính vận tốc điều khiển.

| Executable | Vai trò |
| --- | --- |
| `cmd_vel_to_px4` | `geometry_msgs/Twist` → Offboard heartbeat + trajectory setpoint |
| `aruco_to_px4_landing_target` | ArUco pose → `LandingTargetPose` |

Cả hai node mặc định disabled. Chúng không arm, không đổi mode trừ khi gọi
service `request_offboard` trên `cmd_vel_to_px4`.

```bash
ros2 run px4_uavcup_px4_bridge cmd_vel_to_px4
ros2 run px4_uavcup_px4_bridge aruco_to_px4_landing_target
```

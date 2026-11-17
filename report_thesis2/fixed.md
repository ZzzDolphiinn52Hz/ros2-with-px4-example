# Việc cần sửa trước khi hạ cánh ArUco

Ghi ngày 2026-11-17 sau khi kiểm tra Pi `raspi` (`192.168.1.120`) và PX4 đang nối XRCE. Chưa sửa code, chưa bật adapter, chưa xin Offboard, chưa arm.

## 1. Sửa tên topic vị trí local

PX4 đang phát `/fmu/out/vehicle_local_position`. Bốn chỗ trong repo vẫn subscribe `/fmu/out/vehicle_local_position_v1`:

- `src/px4_uavcup_px4_bridge/config/px4_bridge.yaml`
- `src/px4_uavcup_px4_bridge/px4_uavcup_px4_bridge/cmd_vel_to_px4.py` (giá trị mặc định)
- `src/px4_uavcup_control/config/landing.yaml`
- `src/px4_uavcup_control/px4_uavcup_control/landing/aruco_pid_node.py` (giá trị mặc định)

Cùng bản tin đã đọc được các trường `xy_valid`, `z_valid`, `dist_bottom`, đúng kiểu `VehicleLocalPosition`. Chỉ tên topic sai. Để vậy thì `/cmd_vel_to_px4/enable` từ chối vì không thấy vị trí, và PID không nhận `dist_bottom`.

Các topic sau đã khớp, không đổi:

- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_land_detected`

## 2. Không gộp thành một service tự xin Offboard

`/aruco_landing_pid/start_landing` chỉ đưa PID sang SEARCH và phát `/aruco_land/cmd_vel`. Service này không xin Offboard. Giữ nguyên thiết kế đó.

Khi máy đã bay trên mã và preflight đạt, chuỗi gọi vẫn là:

1. `/cmd_vel_to_px4/enable` với `data: true`
2. `/cmd_vel_to_px4/request_offboard`
3. Xác nhận `nav_state` là 14
4. `/aruco_landing_pid/start_landing`

## 3. Chưa chạy stack ArUco trên Pi

Lúc kiểm tra chỉ có container `xrce-agent`. Không có camera server, không có `run/down_camera.sock`, không có `aruco_detector_node`, `aruco_landing_pid`, `cmd_vel_to_px4`, nên các service hạ cánh chưa tồn tại.

Camera trên Pi là một IMX500 ở cổng 0. Trước khi bay cần chạy `./uavcup aruco-camera`, rồi `./uavcup aruco-vehicle`. Hai lệnh này không tự enable PID, không arm và không tự xin Offboard.

## 4. Session XRCE vẫn bị tạo lại

Một client key `0x00000001`. Log agent có nhiều lần `session re-established`, các lần gần lúc kiểm tra cách nhau khoảng 2–5 phút. Mỗi lần đó `/fmu/out` dừng một nhịp. Chưa có lỗi agent nào chỉ ra nguyên nhân, nên chưa sửa baud hay agent. Đây là rủi ro nếu đang hạ cánh.

## 5. Trạng thái PX4 lúc kiểm tra, chưa đủ điều kiện gọi Offboard

- `arming_state` 1, disarm
- đã tiếp đất, `nav_state` 4 (AUTO_LOITER)
- `failsafe` false
- `pre_flight_checks_pass` false, nên `request_offboard` sẽ từ chối
- vị trí local `xy_valid` và `z_valid`, nhưng `x` khoảng -19.6 m, `z` khoảng -2.87 m trong khi `dist_bottom` khoảng 0.30 m
- `heading_good_for_control` false

Không đổi tham số PX4 và không reboot từ ghi chú này.

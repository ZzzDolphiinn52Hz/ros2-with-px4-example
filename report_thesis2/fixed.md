# Việc cần sửa trước khi hạ cánh ArUco

Ghi ngày 2026-11-17 sau khi kiểm tra Pi `raspi` (`192.168.1.120`) và PX4 đang nối XRCE. Chưa sửa code, chưa bật adapter, chưa xin Offboard, chưa arm.

## Kết quả rà soát ngày 2026-11-20

- Mục 1 là lỗi cấu hình thật và đã được sửa trong bốn vị trí nêu dưới đây.
- Mục 2 là chủ đích an toàn của thiết kế, không thay đổi code.
- Mục 3 được xác nhận lại trên `raspi`: hiện chỉ có container `xrce-agent`, không có socket camera hoặc stack ArUco.
- Mục 4 được xác nhận từ log agent và vẫn là blocker trước khi bay. Log ngày 17-11 có nhiều lần `session re-established`, có thời điểm chỉ cách nhau vài chục giây. Chưa đủ bằng chứng để tự ý đổi baudrate hoặc tham số PX4.
- Mục 5 chỉ là ảnh chụp trạng thái tại ngày 17-11. Phải đọc lại toàn bộ điều kiện ngay trước lần bench test hoặc bay tiếp theo; không dùng các giá trị cũ để kết luận FC hiện đã sẵn sàng.

## 1. Sửa tên topic vị trí local

PX4 đang phát `/fmu/out/vehicle_local_position`. Bốn chỗ trong repo trước đây subscribe nhầm `/fmu/out/vehicle_local_position_v1` và đã được đổi sang topic không có hậu tố `_v1`:

- `src/px4_uavcup_px4_bridge/config/px4_bridge.yaml`
- `src/px4_uavcup_px4_bridge/px4_uavcup_px4_bridge/cmd_vel_to_px4.py` (giá trị mặc định)
- `src/px4_uavcup_control/config/landing.yaml`
- `src/px4_uavcup_control/px4_uavcup_control/landing/aruco_pid_node.py` (giá trị mặc định)

Cùng bản tin đã đọc được các trường `xy_valid`, `z_valid`, `dist_bottom`, đúng kiểu `VehicleLocalPosition`. Chỉ tên topic sai. Trước khi sửa, `/cmd_vel_to_px4/enable` từ chối vì không thấy vị trí và PID không nhận `dist_bottom`.

Các topic sau đã khớp, không đổi:

- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_land_detected`

## 2. Không gộp thành một service tự xin Offboard

`/aruco_landing_pid/start_landing` chỉ đưa PID sang SEARCH và phát `/aruco_land/cmd_vel`. Service này không xin Offboard. Giữ nguyên thiết kế đó.

Khi máy đã bay trên mã và preflight đạt, chuỗi gọi vẫn là:

1. `/cmd_vel_to_px4/enable` với `data: true`
2. Chờ ít nhất hơn 1 giây để bridge phát ổn định `OffboardControlMode` và setpoint ở 20 Hz
3. `/cmd_vel_to_px4/request_offboard`
4. Xác nhận ACK được chấp nhận và `nav_state` là 14
5. `/aruco_landing_pid/start_landing`

Không bỏ qua bước chờ heartbeat: [PX4 v1.17 Offboard Mode](https://docs.px4.io/v1.17/en/flight_modes/offboard) yêu cầu proof-of-life lớn hơn 2 Hz trong hơn 1 giây trước khi chuyển sang Offboard.

## 3. Chưa chạy stack ArUco trên Pi

Lúc kiểm tra chỉ có container `xrce-agent`. Không có camera server, không có `run/down_camera.sock`, không có `aruco_detector_node`, `aruco_landing_pid`, `cmd_vel_to_px4`, nên các service hạ cánh chưa tồn tại.

Camera trên Pi là một IMX500 ở cổng 0. Trước khi bay cần chạy `./uavcup aruco-camera`, rồi `./uavcup aruco-vehicle`. Hai lệnh này không tự enable PID, không arm và không tự xin Offboard.

## 4. Session XRCE vẫn bị tạo lại

**Trạng thái: chưa giải quyết; không bay Offboard cho đến khi đường truyền ổn định.**

Một client key `0x00000001`. Log agent có nhiều lần `session re-established`; khoảng cách giữa các lần thay đổi từ vài chục giây đến vài phút. Mỗi lần đó `/fmu/out` dừng một nhịp. Chưa có lỗi agent nào chỉ ra nguyên nhân, nên chưa sửa baud hay agent. Đây là rủi ro nếu đang hạ cánh.

## 5. Trạng thái PX4 lúc kiểm tra, chưa đủ điều kiện gọi Offboard

**Trạng thái: dữ liệu lịch sử ngày 17-11, phải đo lại trước lần thử tiếp theo.**

- `arming_state` 1, disarm
- đã tiếp đất, `nav_state` 4 (AUTO_LOITER)
- `failsafe` false
- `pre_flight_checks_pass` false, nên `request_offboard` sẽ từ chối
- vị trí local `xy_valid` và `z_valid`, nhưng `x` khoảng -19.6 m, `z` khoảng -2.87 m trong khi `dist_bottom` khoảng 0.30 m
- `heading_good_for_control` false

Giá trị `x`, `z` local và `dist_bottom` không bắt buộc phải bằng nhau: `x`, `z` được tính theo gốc local NED của EKF, còn `dist_bottom` là khoảng cách đến mặt đất. Điểm chặn Offboard trong snapshot này là `pre_flight_checks_pass=false`; đồng thời `heading_good_for_control=false` là cảnh báo phải xử lý trước khi cho phép bridge dùng yaw để đổi vận tốc body-FLU sang NED.

Không đổi tham số PX4 và không reboot từ ghi chú này.

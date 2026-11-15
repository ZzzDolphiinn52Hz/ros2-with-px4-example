# Hiệu chuẩn nội tại IMX500 cho ArUco landing

Quy trình này hiệu chuẩn IMX500 ở đúng chế độ sử dụng khi bay: ảnh `640x480`,
mô hình camera pinhole và checkerboard A4 `9x6` góc trong, cạnh mỗi ô `25 mm`.

## 1. Chuẩn bị

- Chỉ gắn IMX500 vào CAM0 và cố định camera như khi bay.
- Camera nhìn thẳng xuống; mép trên ảnh hướng về mũi UAV.
- Dùng bản in
  `src/px4_uavcup_perception/calibration/checkerboard_9x6_25mm_a4.pdf`.
- Khi in chọn **Actual size / 100%**, không chọn Fit to page.
- Dùng thước kiểm tra một cạnh ô đúng `25 mm`; dán tờ in lên mặt phẳng cứng.
- Cấp đủ sáng, tránh phản chiếu và motion blur.

Không chạy `aruco-debug`, `aruco-vehicle` hoặc chương trình camera khác trong
lúc hiệu chuẩn.

## 2. Chạy camera

Mở Terminal 1 trên desktop Raspberry Pi:

```bash
cd ~/ros2_ws
./uavcup aruco-camera
```

Log đúng phải chứa IMX500 tại index 0, socket `run/down_camera.sock` và độ phân
giải `640x480`.

## 3. Chạy GUI hiệu chuẩn

Mở Terminal 2:

```bash
cd ~/ros2_ws
./uavcup aruco-calibrate
```

Script tự tạo thư mục:

```text
artifacts/imx500_calibration/YYYYMMDD_HHMMSS/
```

Trong cửa sổ GUI, nhấn phím `s` tại các thời điểm cần lưu ảnh minh chứng. Nên
có ít nhất các ảnh sau cho báo cáo:

1. Checkerboard được nhận dạng ở giữa ảnh.
2. Checkerboard ở bốn vùng trái, phải, trên và dưới.
3. Checkerboard ở gần và xa để thay đổi thanh **Size**.
4. Checkerboard nghiêng quanh hai trục để tăng thanh **Skew**.
5. Bốn thanh X, Y, Size, Skew đã đủ và nút **CALIBRATE** sáng.
6. Ảnh đã rectify sau khi calibration hoàn tất.

Di chuyển chậm và giữ bảng đứng yên khoảng một giây ở mỗi tư thế. Không chỉ
xoay checkerboard trong cùng một vị trí; cần phủ đều toàn bộ trường nhìn và có
nhiều góc nghiêng khác nhau.

## 4. Tính và lưu kết quả

Khi nút **CALIBRATE** sáng:

1. Bấm **CALIBRATE** đúng một lần.
2. Chờ tới khi cửa sổ phản hồi trở lại; trên Pi có thể mất khoảng một phút.
3. Kiểm tra các đường thẳng trong ảnh rectify không bị cong bất thường.
4. Nhấn `s` để lưu ảnh kết quả.
5. Bấm **SAVE** để tạo archive. Không bấm **COMMIT**.
6. Quay về Terminal 2 và nhấn `Ctrl+C`.

Script sẽ giữ lại:

```text
terminal.log                  toàn bộ log và ma trận do calibrator in ra
calibrationdata.tar.gz        dữ liệu gốc của ROS camera_calibration
extracted/ost.yaml            thông số dùng để cập nhật cấu hình
extracted/left-*.png          các ảnh checkerboard được chọn
screenshots/dump*.png         ảnh GUI tạo bằng phím s
session.txt                   mode ảnh và kích thước checkerboard
```

## 5. Đọc thông số

Mở file `extracted/ost.yaml`. Các trường cần lấy là:

```yaml
camera_matrix:
  data: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
distortion_coefficients:
  data: [k1, k2, p1, p2, k3]
```

Gửi lại file `ost.yaml` và `terminal.log` để cập nhật đồng nhất
`config/aruco.yaml` và `config/pi_cameras.yaml`. Không tự chép projection
matrix vào camera matrix.

## 6. Tiêu chí chấp nhận

- Kết quả đúng độ phân giải `640x480`.
- `fx`, `fy` dương và có cùng bậc độ lớn.
- `cx`, `cy` nằm bên trong ảnh.
- Không có `NaN` hoặc `Inf`.
- Ảnh rectify hợp lý và đường thẳng gần rìa ảnh thẳng hơn ảnh raw.
- Sau khi cập nhật config phải kiểm tra lại `tvec.x/y/z` bằng marker có kích
  thước đã đo chính xác trước khi bật PID.

Các file trong `artifacts/` là dữ liệu thí nghiệm và không tự động được commit
vào Git. Hãy sao lưu riêng thư mục phiên được dùng trong báo cáo.

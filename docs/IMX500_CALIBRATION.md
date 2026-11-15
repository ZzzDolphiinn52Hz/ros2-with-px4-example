# Hiệu chuẩn nội tại IMX500 cho ArUco landing

Quy trình này hiệu chuẩn IMX500 ở đúng chế độ sử dụng khi bay: ảnh `640x480`,
mô hình camera pinhole và checkerboard A4 `9x6` góc trong, cạnh mỗi ô `25 mm`.

## 1. Chuẩn bị

- Chỉ gắn IMX500 vào CAM0 và cố định camera như khi bay.
- Camera nhìn thẳng xuống; mép trên ảnh hướng về mũi UAV.
- Dùng bản in
  `src/px4_uavcup_perception/calibration/checkerboard_9x6_25mm_a4.pdf`.
- Khi in chọn **Actual size / 100%**, không chọn Fit to page.
- Dùng thước kiểm tra một cạnh ô đúng `25 mm`.
- Dán toàn bộ tờ in lên một tấm bìa cứng hoặc mặt phẳng. Không cầm tờ giấy
  rời vì giấy cong sẽ làm tăng sai số reprojection và sai pose ArUco.
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
DISPLAY=:0 ./uavcup aruco-calibrate
```

Script tự tạo thư mục:

```text
artifacts/imx500_calibration/YYYYMMDD_HHMMSS/
```

## 4. Chụp ảnh minh chứng cho báo cáo

Phím `s` của cửa sổ OpenCV có thể không hoạt động trên XWayland. Cách chụp
được khuyến nghị là mở Terminal 3, chạy lệnh dưới đây rồi dùng chuột chọn cửa
sổ hoặc vùng GUI cần lưu:

```bash
cd ~/ros2_ws
./uavcup aruco-calib-shot
```

Mỗi lần cần một ảnh mới thì chạy lại lệnh. Ảnh được lưu vào `screenshots/` của
phiên calibration mới nhất. Nên có ít nhất các ảnh sau cho báo cáo:

1. Checkerboard được nhận dạng ở giữa ảnh.
2. Checkerboard ở bốn vùng trái, phải, trên và dưới.
3. Checkerboard ở gần và xa để thay đổi thanh **Size**.
4. Checkerboard nghiêng quanh hai trục để tăng thanh **Skew**.
5. Bốn thanh X, Y, Size, Skew đã đủ và nút **CALIBRATE** sáng.
6. Ảnh đã rectify sau khi calibration hoàn tất.

Phím `s` vẫn có thể dùng nếu cửa sổ OpenCV đang có keyboard focus. Nếu sau khi
nhấn không thấy dòng `Saved screen dump` trong Terminal 2 thì dùng
`aruco-calib-shot`.

## 5. Thu thập bộ ảnh calibration

- Thu khoảng 30 đến 40 tư thế rõ nét và khác nhau; không cần nhiều ảnh gần như
  trùng lặp.
- Đưa checkerboard tới gần bốn cạnh và bốn góc ảnh, không chỉ quanh tâm ảnh.
- Thay đổi khoảng cách để thanh **Size** có cả mẫu gần và xa.
- Nghiêng bảng vừa phải theo cả hai trục để tăng **Skew**, nhưng luôn giữ thấy
  trọn toàn bộ góc trong `9x6`.
- Giữ bảng và camera đứng yên khoảng một giây ở mỗi tư thế. Bỏ qua khung hình
  bị nhòe, rung hoặc cháy sáng.
- Không uốn bảng và không thay đổi zoom, độ phân giải hay mode camera giữa
  các mẫu.

Nút **CALIBRATE** sáng chỉ có nghĩa dữ liệu đã đủ đa dạng theo bộ kiểm tra của
GUI; vẫn phải kiểm tra chất lượng ảnh và RMS sau khi lưu.

## 6. Tính và lưu kết quả

Khi nút **CALIBRATE** sáng:

1. Bấm **CALIBRATE** đúng một lần.
2. Chờ tới khi cửa sổ phản hồi trở lại; trên Pi có thể mất khoảng một phút.
3. Kiểm tra các đường thẳng trong ảnh rectify không bị cong bất thường.
4. Dùng `./uavcup aruco-calib-shot` để lưu ảnh kết quả.
5. Bấm **SAVE** để tạo archive. Không bấm **COMMIT**.
6. Quay về Terminal 2 và nhấn `Ctrl+C`.

Script sẽ giữ lại:

```text
terminal.log                  toàn bộ log và ma trận do calibrator in ra
calibrationdata.tar.gz        dữ liệu gốc của ROS camera_calibration
extracted/ost.yaml            thông số dùng để cập nhật cấu hình
extracted/left-*.png          các ảnh checkerboard được chọn
screenshots/gui_*.png          ảnh GUI tạo bởi aruco-calib-shot
screenshots/dump*.png         ảnh tùy chọn tạo bằng phím s
session.txt                   mode ảnh và kích thước checkerboard
```

## 7. Đọc thông số

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

## 8. Tiêu chí chấp nhận

- Kết quả đúng độ phân giải `640x480`.
- `fx`, `fy` dương và có cùng bậc độ lớn.
- `cx`, `cy` nằm bên trong ảnh.
- Không có `NaN` hoặc `Inf`.
- Ảnh rectify hợp lý và đường thẳng gần rìa ảnh thẳng hơn ảnh raw.
- Với pipeline này, đặt tiêu chí RMS reprojection dưới `1.0 px`; nên hiệu
  chuẩn lại nếu RMS lớn hơn mức này. Mục tiêu tốt là khoảng `0.5 px` hoặc thấp
  hơn khi bảng phẳng, ảnh rõ và độ phủ tốt.
- Sau khi cập nhật config phải kiểm tra lại `tvec.x/y/z` bằng marker có kích
  thước đã đo chính xác trước khi bật PID.

## 9. Dấu hiệu phải hiệu chuẩn lại

- Checkerboard được cầm bằng giấy rời và nhìn thấy bị cong trong ảnh.
- Nhiều ảnh bị nhòe hoặc bảng di chuyển khi GUI lấy mẫu.
- Phần lớn mẫu nằm giữa ảnh, thiếu mẫu gần cạnh và góc.
- `fx` và `fy` chênh lệch bất thường, tâm ảnh nằm ngoài kích thước ảnh hoặc có
  `NaN`/`Inf`.
- RMS reprojection lớn hơn `1.0 px`.
- Ảnh rectify bị méo bất thường hoặc pose marker dao động mạnh khi marker đứng
  yên.

Các file trong `artifacts/` là dữ liệu thí nghiệm và không tự động được commit
vào Git. Hãy sao lưu riêng thư mục phiên được dùng trong báo cáo.

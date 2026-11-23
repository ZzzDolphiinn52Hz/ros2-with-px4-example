# Kiểm tra nguồn GPIO cho Raspberry Pi 5

Tài liệu này dùng cho bài test bench khi Pi được cấp nguồn theo đường:

```text
LiPo → XL4015 → chân 5V/GND GPIO → Raspberry Pi 5
```

Bài test không chạy landing PID, không enable PX4 bridge, không request
Offboard và phải tháo cánh quạt.

## 1. Kiểm tra trước khi chạy

```bash
ssh raspi
cd ~/ros2_ws
vcgencmd get_throttled
vcgencmd pmic_read_adc EXT5V_V
vcgencmd measure_temp
docker compose ps
```

Nếu đang có camera server hoặc container perception cũ, dừng chúng trước khi
mở bài test mới. Không dùng đồng thời USB-C và nguồn GPIO nếu chưa có mạch
power-OR phù hợp.

## 2. Mở workload ArUco

Mở ba terminal SSH riêng tới Pi.

Terminal 1 — camera IMX500:

```bash
ssh raspi
cd ~/ros2_ws
./uavcup aruco-camera
```

Terminal 2 — detector ArUco, không có PX4 output:

```bash
ssh raspi
cd ~/ros2_ws
./uavcup aruco-test
```

Terminal 3 — kiểm tra trạng thái detector:

```bash
ssh raspi
cd ~/ros2_ws
docker compose run --rm ros bash -lc \
'source /opt/ros/humble/setup.bash && \
 source install/setup.bash && \
 ros2 topic echo /uav/aruco/status --once'
```

Kết quả hợp lệ tối thiểu là `message: ok`. `marker_count=0` vẫn hợp lệ cho
bài test nguồn nếu không đặt marker.

## 3. Ghi CSV thủ công mỗi giây

Terminal 4 ghi dữ liệu trong 10 phút. Dòng điện chưa được đo; cần INA219,
INA226 hoặc watt meter riêng cho đại lượng đó.

```bash
ssh raspi
cd ~/ros2_ws
OUT="artifacts/power_test/manual_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
CSV="$OUT/power_log.csv"
printf 'timestamp_iso,elapsed_s,ext5v_v,cpu_temp_c,throttled_hex,load_1m\n' > "$CSV"
START=$(date +%s)
for i in $(seq 0 599); do
  NOW=$(date --iso-8601=seconds)
  ELAPSED=$(( $(date +%s) - START ))
  VOLT=$(vcgencmd pmic_read_adc EXT5V_V | sed -n 's/.*=\([0-9.]*\)V/\1/p')
  TEMP=$(vcgencmd measure_temp | sed -n 's/temp=\([0-9.]*\).*/\1/p')
  THROTTLED=$(vcgencmd get_throttled | cut -d= -f2)
  LOAD=$(cut -d' ' -f1 /proc/loadavg)
  printf '%s,%s,%s,%s,%s,%s\n' "$NOW" "$ELAPSED" "$VOLT" "$TEMP" "$THROTTLED" "$LOAD" | tee -a "$CSV"
  sleep 1
done
echo "Saved: $CSV"
```

Dừng sớm bằng `Ctrl+C`; file CSV đã ghi vẫn giữ lại. Với dữ liệu dùng trong
báo cáo, nên chạy đủ 600 mẫu và ghi rõ nếu phiên bị gián đoạn.

## 4. Kiểm tra kết quả

```bash
cd ~/ros2_ws
ls -lh artifacts/power_test/manual_*/power_log.csv
wc -l artifacts/power_test/manual_*/power_log.csv
tail -n 5 artifacts/power_test/manual_*/power_log.csv
vcgencmd get_throttled
journalctl -b -k --no-pager | grep -Eai \
'undervoltage|voltage normalised|thrott|brown.?out' || true
```

`throttled=0x0` nghĩa là không có cờ undervoltage/throttling trong boot hiện
tại. `EXT5V_V` là điện áp ở PMIC, không phải dòng hoặc công suất XL4015.

## 5. Dừng an toàn

Nhấn `Ctrl+C` ở Terminal 2 trước, sau đó Terminal 1:

```bash
ps -eo pid,cmd | grep -E \
'picamera2_frame_server|aruco_test.launch|aruco_detector_node' | grep -v grep || true
docker compose ps
```

Không dừng XRCE Agent nếu nó đang được dùng cho phiên khác.

## Phiên đã hoàn tất

Phiên hợp lệ ngày 2026-11-23 nằm tại:

```text
/home/pi/ros2_ws/artifacts/power_test/gpio_aruco_20261123_222253/
```

Chi tiết số liệu và giới hạn nằm trong
`specs/002-pi-gpio-power-validation/verification.md`.

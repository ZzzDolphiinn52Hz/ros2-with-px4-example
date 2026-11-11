# px4_uavcup_bringup

Package ghép các component Pi 5 nhưng không sao chép config của component.

| Launch | Phạm vi | PX4 output |
| --- | --- | --- |
| `aruco_test.launch.py` | ArUco detector | không |
| `pi_aruco_vehicle.launch.py` | ArUco + landing PID + PX4 bridge, không ZipDepth | có, disabled |
| `zipdepth_test.launch.py` | ZipDepth | không |
| `pi_perception_test.launch.py` | cả hai camera | không |
| `pi_corridor_px4_bench.launch.py` | ZipDepth + hai safety gate | có, disabled |
| `pi_vehicle.launch.py` | perception + landing + bridge | có, disabled |

Để chuyển giao và vận hành, dùng CLI ở root repository thay vì tự ghép chuỗi
lệnh Docker/colcon:

```bash
./uavcup setup
./uavcup aruco-camera
./uavcup agent
./uavcup aruco-vehicle
```

Profile `aruco-vehicle` dành cho hệ một IMX500 tại CAM0. Nó không cần IMX219
hoặc model ZipDepth. `pi_aruco_vehicle.launch.py` không tự enable PID/adapter,
không arm và không tự request Offboard.

Khi chỉ kiểm tra detector, dùng `./uavcup aruco-debug` thay cho
`./uavcup aruco-vehicle`; profile debug không tạo PX4 command topic.

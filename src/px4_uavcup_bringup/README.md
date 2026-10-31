# px4_uavcup_bringup

Package ghép các component Pi 5 nhưng không sao chép config của component.

| Launch | Phạm vi | PX4 output |
| --- | --- | --- |
| `aruco_test.launch.py` | ArUco detector | không |
| `zipdepth_test.launch.py` | ZipDepth | không |
| `pi_perception_test.launch.py` | cả hai camera | không |
| `pi_corridor_px4_bench.launch.py` | ZipDepth + hai safety gate | có, disabled |
| `pi_vehicle.launch.py` | perception + landing + bridge | có, disabled |

Để chuyển giao và vận hành, dùng CLI ở root repository thay vì tự ghép chuỗi
lệnh Docker/colcon:

```bash
./uavcup setup
./uavcup cameras
./uavcup perception-test
./uavcup agent
./uavcup vehicle
```

`pi_vehicle.launch.py` không tự enable controller/adapter, không arm và không
tự request Offboard.

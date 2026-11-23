# Raspberry Pi GPIO Power Validation — 2026-11-23

The Pi at `192.168.1.120` was tested while powered through the GPIO 5 V/GND
header from the LiPo → XL4015 path. The accepted workload was IMX500 at
640x480/15 FPS, the ArUco debug perception profile, and the pre-existing XRCE
Agent. Landing PID, PX4 bridge enable, Offboard request, arming and flight
were not used.

Accepted evidence is documented in:

```text
specs/002-pi-gpio-power-validation/verification.md
```

The repeatable operator procedure is documented in:

```text
docs/PI_GPIO_POWER_TEST.md
```

Pi artifact directory:

```text
/home/pi/ros2_ws/artifacts/power_test/gpio_aruco_20261123_222253/
```

The 10-minute, 600-sample run measured EXT5V_V mean/min/max of
5.0002/4.8923/5.0384 V, CPU temperature mean/max of 70.4/74.1 °C, zero new
undervoltage events and zero nonzero `get_throttled` samples. Input current
was not measured. Repeat at minimum expected LiPo voltage, test dynamic loads
with propellers removed, and add an external current sensor before treating
this as flight-readiness evidence.

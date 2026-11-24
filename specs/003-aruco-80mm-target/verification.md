# Verification: 80 mm ArUco Landing Target

**Date**: 2026-11-24
**Base commit**: `6a1b6cd`
**Hardware/flight**: NOT RUN

## Software Evidence

```text
/usr/bin/python3 -m pytest -q \
  src/px4_uavcup_perception/test/test_camera_calibration_config.py
```

Result: PASS, 2 tests passed.

Python syntax compilation for the target generator and detector node: PASS.
`git diff --check`: PASS.

`./uavcup unit-test`: NOT RUN on this development checkout because the Docker
command is unavailable. The targeted configuration test above does not require
ROS hardware or Docker.

## Asset Evidence

- PDF page: A4 portrait, 595.276 x 841.89 pt.
- 300 DPI page: 2480 x 3508 px.
- Marker square: 945 x 945 px at 300 DPI, equivalent to approximately
  80.01 x 80.01 mm due to integer pixel rounding.
- Marker pixels: binary black/white.
- Visual preview: PASS.

## Remaining Human Verification

Print the PDF with Actual size / 100% and disable Fit/Shrink. Measure both
outer black edges with a ruler; each must be 80 mm before collecting pose.
Then run ArUco-only static tests on the Raspberry Pi. Flight validation remains
NOT RUN.

#!/usr/bin/env python3
"""Start the host-side CAM0 ZipDepth and CAM1 ArUco frame servers."""

from px4_uavcup_perception.cameras.picamera2_supervisor import main


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(unset CDPATH; cd -- "$(dirname -- "$0")/../../.." && pwd)"
cd "$WORKSPACE_DIR"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is empty. Run this from the Raspberry Pi desktop." >&2
  exit 1
fi
if [[ ! -S "$WORKSPACE_DIR/run/down_camera.sock" ]]; then
  echo "ERROR: IMX500 frame server is not running." >&2
  echo "Open another terminal and run: ./uavcup aruco-camera" >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  echo "ERROR: ROS workspace is not built. Run: ./uavcup setup" >&2
  exit 1
fi

if command -v xhost >/dev/null 2>&1; then
  xhost +si:localuser:root >/dev/null
fi

session_id="$(date +%Y%m%d_%H%M%S)"
host_session_dir="$WORKSPACE_DIR/artifacts/imx500_calibration/$session_id"
container_session_dir="/ros2_ws/artifacts/imx500_calibration/$session_id"
mkdir -p "$host_session_dir"

printf '%s\n' \
  'camera_model=IMX500' \
  'image_width=640' \
  'image_height=480' \
  'checkerboard_inner_corners=9x6' \
  'checkerboard_square_size_m=0.025' \
  "started_at=$(date --iso-8601=seconds)" \
  > "$host_session_dir/session.txt"

echo "Calibration session: $host_session_dir"
echo "Press 's' in the GUI to save report screenshots."
echo "After CALIBRATE finishes, click SAVE, then press Ctrl+C here."

set +e
docker compose run --rm \
  -e DISPLAY="$DISPLAY" \
  -e CALIBRATION_SESSION_DIR="$container_session_dir" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  ros bash -lc '
    set -uo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    collect_calibration_artifacts() {
      mkdir -p "$CALIBRATION_SESSION_DIR/screenshots"
      if [[ -f /tmp/calibrationdata.tar.gz ]]; then
        cp /tmp/calibrationdata.tar.gz "$CALIBRATION_SESSION_DIR/"
        mkdir -p "$CALIBRATION_SESSION_DIR/extracted"
        tar -xzf /tmp/calibrationdata.tar.gz \
          -C "$CALIBRATION_SESSION_DIR/extracted"
      fi
      shopt -s nullglob
      for screenshot in /tmp/dump*.png; do
        cp "$screenshot" "$CALIBRATION_SESSION_DIR/screenshots/"
      done
      chown -R "$HOST_UID:$HOST_GID" "$CALIBRATION_SESSION_DIR"
    }
    trap collect_calibration_artifacts EXIT

    ros2 launch px4_uavcup_perception camera_calibration_gui.launch.py \
      board_size:=9x6 square_size_m:=0.025
  ' 2>&1 | tee "$host_session_dir/terminal.log"
run_status="${PIPESTATUS[0]}"
set -e

echo
echo "Saved session: $host_session_dir"
if [[ -f "$host_session_dir/extracted/ost.yaml" ]]; then
  echo "Calibration YAML: $host_session_dir/extracted/ost.yaml"
else
  echo "WARNING: ost.yaml was not created. Click SAVE after calibration." >&2
fi

# Ctrl+C is the documented way to finish after SAVE, so status 130 is normal.
if [[ "$run_status" -ne 0 && "$run_status" -ne 130 ]]; then
  exit "$run_status"
fi

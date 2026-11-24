#!/usr/bin/env python3
"""Plot ArUco position-test pose and command logs.

The input is the compact JSON-gzip export produced from the Pi rosbag files.
Each record has the form ``[test, stream, time_s, ...]``.  Each output figure
overlays pose ``x,y`` and body-FLU velocity ``x,y`` using two y-axes because
the quantities have different units.
"""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt


TESTS = ("center", "right", "multidirectional3")
TEST_TITLES = {
    "center": "Center (z danh định 1,1208 m)",
    "right": "Right (z danh định 1,1208 m)",
    "multidirectional3": "Đa hướng (độ cao khoảng 1,3 m)",
}
COLORS = {"x": "#c0392b", "y": "#2471a3", "z": "#239b56"}


def load_records(path: Path) -> list[list[object]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def series(records: list[list[object]], test: str, stream: str):
    return [record for record in records if record[0] == test and record[1] == stream]


def configure_axis(axis, title: str, ylabel: str) -> None:
    axis.set_title(title, loc="left", fontsize=11, fontweight="bold")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)
    axis.axhline(0.0, color="#666666", linewidth=0.8, alpha=0.7)
    axis.margins(x=0.01)


def plot_pose_cmdvel(records: list[list[object]], test: str, output: Path) -> None:
    """Write one four-signal x/y pose and command plot for a test."""
    figure, axis_pose = plt.subplots(figsize=(11, 5.5))
    axis_cmd = axis_pose.twinx()
    pose_rows = series(records, test, "pose")
    cmd_rows = series(records, test, "cmd")

    if pose_rows:
        time = [float(row[2]) for row in pose_rows]
        pose_x, = axis_pose.plot(
            time, [float(row[3]) for row in pose_rows],
            color=COLORS["x"], linewidth=1.7, label="pose x",
        )
        pose_y, = axis_pose.plot(
            time, [float(row[4]) for row in pose_rows],
            color=COLORS["y"], linewidth=1.7, label="pose y",
        )
    else:
        axis_pose.text(0.5, 0.5, "Không có dữ liệu pose", ha="center", va="center",
                       transform=axis_pose.transAxes)

    handles = list(axis_pose.lines)
    if cmd_rows:
        time = [float(row[2]) for row in cmd_rows]
        cmd_x, = axis_cmd.plot(
            time, [float(row[3]) for row in cmd_rows],
            color=COLORS["x"], linestyle="--", linewidth=1.5, label="cmd_vel x",
        )
        cmd_y, = axis_cmd.plot(
            time, [float(row[4]) for row in cmd_rows],
            color=COLORS["y"], linestyle="--", linewidth=1.5, label="cmd_vel y",
        )
        handles.extend([cmd_x, cmd_y])
    else:
        axis_cmd.text(
            0.5, 0.5, "Không có /aruco_land/cmd_vel trong rosbag center",
            ha="center", va="center", transform=axis_cmd.transAxes, fontsize=10,
        )

    axis_pose.set_title(f"{TEST_TITLES[test]} — pose x,y và cmd_vel x,y",
                        loc="left", fontsize=11, fontweight="bold")
    axis_pose.set_xlabel("Thời gian kể từ đầu phiên (s)")
    axis_pose.set_ylabel("Pose (m)")
    axis_cmd.set_ylabel("Vận tốc body-FLU (m/s)")
    axis_pose.grid(True, alpha=0.25)
    axis_pose.axhline(0.0, color="#666666", linewidth=0.8, alpha=0.7)
    axis_pose.margins(x=0.01)
    axis_pose.legend(handles=handles, loc="upper right", ncol=2, fontsize=9, frameon=True)
    figure.tight_layout()
    figure.savefig(output, dpi=220, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = load_records(args.input)
    for test in TESTS:
        plot_pose_cmdvel(
            records,
            test,
            args.output_dir / f"04-aruco-{test}-pose-cmdvel-xy.png",
        )


if __name__ == "__main__":
    main()

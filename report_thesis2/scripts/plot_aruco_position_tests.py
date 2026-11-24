#!/usr/bin/env python3
"""Plot ArUco position-test pose and command logs.

The input is the compact JSON-gzip export produced from the Pi rosbag files.
Each record has the form ``[test, stream, time_s, ...]``.  The script keeps
pose (metres) and body-FLU velocity (metres/second) in separate figures so the
different units are not visually conflated.
"""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt


TESTS = ("center", "right", "front_right")
TEST_TITLES = {
    "center": "Center (z danh định 1,1208 m)",
    "right": "Right (z danh định 1,1208 m)",
    "front_right": "Front-right (có dịch chuyển trong lúc ghi)",
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


def plot_pose(records: list[list[object]], output: Path) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    for axis, test in zip(axes, TESTS):
        rows = series(records, test, "pose")
        if not rows:
            axis.text(
                0.5,
                0.5,
                "Không có dữ liệu pose",
                ha="center",
                va="center",
                transform=axis.transAxes,
            )
            axis.set_title(TEST_TITLES[test], loc="left", fontsize=11, fontweight="bold")
            axis.grid(True, alpha=0.25)
            continue
        time = [float(row[2]) for row in rows]
        for index, name in enumerate(("x", "y", "z"), start=3):
            axis.plot(time, [float(row[index]) for row in rows],
                      color=COLORS[name], linewidth=1.5, label=f"pose {name}")
        configure_axis(axis, TEST_TITLES[test], "Pose (m)")
        axis.legend(loc="upper right", ncol=3, fontsize=8, frameon=True)
    axes[-1].set_xlabel("Thời gian kể từ đầu phiên (s)")
    figure.suptitle("Pose ArUco trong ba vị trí kiểm tra", fontsize=14, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output, dpi=220, bbox_inches="tight")
    plt.close(figure)


def plot_cmd_vel(records: list[list[object]], output: Path) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    for axis, test in zip(axes, TESTS):
        rows = series(records, test, "cmd")
        if not rows:
            axis.text(
                0.5,
                0.5,
                "Không có /aruco_land/cmd_vel trong rosbag center",
                ha="center",
                va="center",
                transform=axis.transAxes,
                fontsize=10,
            )
            axis.set_title(TEST_TITLES[test], loc="left", fontsize=11, fontweight="bold")
            axis.grid(True, alpha=0.25)
            continue
        time = [float(row[2]) for row in rows]
        for index, name in enumerate(("x", "y", "z"), start=3):
            axis.plot(time, [float(row[index]) for row in rows],
                      color=COLORS[name], linewidth=1.4, label=f"cmd_vel {name}")
        configure_axis(axis, TEST_TITLES[test], "Vận tốc body-FLU (m/s)")
        axis.legend(loc="upper right", ncol=3, fontsize=8, frameon=True)
    axes[-1].set_xlabel("Thời gian kể từ đầu phiên (s)")
    figure.suptitle("Lệnh vận tốc ArUco trong ba vị trí kiểm tra", fontsize=14, fontweight="bold")
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(output, dpi=220, bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = load_records(args.input)
    plot_pose(records, args.output_dir / "04-aruco-position-pose.png")
    plot_cmd_vel(records, args.output_dir / "04-aruco-position-cmdvel.png")


if __name__ == "__main__":
    main()

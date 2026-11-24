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


def plot_xy(records: list[list[object]], test: str, stream: str, output: Path) -> None:
    """Write one x/y plot for a single test and stream."""
    figure, axis = plt.subplots(figsize=(11, 5.5))
    rows = series(records, test, stream)
    ylabel = "Pose (m)" if stream == "pose" else "Vận tốc body-FLU (m/s)"
    prefix = "pose" if stream == "pose" else "cmd_vel"
    if not rows:
        message = (
            "Không có /aruco_land/cmd_vel trong rosbag center"
            if stream == "cmd"
            else "Không có dữ liệu pose"
        )
        axis.text(0.5, 0.5, message, ha="center", va="center", transform=axis.transAxes)
        axis.set_xlim(0, 1)
    else:
        time = [float(row[2]) for row in rows]
        for index, name in ((3, "x"), (4, "y")):
            axis.plot(
                time,
                [float(row[index]) for row in rows],
                color=COLORS[name],
                linewidth=1.6,
                label=f"{prefix} {name}",
            )
        axis.legend(loc="upper right", ncol=2, fontsize=9, frameon=True)
    configure_axis(axis, f"{TEST_TITLES[test]} — {prefix} x,y", ylabel)
    axis.set_xlabel("Thời gian kể từ đầu phiên (s)")
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
        plot_xy(records, test, "pose", args.output_dir / f"04-aruco-{test}-pose-xy.png")
        plot_xy(records, test, "cmd", args.output_dir / f"04-aruco-{test}-cmdvel-xy.png")


if __name__ == "__main__":
    main()

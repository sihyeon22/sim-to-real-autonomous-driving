#!/usr/bin/env python3
"""
Compare AMCL localization accuracy across IMU noise levels.

Reads one bag per noise condition (low / mid / high), computes time-series
AMCL position/yaw error vs CARLA ground truth, and plots all three on one figure.

Usage:
    python3 scripts/plot_imu_noise_compare.py \
        --low-bag  bags/imu/test/low_noise \
        --mid-bag  bags/imu/test/mid_noise \
        --high-bag bags/imu/test/high_noise \
        --out      figures/imu_noise_compare.png
"""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CARLA_ODOM_TOPIC = "/carla/hero/odometry"
AMCL_TOPIC       = "/amcl_pose"

LOW_COLOR  = "#4CAF50"   # green
MID_COLOR  = "#FF9800"   # orange
HIGH_COLOR = "#F44336"   # red


@dataclass
class Pose2D:
    t_ns: int
    x: float
    y: float
    yaw: float


def quat_to_yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        t_ns=b.t_ns,
        x=a.x + ca * b.x - sa * b.y,
        y=a.y + sa * b.x + ca * b.y,
        yaw=wrap(a.yaw + b.yaw),
    )


def invert(p: Pose2D) -> Pose2D:
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose2D(t_ns=p.t_ns, x=-(c * p.x + s * p.y), y=-(-s * p.x + c * p.y), yaw=wrap(-p.yaw))


def nearest(poses: List[Pose2D], t_ns: int, max_dt_ns: int = 300_000_000) -> Optional[Pose2D]:
    if not poses:
        return None
    stamps = [p.t_ns for p in poses]
    i = bisect.bisect_left(stamps, t_ns)
    candidates = []
    if i < len(poses): candidates.append(poses[i])
    if i > 0:          candidates.append(poses[i - 1])
    best = min(candidates, key=lambda p: abs(p.t_ns - t_ns))
    return best if abs(best.t_ns - t_ns) <= max_dt_ns else None


def read_bag(bag_path: Path) -> Tuple[List[Pose2D], List[Pose2D]]:
    """Returns (gt_poses, amcl_poses)."""
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    gt_poses: List[Pose2D] = []
    amcl_poses: List[Pose2D] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == CARLA_ODOM_TOPIC and topic in type_map:
            msg = deserialize_message(data, type_map[topic])
            t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            gt_poses.append(Pose2D(
                t_ns=t_ns,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=quat_to_yaw(msg.pose.pose.orientation),
            ))

        elif topic == AMCL_TOPIC and topic in type_map:
            msg = deserialize_message(data, type_map[topic])
            t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            amcl_poses.append(Pose2D(
                t_ns=t_ns,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=quat_to_yaw(msg.pose.pose.orientation),
            ))

    return gt_poses, amcl_poses


def compute_error_series(
    gt_poses: List[Pose2D], amcl_poses: List[Pose2D]
) -> Tuple[List[float], List[float], List[float]]:
    """Returns (times_s, pos_errors_m, yaw_errors_deg). offset-correct mode."""
    if not gt_poses or not amcl_poses:
        return [], [], []

    t0_ns: Optional[int] = None
    offset: Optional[Pose2D] = None
    times, pos_errs, yaw_errs = [], [], []

    for amcl in amcl_poses:
        gt = nearest(gt_poses, amcl.t_ns)
        if gt is None:
            continue
        if t0_ns is None:
            t0_ns = amcl.t_ns
        if offset is None:
            offset = compose(amcl, invert(gt))

        gt_in_amcl = compose(offset, gt)
        pos_err = math.hypot(amcl.x - gt_in_amcl.x, amcl.y - gt_in_amcl.y)
        yaw_err = abs(wrap(amcl.yaw - gt_in_amcl.yaw))

        times.append((amcl.t_ns - t0_ns) / 1e9)
        pos_errs.append(pos_err)
        yaw_errs.append(math.degrees(yaw_err))

    return times, pos_errs, yaw_errs


def print_summary(label: str, pos_errs: List[float], yaw_errs: List[float]) -> tuple:
    pa = np.mean(pos_errs); pm = np.max(pos_errs)
    ya = np.mean(yaw_errs); ym = np.max(yaw_errs)
    print(f"  {label:12s}  pos avg={pa:.4f}m  pos max={pm:.4f}m  "
          f"yaw avg={ya:.4f}°  yaw max={ym:.4f}°")
    return pa, pm, ya, ym


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--low-bag",  type=Path, required=True)
    parser.add_argument("--mid-bag",  type=Path, required=True)
    parser.add_argument("--high-bag", type=Path, required=True)
    parser.add_argument("--labels",   type=str, nargs=3,
                        default=["low", "mid", "high"],
                        metavar=("LOW", "MID", "HIGH"),
                        help="Legend labels for low/mid/high conditions")
    parser.add_argument("--title",    type=str, default="AMCL Localization Error — Comparison")
    parser.add_argument("--out",      type=Path, default=Path("imu_noise_compare.png"))
    parser.add_argument("--ylim-pos", type=float, default=None)
    parser.add_argument("--ylim-yaw", type=float, default=None)
    args = parser.parse_args()

    conditions = [
        (args.labels[0], args.low_bag.expanduser().resolve(),  LOW_COLOR),
        (args.labels[1], args.mid_bag.expanduser().resolve(),  MID_COLOR),
        (args.labels[2], args.high_bag.expanduser().resolve(), HIGH_COLOR),
    ]

    fig, (ax_pos, ax_yaw) = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

    print("=" * 70)
    print(f"{'label':12s}  {'pos avg':>12}  {'pos max':>12}  {'yaw avg':>12}  {'yaw max':>12}")
    print("-" * 70)

    for label, bag_path, color in conditions:
        print(f"Reading {bag_path.name} ...")
        gt_poses, amcl_poses = read_bag(bag_path)
        times, pos_errs, yaw_errs = compute_error_series(gt_poses, amcl_poses)

        if not times:
            print(f"  [skip] No matched data in {bag_path.name}")
            continue

        ax_pos.plot(times, pos_errs, color=color, linewidth=1.5, label=label)
        ax_yaw.plot(times, yaw_errs, color=color, linewidth=1.5, label=label)

        print_summary(label, pos_errs, yaw_errs)

    print("=" * 70)

    for ax, ylabel, ylim in [
        (ax_pos, "Position error [m]", args.ylim_pos),
        (ax_yaw, "Yaw error [deg]",    args.ylim_yaw),
    ]:
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_xlabel("Time [s]", fontsize=11)
        ax.legend(fontsize=9, loc="upper left")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        if ylim is not None:
            ax.set_ylim(bottom=0, top=ylim)

    fig.suptitle(args.title, fontsize=13, fontweight="bold")
    fig.tight_layout()
    out_path = args.out.expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"\nSaved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()

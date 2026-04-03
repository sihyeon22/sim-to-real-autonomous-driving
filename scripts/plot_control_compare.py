#!/usr/bin/env python3
"""
Compare AMCL localization accuracy: ackermann_control vs twist_to_control.

Reads 3 bag files per condition, computes time-series AMCL position/yaw error
vs CARLA ground truth, then plots mean ± std across runs.

Usage (single scenario):
    python3 plot_control_compare.py \
        --ackermann-dir ~/CARLA_ws/localization_eval/ackermann/straight \
        --twist-dir     ~/CARLA_ws/localization_eval/twist/straight \
        --title "Straight"

Output: saves PNG to current directory (control_compare_<title>.png)
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

ACKERMANN_COLOR = "#2196F3"  # blue
TWIST_COLOR     = "#F44336"  # red


@dataclass
class Pose2D:
    t_ns: int    # nanoseconds (absolute)
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
    if i < len(poses):
        candidates.append(poses[i])
    if i > 0:
        candidates.append(poses[i - 1])
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
    """
    Returns (times_s, pos_errors_m, yaw_errors_deg) relative to bag start.
    Uses offset-correct mode: initial SE2 offset subtracted so only drift is shown.
    """
    if not gt_poses or not amcl_poses:
        return [], [], []

    t0_ns: Optional[int] = None  # set on first GT-matched AMCL pose
    offset: Optional[Pose2D] = None
    times, pos_errs, yaw_errs = [], [], []

    for amcl in amcl_poses:
        gt = nearest(gt_poses, amcl.t_ns)
        if gt is None:
            continue
        if t0_ns is None:
            t0_ns = amcl.t_ns  # t=0 is the first AMCL pose that has a GT match
        if offset is None:
            # offset: maps GT frame into AMCL frame at t0
            offset = compose(amcl, invert(gt))

        gt_in_amcl = compose(offset, gt)
        pos_err = math.hypot(amcl.x - gt_in_amcl.x, amcl.y - gt_in_amcl.y)
        yaw_err = abs(wrap(amcl.yaw - gt_in_amcl.yaw))

        times.append((amcl.t_ns - t0_ns) / 1e9)
        pos_errs.append(pos_err)
        yaw_errs.append(math.degrees(yaw_err))

    return times, pos_errs, yaw_errs


def average_runs(
    runs: List[Tuple[List[float], List[float], List[float]]], n_grid: int = 500
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Interpolates all runs onto a common time grid (intersection) and returns
    (t_grid, pos_mean, pos_std, yaw_mean, yaw_std).
    """
    t_min = max(r[0][0]  for r in runs)
    t_max = min(r[0][-1] for r in runs)
    t_grid = np.linspace(t_min, t_max, n_grid)

    pos_mat = np.array([np.interp(t_grid, r[0], r[1]) for r in runs])
    yaw_mat = np.array([np.interp(t_grid, r[0], r[2]) for r in runs])

    return (
        t_grid,
        pos_mat.mean(axis=0), pos_mat.std(axis=0),
        yaw_mat.mean(axis=0), yaw_mat.std(axis=0),
    )


def plot_mean_std(ax, t, mean, std, color, label):
    ax.plot(t, mean, color=color, linewidth=1.8, label=label)
    ax.fill_between(t, mean - std, mean + std, color=color, alpha=0.2)


def collect_bags(directory: Path) -> List[Path]:
    """Return sorted list of bag dirs (containing *.db3) under directory."""
    bags = sorted(set(p.parent for p in directory.rglob("*.db3")))
    return bags


def process_bags(bags: List[Path], label: str) -> Optional[List[Tuple]]:
    runs = []
    for bag_path in bags:
        print(f"  Reading {bag_path.name} ...")
        gt_poses, amcl_poses = read_bag(bag_path)
        result = compute_error_series(gt_poses, amcl_poses)
        if result[0]:
            runs.append(result)
        else:
            print(f"  [skip] No matched data in {bag_path.name}")
    if not runs:
        print(f"  [warn] No valid runs for {label}")
        return None
    return runs


def plot_comparison(
    ackermann_bags: List[Path],
    twist_bags: List[Path],
    title: str,
    out_path: Path,
    ylim_pos: Optional[float] = None,
    ylim_yaw: Optional[float] = None,
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    ax_pos, ax_yaw = axes

    print("[ ackermann ]")
    ack_runs = process_bags(ackermann_bags, "ackermann")
    print("[ twist ]")
    twi_runs = process_bags(twist_bags, "twist")

    summary_rows = []

    if ack_runs:
        t, pm, ps, ym, ys = average_runs(ack_runs)
        n = len(ack_runs)
        plot_mean_std(ax_pos, t, pm, ps, ACKERMANN_COLOR, f"ackermann  (mean ± std, n={n})")
        plot_mean_std(ax_yaw, t, ym, ys, ACKERMANN_COLOR, f"ackermann  (mean ± std, n={n})")
        print(f"\nackermann — pos mean={pm.mean():.4f}m  max={pm.max():.4f}m  "
              f"yaw mean={ym.mean():.4f}°  max={ym.max():.4f}°")
        summary_rows.append(("ackermann", n, pm.mean(), pm.max(), ym.mean(), ym.max()))

    if twi_runs:
        t, pm, ps, ym, ys = average_runs(twi_runs)
        n = len(twi_runs)
        plot_mean_std(ax_pos, t, pm, ps, TWIST_COLOR, f"twist  (mean ± std, n={n})")
        plot_mean_std(ax_yaw, t, ym, ys, TWIST_COLOR, f"twist  (mean ± std, n={n})")
        print(f"twist     — pos mean={pm.mean():.4f}m  max={pm.max():.4f}m  "
              f"yaw mean={ym.mean():.4f}°  max={ym.max():.4f}°")
        summary_rows.append(("twist", n, pm.mean(), pm.max(), ym.mean(), ym.max()))

    # Print summary table
    print()
    print("=" * 70)
    print(f"{'':12s} {'n':>3}  {'pos avg':>10}  {'pos max':>10}  {'yaw avg':>10}  {'yaw max':>10}")
    print("-" * 70)
    for row in summary_rows:
        label, n, pa, pm2, ya, ym2 = row
        print(f"{label:12s} {n:>3}  {pa:>10.4f}m  {pm2:>10.4f}m  {ya:>10.4f}°  {ym2:>10.4f}°")
    print("=" * 70)

    for ax, ylabel, ylim in [
        (ax_pos, "Position error [m]", ylim_pos),
        (ax_yaw, "Yaw error [deg]",    ylim_yaw),
    ]:
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_xlabel("Time [s]", fontsize=11)
        ax.legend(fontsize=9, loc="upper left")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        if ylim is not None:
            ax.set_ylim(bottom=0, top=ylim)

    fig.suptitle(f"Control method comparison — {title}", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved: {out_path}")
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare ackermann vs twist AMCL localization accuracy.")
    parser.add_argument("--ackermann-dir", type=Path, required=True,
                        help="Dir containing ackermann bag folders (bag01, bag02, bag03)")
    parser.add_argument("--twist-dir",     type=Path, required=True,
                        help="Dir containing twist bag folders (bag01, bag02, bag03)")
    parser.add_argument("--title",    default="comparison", help="Plot title / filename suffix")
    parser.add_argument("--out",      type=Path, default=None,  help="Output PNG path")
    parser.add_argument("--ylim-pos", type=float, default=None, help="Fixed y-axis max for position error [m]")
    parser.add_argument("--ylim-yaw", type=float, default=None, help="Fixed y-axis max for yaw error [deg]")
    args = parser.parse_args()

    ackermann_bags = collect_bags(args.ackermann_dir.expanduser().resolve())
    twist_bags     = collect_bags(args.twist_dir.expanduser().resolve())

    print(f"ackermann bags : {[b.name for b in ackermann_bags]}")
    print(f"twist bags     : {[b.name for b in twist_bags]}")

    if len(ackermann_bags) != len(twist_bags):
        print(f"Warning: {len(ackermann_bags)} ackermann vs {len(twist_bags)} twist bags — using min count")
        n = min(len(ackermann_bags), len(twist_bags))
        ackermann_bags, twist_bags = ackermann_bags[:n], twist_bags[:n]

    safe_title = args.title.replace(" ", "_").replace("/", "-")
    out_path = args.out or Path(f"control_compare_{safe_title}.png")

    plot_comparison(ackermann_bags, twist_bags, args.title, out_path,
                    ylim_pos=args.ylim_pos, ylim_yaw=args.ylim_yaw)


if __name__ == "__main__":
    main()

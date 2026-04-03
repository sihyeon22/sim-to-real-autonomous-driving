#!/usr/bin/env python3
"""
Plot odometry drift over time: odom_local vs odom_wheel vs ground truth.

Usage (single pair):
    python3 plot_odom_drift.py \
        --local  ~/CARLA_ws/bags/odom_local/straight/test01 \
        --wheel  ~/CARLA_ws/bags/odom_wheel/straight/test01

Usage (all runs in a scenario):
    python3 plot_odom_drift.py \
        --local-dir  ~/CARLA_ws/bags/odom_local/straight \
        --wheel-dir  ~/CARLA_ws/bags/odom_wheel/straight \
        --title "Straight"

Output: saves PNG to current directory (odom_drift_<title>.png)
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
ODOM_LOCAL_TOPIC = "/odom_local"
ODOM_WHEEL_TOPIC = "/odom_wheel"

LOCAL_COLOR = "#2196F3"  # blue
WHEEL_COLOR = "#F44336"  # red


@dataclass
class Pose2D:
    t: float  # seconds from bag start
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


def _ns(msg_header) -> int:
    return msg_header.stamp.sec * 1_000_000_000 + msg_header.stamp.nanosec


def _pose(msg, t0_ns: int) -> Pose2D:
    return Pose2D(
        t=(_ns(msg.header) - t0_ns) / 1e9,
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        yaw=quat_to_yaw(msg.pose.pose.orientation),
    )


def read_bag(bag_path: Path) -> Tuple[List[Pose2D], List[Pose2D], List[Pose2D]]:
    """Returns (gt_poses, local_poses, wheel_poses) relative to bag start time."""
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    raw: dict[str, list] = {CARLA_ODOM_TOPIC: [], ODOM_LOCAL_TOPIC: [], ODOM_WHEEL_TOPIC: []}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in raw and topic in type_map:
            raw[topic].append(deserialize_message(data, type_map[topic]))

    if not raw[CARLA_ODOM_TOPIC]:
        raise ValueError(f"No {CARLA_ODOM_TOPIC} in {bag_path}")

    t0_ns = _ns(raw[CARLA_ODOM_TOPIC][0].header)

    gt     = [_pose(m, t0_ns) for m in raw[CARLA_ODOM_TOPIC]]
    local  = [_pose(m, t0_ns) for m in raw[ODOM_LOCAL_TOPIC]]
    wheel  = [_pose(m, t0_ns) for m in raw[ODOM_WHEEL_TOPIC]]
    return gt, local, wheel


def _nearest(poses: List[Pose2D], t: float, max_dt: float = 0.2) -> Optional[Pose2D]:
    if not poses:
        return None
    ts = [p.t for p in poses]
    i = bisect.bisect_left(ts, t)
    candidates = []
    if i < len(poses):
        candidates.append(poses[i])
    if i > 0:
        candidates.append(poses[i - 1])
    best = min(candidates, key=lambda p: abs(p.t - t))
    return best if abs(best.t - t) <= max_dt else None


def _compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """SE2 composition: a * b"""
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        t=b.t,
        x=a.x + ca * b.x - sa * b.y,
        y=a.y + sa * b.x + ca * b.y,
        yaw=wrap(a.yaw + b.yaw),
    )


def _invert(p: Pose2D) -> Pose2D:
    """SE2 inverse"""
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose2D(t=p.t, x=-(c * p.x + s * p.y), y=-(-s * p.x + c * p.y), yaw=wrap(-p.yaw))


def compute_drift(
    gt: List[Pose2D], odom: List[Pose2D]
) -> Tuple[List[float], List[float], List[float]]:
    """
    Returns (times, pos_errors, yaw_errors).
    Offset-corrects with SE2 transform (rotation + translation) so only drift is shown.
    """
    times, pos_errs, yaw_errs = [], [], []
    offset: Optional[Pose2D] = None  # T_odom_gt at t0

    for g in gt:
        o = _nearest(odom, g.t)
        if o is None:
            continue
        if offset is None:
            # offset = odom_t0 * inv(gt_t0)  →  maps gt frame into odom frame
            offset = _compose(o, _invert(g))

        g_in_odom = _compose(offset, g)
        pos_err = math.hypot(o.x - g_in_odom.x, o.y - g_in_odom.y)
        yaw_err = abs(wrap(o.yaw - g_in_odom.yaw))
        times.append(g.t)
        pos_errs.append(pos_err)
        yaw_errs.append(math.degrees(yaw_err))

    return times, pos_errs, yaw_errs


def _extract_run_data(
    bag_path: Path, odom_topic: str
) -> Optional[Tuple[List[float], List[float], List[float]]]:
    """Returns (times, pos_errs, yaw_errs) for one bag, or None if topic missing."""
    gt, local, wheel = read_bag(bag_path)
    odom = local if odom_topic == ODOM_LOCAL_TOPIC else wheel
    if not odom:
        print(f"  [skip] {odom_topic} not found in {bag_path.name}")
        return None
    return compute_drift(gt, odom)


def _average_runs(
    runs: List[Tuple[List[float], List[float], List[float]]], n_grid: int = 500
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Interpolates all runs to a common time grid and returns
    (t_grid, pos_mean, pos_std, yaw_mean, yaw_std).
    Uses the intersection of all run time ranges.
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


def _plot_mean_std(ax, t, mean, std, color, label):
    ax.plot(t, mean, color=color, linewidth=1.8, label=label)
    ax.fill_between(t, mean - std, mean + std, color=color, alpha=0.2)


def plot_runs(
    local_bags: List[Path],
    wheel_bags: List[Path],
    title: str,
    out_path: Path,
    ylim_pos: Optional[float] = None,
    ylim_yaw: Optional[float] = None,
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
    ax_pos, ax_yaw = axes

    local_runs = [_extract_run_data(b, ODOM_LOCAL_TOPIC) for b in local_bags]
    wheel_runs = [_extract_run_data(b, ODOM_WHEEL_TOPIC) for b in wheel_bags]
    local_runs = [r for r in local_runs if r is not None]
    wheel_runs = [r for r in wheel_runs if r is not None]

    if local_runs:
        t, pm, ps, ym, ys = _average_runs(local_runs)
        n = len(local_runs)
        _plot_mean_std(ax_pos, t, pm, ps, LOCAL_COLOR, f"odom_local  (mean ± std, n={n})")
        _plot_mean_std(ax_yaw, t, ym, ys, LOCAL_COLOR, f"odom_local  (mean ± std, n={n})")
        print(f"odom_local  — pos RMSE mean={pm.mean():.4f}m  yaw mean={ym.mean():.4f}°")

    if wheel_runs:
        t, pm, ps, ym, ys = _average_runs(wheel_runs)
        n = len(wheel_runs)
        _plot_mean_std(ax_pos, t, pm, ps, WHEEL_COLOR, f"odom_wheel (mean ± std, n={n})")
        _plot_mean_std(ax_yaw, t, ym, ys, WHEEL_COLOR, f"odom_wheel (mean ± std, n={n})")
        print(f"odom_wheel — pos RMSE mean={pm.mean():.4f}m  yaw mean={ym.mean():.4f}°")

    for ax, ylabel, ylim in [
        (ax_pos, "Position error [m]", ylim_pos),
        (ax_yaw, "Yaw error [deg]",   ylim_yaw),
    ]:
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_xlabel("Time [s]", fontsize=11)
        ax.legend(fontsize=9, loc="upper left")
        ax.grid(True, linestyle="--", alpha=0.5)
        ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
        if ylim is not None:
            ax.set_ylim(bottom=0, top=ylim)

    fig.suptitle(f"Odometry drift — {title}", fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    plt.show()


def _collect_bags(directory: Path) -> List[Path]:
    """Return sorted list of bag dirs (containing *.db3) under directory."""
    bags = sorted([p.parent for p in directory.rglob("*.db3")])
    # deduplicate
    seen = set()
    result = []
    for b in bags:
        if b not in seen:
            seen.add(b)
            result.append(b)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot odom_local vs odom_wheel drift.")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--local", type=Path, help="Single odom_local bag folder")
    group.add_argument("--local-dir", type=Path, help="Dir containing multiple odom_local bags")

    parser.add_argument("--wheel", type=Path, help="Single odom_wheel bag folder")
    parser.add_argument("--wheel-dir", type=Path, help="Dir containing multiple odom_wheel bags")
    parser.add_argument("--title", default="odom drift", help="Plot title / output filename suffix")
    parser.add_argument("--out", type=Path, default=None, help="Output PNG path")
    parser.add_argument("--ylim-pos", type=float, default=None, help="Fixed y-axis max for position error [m]")
    parser.add_argument("--ylim-yaw", type=float, default=None, help="Fixed y-axis max for yaw error [deg]")
    args = parser.parse_args()

    if args.local:
        local_bags = [args.local.expanduser().resolve()]
    else:
        local_bags = _collect_bags(args.local_dir.expanduser().resolve())

    if args.wheel:
        wheel_bags = [args.wheel.expanduser().resolve()]
    elif args.wheel_dir:
        wheel_bags = _collect_bags(args.wheel_dir.expanduser().resolve())
    else:
        parser.error("Provide --wheel or --wheel-dir to match --local / --local-dir")

    if len(local_bags) != len(wheel_bags):
        print(f"Warning: {len(local_bags)} local bags vs {len(wheel_bags)} wheel bags — using min count")
        n = min(len(local_bags), len(wheel_bags))
        local_bags, wheel_bags = local_bags[:n], wheel_bags[:n]

    safe_title = args.title.replace(" ", "_").replace("/", "-")
    out_path = args.out or Path(f"odom_drift_{safe_title}.png")

    print(f"Local bags : {[b.name for b in local_bags]}")
    print(f"Wheel bags : {[b.name for b in wheel_bags]}")
    plot_runs(local_bags, wheel_bags, args.title, out_path,
              ylim_pos=args.ylim_pos, ylim_yaw=args.ylim_yaw)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Compare localization sources against ground truth from a rosbag2 bag.

Section 1 — AMCL vs GT (map frame)
    Ground truth: T_map_hero = T_map_odom * T_odom_hero
    Sources: /carla/hero/odometry, /amcl_pose, /tf (map->odom)

Section 2 — odom_wheel vs odom_local (odom frame)
    Ground truth: /odom_local  (derived from CARLA exact odometry)
    Estimate:     /odom_wheel  (speed + IMU dead reckoning)

Two comparison modes (applies to both sections):
    raw            : direct comparison
    offset-correct : subtract initial offset to measure relative drift
"""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CARLA_ODOM_TOPIC = "/carla/hero/odometry"
AMCL_TOPIC       = "/amcl_pose"
ODOM_LOCAL_TOPIC = "/odom_local"
ODOM_WHEEL_TOPIC = "/odom_wheel"


@dataclass
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def compose_pose(a: Pose2D, b: Pose2D, stamp_ns: int) -> Pose2D:
    ca, sa = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        stamp_ns=stamp_ns,
        x=a.x + ca * b.x - sa * b.y,
        y=a.y + sa * b.x + ca * b.y,
        yaw=wrap_angle(a.yaw + b.yaw),
    )


def invert_pose(p: Pose2D, stamp_ns: Optional[int] = None) -> Pose2D:
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    return Pose2D(
        stamp_ns=p.stamp_ns if stamp_ns is None else stamp_ns,
        x=-(c * p.x + s * p.y),
        y=-(-s * p.x + c * p.y),
        yaw=wrap_angle(-p.yaw),
    )


def position_error(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def yaw_error(a: Pose2D, b: Pose2D) -> float:
    return wrap_angle(a.yaw - b.yaw)


def nearest_by_time(poses: Sequence[Pose2D], target_ns: int, max_dt_ns: int) -> Optional[Pose2D]:
    if not poses:
        return None
    stamps = [p.stamp_ns for p in poses]
    idx = bisect.bisect_left(stamps, target_ns)
    candidates: List[Pose2D] = []
    if idx < len(poses):
        candidates.append(poses[idx])
    if idx > 0:
        candidates.append(poses[idx - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda p: abs(p.stamp_ns - target_ns))
    if abs(best.stamp_ns - target_ns) > max_dt_ns:
        return None
    return best


def pose_from_odometry(msg) -> Pose2D:
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return Pose2D(
        stamp_ns=stamp_ns,
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        yaw=quat_to_yaw(msg.pose.pose.orientation),
    )


def read_bag(bag_path: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    def get_type(topic):
        if topic not in topic_types:
            return None
        return get_message(topic_types[topic])

    carla_odom_type = get_type(CARLA_ODOM_TOPIC)
    amcl_type       = get_type(AMCL_TOPIC)
    odom_local_type = get_type(ODOM_LOCAL_TOPIC)
    odom_wheel_type = get_type(ODOM_WHEEL_TOPIC)

    carla_odom_poses: List[Pose2D] = []
    amcl_poses:       List[Pose2D] = []
    odom_local_poses: List[Pose2D] = []
    odom_wheel_poses: List[Pose2D] = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic == CARLA_ODOM_TOPIC and carla_odom_type:
            carla_odom_poses.append(pose_from_odometry(deserialize_message(data, carla_odom_type)))

        elif topic == AMCL_TOPIC and amcl_type:
            msg = deserialize_message(data, amcl_type)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            amcl_poses.append(Pose2D(
                stamp_ns=stamp_ns,
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                yaw=quat_to_yaw(msg.pose.pose.orientation),
            ))

        elif topic == ODOM_LOCAL_TOPIC and odom_local_type:
            odom_local_poses.append(pose_from_odometry(deserialize_message(data, odom_local_type)))

        elif topic == ODOM_WHEEL_TOPIC and odom_wheel_type:
            odom_wheel_poses.append(pose_from_odometry(deserialize_message(data, odom_wheel_type)))

    return carla_odom_poses, amcl_poses, odom_local_poses, odom_wheel_poses


def summarize(values: Sequence[float]) -> Tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    return sum(values) / len(values), max(values)


def print_section(title: str):
    print()
    print("=" * 55)
    print(f"  {title}")
    print("=" * 55)


def compare_poses(
    reference_poses: List[Pose2D],
    estimate_poses: List[Pose2D],
    max_dt_ns: int,
    mode: str,
) -> Tuple[int, float, float, float, float]:
    matched = 0
    pos_errors: List[float] = []
    yaw_errors: List[float] = []
    initial_offset: Optional[Pose2D] = None

    for ref in reference_poses:
        est = nearest_by_time(estimate_poses, ref.stamp_ns, max_dt_ns)
        if est is None:
            continue

        ref_for_compare = ref
        if mode == "offset-correct":
            if initial_offset is None:
                initial_offset = compose_pose(est, invert_pose(ref), ref.stamp_ns)
            ref_for_compare = compose_pose(initial_offset, ref, ref.stamp_ns)

        pos_errors.append(position_error(ref_for_compare, est))
        yaw_errors.append(yaw_error(ref_for_compare, est))
        matched += 1

    avg_pos, max_pos = summarize(pos_errors)
    avg_yaw, max_yaw = summarize([abs(v) for v in yaw_errors])
    return matched, avg_pos, max_pos, avg_yaw, max_yaw


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare localization sources from rosbag2.")
    parser.add_argument("bag", help="Path to rosbag2 folder")
    parser.add_argument(
        "--max-dt", type=float, default=0.20,
        help="Max timestamp diff [s] for matching (default: 0.20)",
    )
    parser.add_argument(
        "--mode", choices=["raw", "offset-correct"], default="offset-correct",
        help="Comparison mode (default: offset-correct)",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    carla_odom_poses, amcl_poses, odom_local_poses, odom_wheel_poses = read_bag(bag_path)
    max_dt_ns = int(args.max_dt * 1_000_000_000)

    print(f"Bag  : {bag_path}")
    print(f"Mode : {args.mode}")

    # ── Section 1: AMCL vs GT (map frame) ───────────────────────────────
    print_section("Section 1: AMCL vs Ground Truth  [map frame]")
    print(f"  /carla/hero/odometry samples : {len(carla_odom_poses)}")
    print(f"  /amcl_pose samples           : {len(amcl_poses)}")

    # /carla/hero/odometry is already in CARLA world frame (= map frame).
    # Compare directly against /amcl_pose using offset-correct to absorb
    # the constant offset between CARLA world origin and ROS map origin.
    matched1, avg_pos1, max_pos1, avg_yaw1, max_yaw1 = compare_poses(
        amcl_poses, carla_odom_poses, max_dt_ns, args.mode
    )
    print(f"  Matched samples              : {matched1}")
    print()
    print(f"  Avg position error : {avg_pos1:.4f} m")
    print(f"  Max position error : {max_pos1:.4f} m")
    print(f"  Avg yaw error      : {avg_yaw1:.4f} rad  ({math.degrees(avg_yaw1):.2f} deg)")
    print(f"  Max yaw error      : {max_yaw1:.4f} rad  ({math.degrees(max_yaw1):.2f} deg)")

    # ── Section 2: odom_wheel vs odom_local (odom frame) ────────────────
    print_section("Section 2: odom_wheel vs odom_local  [odom frame]")
    print(f"  /odom_local samples  : {len(odom_local_poses)}")
    print(f"  /odom_wheel samples  : {len(odom_wheel_poses)}")

    if not odom_local_poses or not odom_wheel_poses:
        print("  [SKIP] One or both topics not found in bag.")
    else:
        matched2, avg_pos2, max_pos2, avg_yaw2, max_yaw2 = compare_poses(
            odom_wheel_poses, odom_local_poses, max_dt_ns, args.mode
        )
        print(f"  Matched samples      : {matched2}")
        print()
        print(f"  Avg position error : {avg_pos2:.4f} m")
        print(f"  Max position error : {max_pos2:.4f} m")
        print(f"  Avg yaw error      : {avg_yaw2:.4f} rad  ({math.degrees(avg_yaw2):.2f} deg)")
        print(f"  Max yaw error      : {max_yaw2:.4f} rad  ({math.degrees(max_yaw2):.2f} deg)")

    print()


if __name__ == "__main__":
    main()

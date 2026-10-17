#!/usr/bin/env python3
"""Diagnose wheel/IMU/EKF disagreement and map->odom correction in a Nav2 bag."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/cmd_vel_safe",
    "/wheel/odom",
    "/odometry/filtered",
    "/imu/data",
    "/tf",
    "/rosout",
}


def yaw(q):
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def compose(one, two):
    x1, y1, a1 = one
    x2, y2, a2 = two
    return (
        x1 + math.cos(a1) * x2 - math.sin(a1) * y2,
        y1 + math.sin(a1) * x2 + math.cos(a1) * y2,
        wrap(a1 + a2),
    )


def integrate(samples, start, end, absolute=False):
    selected = [(t, v) for t, v in samples if start <= t <= end]
    total = 0.0
    for (t1, v1), (t2, v2) in zip(selected, selected[1:]):
        dt = (t2 - t1) * 1.0e-9
        value = 0.5 * (v1 + v2)
        total += (abs(value) if absolute else value) * dt
    return total


def delta_pose(samples, start, end):
    selected = [(t, pose) for t, pose in samples if start <= t <= end]
    if len(selected) < 2:
        return math.nan, math.nan, math.nan
    one = selected[0][1]
    two = selected[-1][1]
    return (
        math.hypot(two[0] - one[0], two[1] - one[1]),
        math.degrees(wrap(two[2] - one[2])),
        sum(
            math.hypot(right[1][0] - left[1][0], right[1][1] - left[1][1])
            for left, right in zip(selected, selected[1:])
        ),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--window", type=float, default=60.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    classes = {topic: get_message(types[topic]) for topic in TOPICS if topic in types}

    cmd = []
    imu = []
    wheel_twist = []
    ekf_twist = []
    wheel_pose = []
    ekf_pose = []
    map_odom = []
    odom_base = []
    map_base = []
    rosout = []
    tf_cache = {}
    first = None
    last = None
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(raw, classes[topic])
        first = timestamp if first is None else min(first, timestamp)
        last = timestamp if last is None else max(last, timestamp)
        if topic == "/cmd_vel_safe":
            cmd.append((timestamp, float(msg.angular.z)))
        elif topic == "/imu/data":
            imu.append((timestamp, float(msg.angular_velocity.z)))
        elif topic in ("/wheel/odom", "/odometry/filtered"):
            pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                yaw(msg.pose.pose.orientation),
            )
            twist = (timestamp, float(msg.twist.twist.angular.z))
            if topic == "/wheel/odom":
                wheel_pose.append((timestamp, pose))
                wheel_twist.append(twist)
            else:
                ekf_pose.append((timestamp, pose))
                ekf_twist.append(twist)
        elif topic == "/tf":
            changed = False
            for item in msg.transforms:
                key = (
                    item.header.frame_id.lstrip("/"),
                    item.child_frame_id.lstrip("/"),
                )
                pose = (
                    float(item.transform.translation.x),
                    float(item.transform.translation.y),
                    yaw(item.transform.rotation),
                )
                tf_cache[key] = pose
                if key == ("map", "odom"):
                    map_odom.append((timestamp, pose))
                    changed = True
                elif key == ("odom", "base_footprint"):
                    odom_base.append((timestamp, pose))
                    changed = True
            if changed and ("map", "odom") in tf_cache and ("odom", "base_footprint") in tf_cache:
                map_base.append(
                    (
                        timestamp,
                        compose(tf_cache[("map", "odom")], tf_cache[("odom", "base_footprint")]),
                    )
                )
        elif topic == "/rosout":
            rosout.append((timestamp, int(msg.level), str(msg.name), str(msg.msg)))

    if first is None or last is None:
        raise SystemExit("no requested messages")
    duration = (last - first) * 1.0e-9
    print(f"bag={args.bag} duration={duration:.2f}s")
    print(
        f"samples cmd={len(cmd)} imu={len(imu)} wheel={len(wheel_pose)} "
        f"ekf={len(ekf_pose)} map_odom={len(map_odom)} map_base={len(map_base)}"
    )
    print("\n===== fixed windows =====")
    offset = 0.0
    while offset < duration:
        end_offset = min(duration, offset + args.window)
        start_ns = first + int(offset * 1.0e9)
        end_ns = first + int(end_offset * 1.0e9)
        wheel_move, wheel_yaw, _ = delta_pose(wheel_pose, start_ns, end_ns)
        ekf_move, ekf_yaw, _ = delta_pose(ekf_pose, start_ns, end_ns)
        map_correction, map_yaw, map_cumulative = delta_pose(map_odom, start_ns, end_ns)
        base_move, base_yaw, base_cumulative = delta_pose(map_base, start_ns, end_ns)
        print(
            f"+{offset:6.1f}..{end_offset:6.1f}s "
            f"yaw_integral_deg cmd={math.degrees(integrate(cmd, start_ns, end_ns)):+7.1f} "
            f"imu={math.degrees(integrate(imu, start_ns, end_ns)):+7.1f} "
            f"wheel={math.degrees(integrate(wheel_twist, start_ns, end_ns)):+7.1f} "
            f"ekf={math.degrees(integrate(ekf_twist, start_ns, end_ns)):+7.1f} | "
            f"pose wheel={wheel_move:.2f}m/{wheel_yaw:+6.1f}deg "
            f"ekf={ekf_move:.2f}m/{ekf_yaw:+6.1f}deg "
            f"map->odom={map_correction:.2f}m/{map_yaw:+6.1f}deg "
            f"map_base={base_move:.2f}m/{base_yaw:+6.1f}deg "
            f"(cum {map_cumulative:.2f}/{base_cumulative:.2f}m)"
        )
        offset = end_offset

    print("\n===== largest consecutive map->odom corrections =====")
    jumps = []
    for left, right in zip(map_odom, map_odom[1:]):
        translation = math.hypot(right[1][0] - left[1][0], right[1][1] - left[1][1])
        angle = abs(math.degrees(wrap(right[1][2] - left[1][2])))
        score = translation + math.radians(angle)
        jumps.append((score, right[0], translation, angle))
    for _, timestamp, translation, angle in sorted(jumps, reverse=True)[:15]:
        print(
            f"+{(timestamp - first) * 1.0e-9:7.2f}s "
            f"step={translation:.3f}m/{angle:.2f}deg"
        )

    print("\n===== SLAM / TF / localization rosout =====")
    keywords = ("slam", "scan", "match", "transform", "tf_", "loop closure", "queue")
    selected = [
        item for item in rosout
        if item[1] >= 30
        or any(word in (item[2] + " " + item[3]).lower() for word in keywords)
    ]
    print(f"selected={len(selected)} of rosout={len(rosout)}")
    for timestamp, level, name, message in selected[-120:]:
        print(
            f"+{(timestamp - first) * 1.0e-9:7.2f}s L{level} {name}: {message}"
        )


if __name__ == "__main__":
    main()

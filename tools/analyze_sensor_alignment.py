#!/usr/bin/env python3
"""Report sensor timestamp health and TF ownership from a ROS 2 bag.

This intentionally separates temporal alignment (message capture stamps) from
spatial alignment (the TF tree).  A plausible URDF transform is not proof that
the physical camera/lidar extrinsics have been calibrated.
"""

from __future__ import annotations

import collections
import math
import statistics
import sys
from bisect import bisect_left
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


SENSOR_TOPICS = {
    "/imu/data",
    "/scan_filtered",
    "/scan_camera",
    "/wheel/odom",
    "/odometry/filtered",
    "/camera/depth/camera_info",
    "/camera/left_ir/camera_info",
    "/camera/right_ir/camera_info",
}


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def nearest_offsets(left: list[int], right: list[int]) -> list[float]:
    if not left or not right:
        return []
    ordered = sorted(right)
    offsets: list[float] = []
    for value in left:
        index = bisect_left(ordered, value)
        choices = []
        if index < len(ordered):
            choices.append(ordered[index])
        if index:
            choices.append(ordered[index - 1])
        nearest = min(choices, key=lambda candidate: abs(candidate - value))
        offsets.append(abs(nearest - value) / 1e6)
    return offsets


def interpolate(series: list[tuple[int, float]], timestamp: int) -> float | None:
    index = bisect_left(series, (timestamp, -math.inf))
    if index == 0 or index >= len(series):
        return None
    left_t, left_v = series[index - 1]
    right_t, right_v = series[index]
    if right_t == left_t or right_t - left_t > 100_000_000:
        return None
    fraction = (timestamp - left_t) / (right_t - left_t)
    return left_v + fraction * (right_v - left_v)


def correlation(left: list[float], right: list[float]) -> float:
    if len(left) < 20 or len(left) != len(right):
        return math.nan
    mean_left = statistics.fmean(left)
    mean_right = statistics.fmean(right)
    numerator = sum(
        (one - mean_left) * (two - mean_right)
        for one, two in zip(left, right)
    )
    denominator = math.sqrt(
        sum((value - mean_left) ** 2 for value in left) *
        sum((value - mean_right) ** 2 for value in right)
    )
    return numerator / denominator if denominator else math.nan


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY", file=sys.stderr)
        return 2

    bag_dir = Path(sys.argv[1])
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    samples: dict[str, list[tuple[int, int, str]]] = collections.defaultdict(list)
    yaw_rates: dict[str, list[tuple[int, float]]] = collections.defaultdict(list)
    tf_edges: dict[str, set[tuple[str, str]]] = collections.defaultdict(set)
    tf_values: dict[tuple[str, str, str], set[tuple[float, ...]]] = collections.defaultdict(set)

    with Reader(bag_dir) as reader:
        connections = [
            connection for connection in reader.connections
            if connection.topic in SENSOR_TOPICS | {"/tf", "/tf_static"}
        ]
        for connection, record_time, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            if connection.topic in SENSOR_TOPICS:
                samples[connection.topic].append(
                    (stamp_ns(msg.header.stamp), int(record_time), str(msg.header.frame_id))
                )
                if connection.topic == "/imu/data":
                    yaw_rates[connection.topic].append(
                        (stamp_ns(msg.header.stamp), float(msg.angular_velocity.z))
                    )
                elif connection.topic in {"/wheel/odom", "/odometry/filtered"}:
                    yaw_rates[connection.topic].append(
                        (stamp_ns(msg.header.stamp), float(msg.twist.twist.angular.z))
                    )
                continue
            for transform in msg.transforms:
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                authority = connection.topic
                tf_edges[child].add((parent, authority))
                tr = transform.transform.translation
                rot = transform.transform.rotation
                tf_values[(parent, child, authority)].add(tuple(round(float(value), 7) for value in (
                    tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w
                )))

    print("===== Topic timestamp health =====")
    for topic in sorted(SENSOR_TOPICS):
        data = samples.get(topic, [])
        if not data:
            print(f"{topic}: MISSING")
            continue
        header_times = [item[0] for item in data]
        record_times = [item[1] for item in data]
        frames = sorted({item[2] for item in data})
        periods = [
            (later - earlier) / 1e6
            for earlier, later in zip(header_times, header_times[1:])
            if later > earlier
        ]
        delays = [(record - header) / 1e6 for header, record, _ in data]
        backward = sum(later <= earlier for earlier, later in zip(header_times, header_times[1:]))
        duration = max(1e-9, (header_times[-1] - header_times[0]) / 1e9)
        print(
            f"{topic}: n={len(data)} rate={max(0, len(data)-1)/duration:.2f}Hz "
            f"period_med/p95={statistics.median(periods):.2f}/{percentile(periods, .95):.2f}ms "
            f"record-header med/p95={statistics.median(delays):.2f}/{percentile(delays, .95):.2f}ms "
            f"non_monotonic={backward} frame={frames}"
        )

    print("===== Nearest capture-stamp separation =====")
    pairs = [
        ("/imu/data", "/wheel/odom"),
        ("/imu/data", "/odometry/filtered"),
        ("/scan_filtered", "/odometry/filtered"),
        ("/scan_camera", "/odometry/filtered"),
        ("/scan_filtered", "/scan_camera"),
        ("/camera/left_ir/camera_info", "/camera/right_ir/camera_info"),
    ]
    for left, right in pairs:
        offsets = nearest_offsets(
            [item[0] for item in samples.get(left, [])],
            [item[0] for item in samples.get(right, [])],
        )
        if offsets:
            print(
                f"{left} -> {right}: abs nearest med/p95/max="
                f"{statistics.median(offsets):.2f}/{percentile(offsets, .95):.2f}/"
                f"{max(offsets):.2f}ms"
            )
        else:
            print(f"{left} -> {right}: MISSING")

    print("===== IMU/wheel yaw-rate phase check =====")
    # The installed IMU is upside down, so base-frame Z is expected to have
    # the opposite sign to raw imu_link Z. Search a small lag window after
    # applying that known spatial sign conversion.
    imu_series = yaw_rates.get("/imu/data", [])
    wheel_series = yaw_rates.get("/wheel/odom", [])
    best: tuple[float, int, int] | None = None
    for lag_ms in range(-300, 301, 10):
        wheel_values: list[float] = []
        imu_values: list[float] = []
        for timestamp, wheel_value in wheel_series:
            if abs(wheel_value) < 0.02:
                continue
            imu_value = interpolate(imu_series, timestamp + lag_ms * 1_000_000)
            if imu_value is None:
                continue
            wheel_values.append(wheel_value)
            imu_values.append(-imu_value)
        value = correlation(wheel_values, imu_values)
        if math.isfinite(value) and (best is None or value > best[0]):
            best = (value, lag_ms, len(wheel_values))
    if best is None:
        print("insufficient rotating samples")
    else:
        print(
            "corr(wheel_z, base-frame imu_z): "
            f"best={best[0]:.3f} at imu timestamp offset {best[1]:+d}ms "
            f"using {best[2]} moving wheel samples"
        )

    print("===== TF parent/transport ownership =====")
    duplicates = 0
    for child in sorted(tf_edges):
        owners = sorted(tf_edges[child])
        print(f"{child} <- {owners}")
        parents = {parent for parent, _ in owners}
        if len(parents) > 1:
            duplicates += 1
            print(f"  ERROR: child has multiple parents: {sorted(parents)}")
    changing_static = [
        key for key, values in tf_values.items()
        if key[2] == "/tf_static" and len(values) > 1
    ]
    print(
        f"TF summary: children={len(tf_edges)} multiple_parent_children={duplicates} "
        f"changing_static_edges={len(changing_static)}"
    )
    print("===== Static TF values (xyz + quaternion) =====")
    for key in sorted(tf_values):
        parent, child, authority = key
        if authority != "/tf_static":
            continue
        for value in sorted(tf_values[key]):
            print(f"{parent} -> {child}: {value}")
    if changing_static:
        print(f"changing static transforms: {changing_static}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

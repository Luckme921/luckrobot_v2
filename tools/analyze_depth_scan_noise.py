#!/usr/bin/env python3
"""Measure persistence and spatial support of near returns in a LaserScan bag."""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


def percentile(values, fraction):
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))]


def longest_true_run(values):
    longest = current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def main():
    if len(sys.argv) not in (2, 3, 4):
        print(
            f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY "
            "[TOPIC=/scan_camera] [NEAR_RANGE_M=0.45]",
            file=sys.stderr,
        )
        return 2

    bag = Path(sys.argv[1])
    topic = sys.argv[2] if len(sys.argv) >= 3 else "/scan_camera"
    near_range = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.45
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    samples = []

    with Reader(bag) as reader:
        connections = [item for item in reader.connections if item.topic == topic]
        for connection, timestamp, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            ranges = [float(value) for value in msg.ranges]
            valid = [
                math.isfinite(value) and msg.range_min <= value <= msg.range_max
                for value in ranges
            ]
            close = [is_valid and value < near_range for is_valid, value in zip(valid, ranges)]
            nearest = min((value for is_valid, value in zip(valid, ranges) if is_valid), default=math.inf)
            front = []
            for index, (is_valid, value) in enumerate(zip(valid, ranges)):
                angle = msg.angle_min + index * msg.angle_increment
                if is_valid and abs(angle) <= math.radians(30.0):
                    front.append(value)
            samples.append(
                (
                    timestamp,
                    nearest,
                    min(front, default=math.inf),
                    sum(close),
                    longest_true_run(close),
                )
            )

    if not samples:
        print(f"{topic}: NO_DATA")
        return 1

    timestamps = [item[0] for item in samples]
    gaps = [(b - a) / 1e9 for a, b in zip(timestamps, timestamps[1:])]
    duration = (timestamps[-1] - timestamps[0]) / 1e9
    nearest = [item[1] for item in samples if math.isfinite(item[1])]
    front = [item[2] for item in samples if math.isfinite(item[2])]
    close_mask = [item[3] > 0 for item in samples]

    segments = []
    begin = None
    for index, is_close in enumerate(close_mask + [False]):
        if is_close and begin is None:
            begin = index
        elif not is_close and begin is not None:
            end = index
            segments.append((begin, end))
            begin = None

    print(f"bag={bag} topic={topic}")
    print(
        f"samples={len(samples)} duration={duration:.2f}s "
        f"rate={len(samples) / max(duration, 1e-9):.2f}Hz "
        f"gap_p95={percentile(gaps, 0.95) * 1000:.1f}ms"
    )
    print(
        "nearest_m: "
        f"p01={percentile(nearest, 0.01):.3f} "
        f"p05={percentile(nearest, 0.05):.3f} "
        f"median={statistics.median(nearest):.3f}"
    )
    print(
        "front30_m: "
        f"p01={percentile(front, 0.01):.3f} "
        f"p05={percentile(front, 0.05):.3f} "
        f"median={statistics.median(front):.3f}"
    )
    print(
        f"near<{near_range:.2f}m: frames={sum(close_mask)}/{len(samples)} "
        f"fraction={sum(close_mask) / len(samples):.3f} segments={len(segments)}"
    )
    ranked_segments = sorted(
        segments,
        key=lambda item: (item[1] - item[0], samples[item[1] - 1][0] - samples[item[0]][0]),
        reverse=True,
    )
    print("longest near-return segments:")
    for number, (begin, end) in enumerate(ranked_segments[:12], 1):
        segment = samples[begin:end]
        segment_duration = (
            (segment[-1][0] - segment[0][0]) / 1e9 + statistics.median(gaps)
            if gaps else 0.0
        )
        print(
            f" {number:02d}: t={(segment[0][0] - timestamps[0]) / 1e9:6.2f}s "
            f"duration={segment_duration:.2f}s frames={len(segment)} "
            f"nearest={min(item[1] for item in segment):.3f}m "
            f"max_close_beams={max(item[3] for item in segment)} "
            f"max_contiguous_beams={max(item[4] for item in segment)}"
        )
    if len(segments) > 12:
        print(f" ... {len(segments) - 12} shorter segments")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

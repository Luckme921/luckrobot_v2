#!/usr/bin/env python3
"""Create a compact, timing-aware summary of an automatic-exploration bag."""

from __future__ import annotations

import math
import re
import statistics
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


GOAL_RE = re.compile(
    r"Begin navigating from current location \(([-+0-9.]+), ([-+0-9.]+)\) "
    r"to \(([-+0-9.]+), ([-+0-9.]+)\)"
)
LOCK_RE = re.compile(
    r"V[\d.]+ goal locked: (.+?) known-white goal "
    r"\(([-+0-9.]+), ([-+0-9.]+)\).*frontier "
    r"\(([-+0-9.]+), ([-+0-9.]+)\)(.*)"
)
PATH_COST_RE = re.compile(r"(?:path cost|global cost) ([0-9]+)")


@dataclass
class Goal:
    start_ns: int
    start_x: float
    start_y: float
    goal_x: float
    goal_y: float
    end_ns: int = 0
    success_ns: int = 0


def nearest_grid(records, timestamp):
    before = [item for item in records.get("/map", []) if item[0] <= timestamp]
    return before[-1][1] if before else None


def map_counts(grid):
    if grid is None:
        return (0, 0, 0)
    unknown = sum(int(value) < 0 for value in grid.data)
    occupied = sum(int(value) >= 65 for value in grid.data)
    free = len(grid.data) - unknown - occupied
    return (unknown, free, occupied)


def bounded_unknown_area(grid, x, y, radius_m=1.5):
    """Count UNKNOWN connected to the frontier in a bounded local basin."""
    if grid is None:
        return 0.0
    resolution = float(grid.info.resolution)
    width = int(grid.info.width)
    height = int(grid.info.height)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    cx = math.floor((x - ox) / resolution)
    cy = math.floor((y - oy) / resolution)
    radius = max(1, math.ceil(radius_m / resolution))

    seeds = []
    for search_radius in range(0, 7):
        for gy in range(max(0, cy - search_radius), min(height, cy + search_radius + 1)):
            for gx in range(max(0, cx - search_radius), min(width, cx + search_radius + 1)):
                if max(abs(gx - cx), abs(gy - cy)) != search_radius:
                    continue
                if int(grid.data[gy * width + gx]) < 0:
                    seeds.append((gx, gy))
        if seeds:
            break
    if not seeds:
        return 0.0

    queue = deque(seeds)
    visited = set(seeds)
    while queue:
        gx, gy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = gx + dx, gy + dy
            if (nx, ny) in visited or not (0 <= nx < width and 0 <= ny < height):
                continue
            if (nx - cx) ** 2 + (ny - cy) ** 2 > radius ** 2:
                continue
            if int(grid.data[ny * width + nx]) >= 0:
                continue
            visited.add((nx, ny))
            queue.append((nx, ny))
    return len(visited) * resolution * resolution


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def yaw_from_orientation(orientation):
    """Return planar yaw for a geometry_msgs quaternion."""
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def odom_motion(samples):
    """Measure translation and accumulated yaw during a goal interval."""
    if len(samples) < 2:
        return (0.0, 0.0, 0.0)
    positions = [
        (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        for _, msg in samples
    ]
    yaws = [yaw_from_orientation(msg.pose.pose.orientation) for _, msg in samples]
    translation = sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(positions, positions[1:])
    )
    accumulated_yaw = sum(
        abs(wrapped_angle(after - before))
        for before, after in zip(yaws, yaws[1:])
    )
    net_yaw = abs(wrapped_angle(yaws[-1] - yaws[0]))
    return (translation, accumulated_yaw, net_yaw)


def main():
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY", file=sys.stderr)
        return 2
    bag_dir = Path(sys.argv[1])
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    wanted = {
        "/rosout", "/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe",
        "/wheel/odom", "/odometry/filtered", "/map",
    }
    records = {}
    with Reader(bag_dir) as reader:
        connections = [connection for connection in reader.connections if connection.topic in wanted]
        start_ns, end_ns = reader.start_time, reader.end_time
        for connection, timestamp, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            records.setdefault(connection.topic, []).append((timestamp, msg))

    goals = []
    locks = []
    dfs_counts = {
        "remember": 0,
        "complete": 0,
        "backtrack": 0,
        "direct_transit": 0,
        "reject": 0,
    }
    staging_rejects = 0
    path_reject_costs = []
    for timestamp, log in records.get("/rosout", []):
        match = GOAL_RE.search(log.msg)
        if match:
            sx, sy, gx, gy = map(float, match.groups())
            goals.append(Goal(timestamp, sx, sy, gx, gy))
        lock = LOCK_RE.search(log.msg)
        if lock:
            kind, gx, gy, fx, fy, suffix = lock.groups()
            grid = nearest_grid(records, timestamp)
            locks.append((timestamp, kind, float(gx), float(gy), float(fx), float(fy), suffix,
                          bounded_unknown_area(grid, float(fx), float(fy))))
        if "DFS remembered" in log.msg:
            dfs_counts["remember"] += 1
        if "DFS branch #" in log.msg and "complete" in log.msg:
            dfs_counts["complete"] += 1
        if "DFS backtrack" in log.msg or "returning to parent junction" in log.msg:
            dfs_counts["backtrack"] += 1
        if "DFS direct transit" in log.msg:
            dfs_counts["direct_transit"] += 1
        if "Rejecting branch" in log.msg or "Rejecting junction" in log.msg:
            dfs_counts["reject"] += 1
            path_cost = PATH_COST_RE.search(log.msg)
            if path_cost:
                path_reject_costs.append(int(path_cost.group(1)))
        if "Skipping frontier" in log.msg and "no staging point" in log.msg:
            staging_rejects += 1

    for index, goal in enumerate(goals):
        goal.end_ns = goals[index + 1].start_ns if index + 1 < len(goals) else end_ns
        successes = [timestamp for timestamp, log in records.get("/rosout", [])
                     if goal.start_ns <= timestamp < goal.end_ns and "Reached the goal!" in log.msg]
        goal.success_ns = successes[0] if successes else 0

    initial = records.get("/map", [(0, None)])[0][1]
    final = records.get("/map", [(0, None)])[-1][1]
    ic = map_counts(initial)
    fc = map_counts(final)
    resolution = float(final.info.resolution) if final is not None else 0.05
    wheel = records.get("/wheel/odom", [])
    traveled = 0.0
    if wheel:
        points = [(float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)) for _, msg in wheel]
        traveled = sum(math.hypot(bx-ax, by-ay) for (ax, ay), (bx, by) in zip(points, points[1:]))

    print(f"bag={bag_dir}")
    print(f"duration={(end_ns-start_ns)/1e9:.1f}s goals={len(goals)} wheel_travel={traveled:.2f}m")
    print(f"map unknown/free/occupied: {ic} -> {fc}; newly_known={(ic[0]-fc[0])*resolution*resolution:.2f}m2")
    print("DFS events: " + " ".join(f"{key}={value}" for key, value in dfs_counts.items()))
    efficiency = ((ic[0] - fc[0]) * resolution * resolution / traveled
                  if traveled > 0.0 else 0.0)
    print(f"exploration_efficiency={efficiency:.2f}m2/m staging_reject_logs={staging_rejects}")
    if path_reject_costs:
        print(
            "rejected_path_costs: "
            f"min={min(path_reject_costs)} median={statistics.median(path_reject_costs):.1f} "
            f"max={max(path_reject_costs)} count={len(path_reject_costs)}"
        )
    print("\nGoal timeline:")
    previous_success = 0
    for index, goal in enumerate(goals, 1):
        interval_end = goal.success_ns or goal.end_ns
        twists = [(timestamp, msg) for timestamp, msg in records.get("/cmd_vel_safe", [])
                  if goal.start_ns <= timestamp <= interval_end]
        moving = [timestamp for timestamp, msg in twists
                  if abs(float(msg.linear.x)) > 0.005 or abs(float(msg.angular.z)) > 0.02]
        linear = [timestamp for timestamp, msg in twists if abs(float(msg.linear.x)) > 0.005]
        pure_rotate = [timestamp for timestamp, msg in twists
                       if abs(float(msg.linear.x)) <= 0.005 and abs(float(msg.angular.z)) > 0.02]
        angular_commands = [
            abs(float(msg.angular.z)) for _, msg in twists
            if abs(float(msg.angular.z)) > 0.02
        ]
        wheel_interval = [
            item for item in records.get("/wheel/odom", [])
            if goal.start_ns <= item[0] <= interval_end
        ]
        filtered_interval = [
            item for item in records.get("/odometry/filtered", [])
            if goal.start_ns <= item[0] <= interval_end
        ]
        wheel_translation, wheel_yaw, wheel_net_yaw = odom_motion(wheel_interval)
        filtered_translation, filtered_yaw, filtered_net_yaw = odom_motion(filtered_interval)
        nav_time = (interval_end-goal.start_ns)/1e9
        stationary_tail = ((interval_end-linear[-1])/1e9 if linear else nav_time)
        handoff = ((goal.start_ns-previous_success)/1e9 if previous_success else 0.0)
        previous_success = goal.success_ns or previous_success
        print(
            f" {index:02d} ({goal.start_x:+.2f},{goal.start_y:+.2f})->"
            f"({goal.goal_x:+.2f},{goal.goal_y:+.2f}) "
            f"nav={nav_time:5.1f}s handoff={handoff:4.1f}s "
            f"stationary_tail={stationary_tail:4.1f}s "
            f"moving_samples={len(moving)} pure_rotate={len(pure_rotate)} "
            f"cmd_w_mean={(statistics.mean(angular_commands) if angular_commands else 0.0):.3f} "
            f"wheel_d={wheel_translation:.2f}m/yaw={math.degrees(wheel_yaw):.0f}deg "
            f"(net {math.degrees(wheel_net_yaw):.0f}) "
            f"ekf_d={filtered_translation:.2f}m/yaw={math.degrees(filtered_yaw):.0f}deg "
            f"(net {math.degrees(filtered_net_yaw):.0f}) "
            f"status={'OK' if goal.success_ns else 'OPEN'}"
        )

    print("\nLocked exploration goals and bounded connected UNKNOWN gain (1.5m):")
    for timestamp, kind, gx, gy, fx, fy, suffix, gain in locks:
        print(f" t={(timestamp-start_ns)/1e9:6.1f}s {kind:13s} goal=({gx:+.2f},{gy:+.2f}) "
              f"frontier=({fx:+.2f},{fy:+.2f}) unknown_gain={gain:.2f}m2{suffix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Measure path-heading alignment before translation for each Nav2 goal."""

from __future__ import annotations

import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


BEGIN_RE = re.compile(
    r"Begin navigating from current location \(([-+0-9.]+), ([-+0-9.]+)\) "
    r"to \(([-+0-9.]+), ([-+0-9.]+)\)"
)


def yaw(q):
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, wrap(ath + bth)


@dataclass
class Goal:
    time: int
    sx: float
    sy: float
    gx: float
    gy: float


def nearest_before(samples, when):
    selected = None
    for sample in samples:
        if sample[0] > when:
            break
        selected = sample
    return selected


def first_path_heading(points, start, lookahead=0.25):
    if len(points) < 2:
        return math.nan
    nearest = min(range(len(points)), key=lambda i: math.hypot(points[i][0] - start[0], points[i][1] - start[1]))
    origin = points[nearest]
    index = nearest + 1
    while index < len(points) and math.hypot(points[index][0] - origin[0], points[index][1] - origin[1]) < lookahead:
        index += 1
    if index >= len(points):
        index = len(points) - 1
    return math.atan2(points[index][1] - origin[1], points[index][0] - origin[0])


def main():
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY", file=sys.stderr)
        return 2
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    topics = {"/rosout", "/tf", "/plan", "/cmd_vel_safe"}
    logs, poses, plans, commands = [], [], [], []
    transforms = {}
    with Reader(Path(sys.argv[1])) as reader:
        start_ns, end_ns = reader.start_time, reader.end_time
        connections = [c for c in reader.connections if c.topic in topics]
        last_pose_ns = 0
        for connection, timestamp, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            if connection.topic == "/rosout":
                logs.append((timestamp, msg.msg))
            elif connection.topic == "/plan":
                plans.append((timestamp, [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]))
            elif connection.topic == "/cmd_vel_safe":
                commands.append((timestamp, float(msg.linear.x), float(msg.angular.z)))
            else:
                for item in msg.transforms:
                    transforms[(item.header.frame_id.lstrip("/"), item.child_frame_id.lstrip("/"))] = (
                        float(item.transform.translation.x),
                        float(item.transform.translation.y),
                        yaw(item.transform.rotation),
                    )
                if timestamp - last_pose_ns >= 20_000_000:
                    map_odom = transforms.get(("map", "odom"))
                    odom_base = transforms.get(("odom", "base_footprint"))
                    if map_odom and odom_base:
                        poses.append((timestamp, *compose(map_odom, odom_base)))
                        last_pose_ns = timestamp

    goals = []
    for timestamp, message in logs:
        match = BEGIN_RE.search(message)
        if match:
            goals.append(Goal(timestamp, *map(float, match.groups())))

    print(f"bag={sys.argv[1]} goals={len(goals)}")
    for number, goal in enumerate(goals, 1):
        stop = goals[number].time if number < len(goals) else end_ns
        start_pose_sample = nearest_before(poses, goal.time)
        if not start_pose_sample:
            continue
        _, px, py, start_yaw = start_pose_sample
        plan_samples = [(t, p) for t, p in plans if goal.time <= t < stop and len(p) >= 2]
        plan_time, plan = plan_samples[0] if plan_samples else (0, [])
        plan_heading = first_path_heading(plan, (px, py)) if plan else math.nan
        goal_bearing = math.atan2(goal.gy - py, goal.gx - px)
        initial_error = abs(wrap(plan_heading - start_yaw)) if math.isfinite(plan_heading) else math.nan
        goal_error = abs(wrap(goal_bearing - start_yaw))
        linear = next((item for item in commands if max(goal.time, plan_time) <= item[0] < stop and abs(item[1]) > 0.01), None)
        interval_commands = [item for item in commands if goal.time <= item[0] < stop and abs(item[2]) > 0.02]
        angular_signs = [1 if item[2] > 0.0 else -1 for item in interval_commands]
        sign_changes = sum(one != two for one, two in zip(angular_signs, angular_signs[1:]))
        positive = sum(sign > 0 for sign in angular_signs)
        negative = sum(sign < 0 for sign in angular_signs)
        interval_plans = [(t, p) for t, p in plans if goal.time <= t < stop and len(p) >= 2]
        plan_headings = [first_path_heading(points, (px, py)) for _, points in interval_plans]
        plan_heading_span = 0.0
        if plan_headings:
            plan_heading_span = math.degrees(max(abs(wrap(value - plan_headings[0])) for value in plan_headings))
        if linear:
            linear_pose = nearest_before(poses, linear[0])
            linear_yaw = linear_pose[3] if linear_pose else start_yaw
            residual = abs(wrap(plan_heading - linear_yaw)) if math.isfinite(plan_heading) else math.nan
            delay = (linear[0] - max(goal.time, plan_time)) / 1e9
            first_cmd = f"v={linear[1]:.3f}/w={linear[2]:.3f}"
        else:
            residual, delay, first_cmd = math.nan, math.nan, "none"

        # Direction of the first 0.20 m of physical displacement.
        motion_heading = math.nan
        for timestamp, x, y, _ in poses:
            if timestamp < goal.time:
                continue
            if timestamp >= stop:
                break
            if math.hypot(x - px, y - py) >= 0.20:
                motion_heading = math.atan2(y - py, x - px)
                break
        track_error = abs(wrap(motion_heading - plan_heading)) if math.isfinite(motion_heading) and math.isfinite(plan_heading) else math.nan
        print(
            f" {number:02d} ({goal.sx:+.2f},{goal.sy:+.2f})->({goal.gx:+.2f},{goal.gy:+.2f}) "
            f"plan_start_err={math.degrees(initial_error):5.1f}deg "
            f"goal_bearing_err={math.degrees(goal_error):5.1f}deg "
            f"linear_delay={delay:4.1f}s residual={math.degrees(residual):5.1f}deg "
            f"first={first_cmd} first20cm_track_err={math.degrees(track_error):5.1f}deg "
            f"w_sign=+{positive}/-{negative}/flips{sign_changes} plan_span={plan_heading_span:4.0f}deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

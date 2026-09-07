#!/usr/bin/env python3
"""Summarize LuckController tracking, command oscillation and path replanning."""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def sign_runs(values: list[float], threshold: float = 0.04, minimum: int = 3):
    raw: list[tuple[int, int]] = []
    for value in values:
        sign = 1 if value > threshold else -1 if value < -threshold else 0
        if sign == 0:
            continue
        if raw and raw[-1][0] == sign:
            raw[-1] = (sign, raw[-1][1] + 1)
        else:
            raw.append((sign, 1))
    stable = [item for item in raw if item[1] >= minimum]
    return stable, max(0, len(stable) - 1)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def unwrap_delta(one: float, two: float) -> float:
    return math.atan2(math.sin(two - one), math.cos(two - one))


def compose_pose(one, two):
    x1, y1, yaw1 = one
    x2, y2, yaw2 = two
    cosine = math.cos(yaw1)
    sine = math.sin(yaw1)
    return (
        x1 + cosine * x2 - sine * y2,
        y1 + sine * x2 + cosine * y2,
        math.atan2(math.sin(yaw1 + yaw2), math.cos(yaw1 + yaw2)),
    )


@dataclass
class Tracking:
    timestamp: int
    distance: float
    carrot_angle: float
    cross_track: float
    curvature: float
    linear: float
    angular: float
    cost: float
    rotate: bool
    goal_yaw_error: float = float("nan")
    final_heading: bool = False
    active_position_tolerance: float = float("nan")


def tracking_episodes(samples: list[Tracking]) -> list[list[Tracking]]:
    if not samples:
        return []
    episodes: list[list[Tracking]] = [[samples[0]]]
    for previous, current in zip(samples, samples[1:]):
        gap = (current.timestamp - previous.timestamp) / 1e9
        # A new RViz goal normally causes an immediate distance jump. Small
        # increases are expected from localization noise and replanning.
        new_goal = gap > 1.0 or current.distance - previous.distance > 0.30
        if new_goal:
            episodes.append([])
        episodes[-1].append(current)
    return episodes


def path_length(path) -> float:
    points = [pose.pose.position for pose in path.poses]
    return sum(
        math.hypot(float(b.x - a.x), float(b.y - a.y))
        for a, b in zip(points, points[1:])
    )


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY", file=sys.stderr)
        return 2

    bag_dir = Path(sys.argv[1])
    wanted = {
        "/luck_controller/tracking_error",
        "/cmd_vel_nav",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/wheel/odom",
        "/plan",
        "/rosout",
        "/tf",
    }
    records: dict[str, list[tuple[int, object]]] = {}
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with Reader(bag_dir) as reader:
        start_ns = reader.start_time
        end_ns = reader.end_time
        connections = [item for item in reader.connections if item.topic in wanted]
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = typestore.deserialize_cdr(raw, connection.msgtype)
            records.setdefault(connection.topic, []).append((timestamp, message))

    tracking = []
    for timestamp, message in records.get("/luck_controller/tracking_error", []):
        if len(message.data) < 8:
            continue
        extra = list(map(float, message.data[8:11]))
        extra += [float("nan")] * (3 - len(extra))
        tracking.append(
            Tracking(
                timestamp,
                *map(float, message.data[:7]),
                bool(round(float(message.data[7]))),
                extra[0],
                bool(round(extra[1])) if math.isfinite(extra[1]) else False,
                extra[2],
            )
        )
    print(f"bag: {bag_dir}")
    print(f"duration: {(end_ns - start_ns) / 1e9:.2f}s; tracking samples: {len(tracking)}")

    for topic in ("/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe"):
        messages = [message for _, message in records.get(topic, [])]
        linear = [float(message.linear.x) for message in messages]
        angular = [float(message.angular.z) for message in messages]
        runs, changes = sign_runs(angular)
        print(
            f"{topic}: n={len(messages)}, |v| p50/p90/max="
            f"{percentile([abs(v) for v in linear], .50):.3f}/"
            f"{percentile([abs(v) for v in linear], .90):.3f}/"
            f"{max([abs(v) for v in linear], default=0):.3f}, "
            f"|w| p50/p90/max={percentile([abs(w) for w in angular], .50):.3f}/"
            f"{percentile([abs(w) for w in angular], .90):.3f}/"
            f"{max([abs(w) for w in angular], default=0):.3f}, "
            f"stable direction changes={changes}, runs={runs[:14]}"
        )

    # Pair by timestamp. Zipping by array index is invalid because the relay
    # topics can start at different times and have occasional dropped samples.
    before = records.get("/cmd_vel", [])
    after = records.get("/cmd_vel_safe", [])
    changed = 0
    paired = 0
    after_index = 0
    for timestamp, message in before:
        while (
            after_index + 1 < len(after)
            and abs(after[after_index + 1][0] - timestamp)
            <= abs(after[after_index][0] - timestamp)
        ):
            after_index += 1
        if after and abs(after[after_index][0] - timestamp) <= 80_000_000:
            safe = after[after_index][1]
            paired += 1
            changed += (
                abs(float(message.linear.x - safe.linear.x)) > 1e-4
                or abs(float(message.angular.z - safe.angular.z)) > 1e-4
            )
    print(f"collision monitor changed timestamp-paired commands: {changed}/{paired}")

    odom = records.get("/wheel/odom", [])
    if len(odom) >= 2:
        path = 0.0
        yaw_total = 0.0
        yaw_signed = 0.0
        previous = odom[0][1]
        previous_yaw = yaw_from_quaternion(previous.pose.pose.orientation)
        for _, message in odom[1:]:
            path += math.hypot(
                float(message.pose.pose.position.x - previous.pose.pose.position.x),
                float(message.pose.pose.position.y - previous.pose.pose.position.y),
            )
            current_yaw = yaw_from_quaternion(message.pose.pose.orientation)
            delta = unwrap_delta(previous_yaw, current_yaw)
            yaw_total += abs(delta)
            yaw_signed += delta
            previous = message
            previous_yaw = current_yaw
        print(
            f"wheel odom: path={path:.3f}m, |yaw travel|={math.degrees(yaw_total):.1f}deg, "
            f"net yaw={math.degrees(yaw_signed):.1f}deg"
        )

    plans = records.get("/plan", [])
    if plans:
        lengths = [path_length(message) for _, message in plans]
        headings = []
        for _, message in plans:
            if len(message.poses) >= 2:
                one = message.poses[0].pose.position
                two = message.poses[min(5, len(message.poses) - 1)].pose.position
                headings.append(math.atan2(float(two.y - one.y), float(two.x - one.x)))
        heading_changes = [
            abs(unwrap_delta(one, two)) for one, two in zip(headings, headings[1:])
        ]
        print(
            f"plans: n={len(plans)}, length min/p50/max="
            f"{min(lengths):.3f}/{statistics.median(lengths):.3f}/{max(lengths):.3f}m, "
            f"successive initial-heading change p90/max="
            f"{math.degrees(percentile(heading_changes, .90)):.1f}/"
            f"{math.degrees(max(heading_changes, default=0)):.1f}deg"
        )

    # Reconstruct map->base pose and compare the requested final orientation
    # with the controller's turn direction. This distinguishes a sign/frame
    # error from an under-damped angular loop.
    map_poses: list[tuple[int, tuple[float, float, float]]] = []
    transforms = {}
    for timestamp, message in records.get("/tf", []):
        for item in message.transforms:
            key = (item.header.frame_id.lstrip("/"), item.child_frame_id.lstrip("/"))
            transforms[key] = (
                float(item.transform.translation.x),
                float(item.transform.translation.y),
                yaw_from_quaternion(item.transform.rotation),
            )
        map_odom = transforms.get(("map", "odom"))
        odom_base = transforms.get(("odom", "base_footprint"))
        if map_odom is not None and odom_base is not None:
            map_poses.append((timestamp, compose_pose(map_odom, odom_base)))

    if map_poses and plans and tracking:
        pose_index = 0
        plan_index = 0
        orientation_errors = []
        direction_mismatch = 0
        comparable = 0
        for sample in tracking:
            while pose_index + 1 < len(map_poses) and map_poses[pose_index + 1][0] <= sample.timestamp:
                pose_index += 1
            while plan_index + 1 < len(plans) and plans[plan_index + 1][0] <= sample.timestamp:
                plan_index += 1
            plan_message = plans[plan_index][1]
            if not plan_message.poses:
                continue
            robot_yaw = map_poses[pose_index][1][2]
            goal_yaw = yaw_from_quaternion(plan_message.poses[-1].pose.orientation)
            error = unwrap_delta(robot_yaw, goal_yaw)
            orientation_errors.append(abs(error))
            if abs(error) > 0.08 and abs(sample.angular) > 0.04:
                comparable += 1
                direction_mismatch += (error > 0.0) != (sample.angular > 0.0)
        if orientation_errors:
            print(
                f"final-yaw error |e| p50/p90/max="
                f"{math.degrees(percentile(orientation_errors, .50)):.1f}/"
                f"{math.degrees(percentile(orientation_errors, .90)):.1f}/"
                f"{math.degrees(max(orientation_errors)):.1f}deg; "
                f"command/error sign mismatch={direction_mismatch}/{comparable}"
            )

    for number, episode in enumerate(tracking_episodes(tracking), 1):
        angular = [sample.angular for sample in episode]
        carrot = [abs(sample.carrot_angle) for sample in episode]
        cross = [abs(sample.cross_track) for sample in episode]
        runs, changes = sign_runs(angular)
        stopped = sum(
            abs(sample.linear) < 1e-4 and abs(sample.angular) < 1e-4
            for sample in episode
        )
        near = [sample for sample in episode if sample.distance <= 0.20]
        near_runs, near_changes = sign_runs([sample.angular for sample in near])
        final_heading = [sample for sample in episode if sample.final_heading]
        print(
            f"episode {number}: {(episode[-1].timestamp - episode[0].timestamp) / 1e9:.1f}s, "
            f"distance {episode[0].distance:.3f}->{episode[-1].distance:.3f}m "
            f"(min {min(sample.distance for sample in episode):.3f}), "
            f"rotate={sum(sample.rotate for sample in episode) / len(episode):.1%}, "
            f"zero={stopped / len(episode):.1%}, angular changes={changes}, "
            f"|carrot angle| p90/max={math.degrees(percentile(carrot, .90)):.1f}/"
            f"{math.degrees(max(carrot)):.1f}deg, "
            f"|cross track| p90/max={percentile(cross, .90):.3f}/{max(cross):.3f}m"
        )
        if near:
            print(
                f"  within 0.20m: n={len(near)}, distance final={near[-1].distance:.3f}m, "
                f"angular changes={near_changes}, runs={near_runs[:12]}, "
                f"|w| p90={percentile([abs(sample.angular) for sample in near], .90):.3f}"
            )
        if final_heading:
            finite_yaw_errors = [
                abs(sample.goal_yaw_error)
                for sample in final_heading
                if math.isfinite(sample.goal_yaw_error)
            ]
            print(
                f"  final heading phase: n={len(final_heading)}, "
                f"distance min/max={min(s.distance for s in final_heading):.3f}/"
                f"{max(s.distance for s in final_heading):.3f}m, "
                f"|yaw error| final="
                f"{math.degrees(finite_yaw_errors[-1]) if finite_yaw_errors else 0.0:.1f}deg"
            )

    warnings: dict[str, int] = {}
    for _, message in records.get("/rosout", []):
        text = str(message.msg)
        if int(message.level) >= 30:
            key = text
            if "Behavior Tree tick rate" in text:
                key = "Behavior Tree tick rate exceeded"
            elif "Failed to meet update rate" in text:
                key = "EKF failed to meet update rate"
            warnings[key] = warnings.get(key, 0) + 1
    print("warnings/errors:")
    for message, count in sorted(warnings.items(), key=lambda item: -item[1])[:12]:
        print(f"  {count:4d}  {message}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

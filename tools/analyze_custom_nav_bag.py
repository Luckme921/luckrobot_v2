#!/usr/bin/env python3
"""Summarize LuckController tracking, command oscillation and path replanning."""

from __future__ import annotations

import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


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
    avoidance_state: int = 0
    valid_candidate_count: int = 0
    selected_trajectory_cost: float = float("nan")
    selected_center_cost: float = float("nan")
    blocked_duration: float = 0.0
    path_heading_error: float = float("nan")
    filtered_heading_error: float = float("nan")
    pure_pursuit_angular: float = float("nan")
    heading_correction: float = float("nan")
    heading_damping: float = float("nan")
    effective_lookahead: float = float("nan")
    lookahead_bend: float = float("nan")


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


def tracking_point(points, robot_xy, lookahead):
    nearest = min(
        range(len(points)),
        key=lambda index: math.hypot(
            points[index][0] - robot_xy[0], points[index][1] - robot_xy[1]),
    )
    accumulated = 0.0
    result = points[nearest]
    for one, two in zip(points[nearest:], points[nearest + 1:]):
        segment = math.hypot(two[0] - one[0], two[1] - one[1])
        if segment > 1.0e-6 and accumulated + segment >= lookahead:
            ratio = (lookahead - accumulated) / segment
            return (
                one[0] + ratio * (two[0] - one[0]),
                one[1] + ratio * (two[1] - one[1]),
            )
        accumulated += segment
        result = two
    return result


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
        "/wheel_feedback",
        "/wheel/odom",
        "/odometry/filtered",
        "/imu/data",
        "/scan_filtered",
        "/plan",
        "/rosout",
        "/tf",
    }
    records: dict[str, list[tuple[int, object]]] = {}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_classes = {
        topic: get_message(msg_type)
        for topic, msg_type in topic_types.items()
        if topic in wanted
    }
    start_ns: int | None = None
    end_ns: int | None = None
    while reader.has_next():
        topic, raw, timestamp = reader.read_next()
        if topic not in message_classes:
            continue
        message = deserialize_message(raw, message_classes[topic])
        records.setdefault(topic, []).append((timestamp, message))
        start_ns = timestamp if start_ns is None else min(start_ns, timestamp)
        end_ns = timestamp if end_ns is None else max(end_ns, timestamp)
    if start_ns is None or end_ns is None:
        print("bag has no readable requested topics", file=sys.stderr)
        return 3

    tracking = []
    for timestamp, message in records.get("/luck_controller/tracking_error", []):
        if len(message.data) < 8:
            continue
        extra = list(map(float, message.data[8:]))
        # V11.2 ends at index 14 (blocked duration). V11.3 inserts predicted
        # center cost at index 14 and moves blocked duration to index 15.
        selected_center_cost = extra[6] if len(extra) >= 8 else float("nan")
        blocked_duration = extra[7] if len(extra) >= 8 else (
            extra[6] if len(extra) >= 7 else 0.0
        )
        extra += [float("nan")] * (15 - len(extra))
        tracking.append(
            Tracking(
                timestamp,
                *map(float, message.data[:7]),
                bool(round(float(message.data[7]))),
                extra[0],
                bool(round(extra[1])) if math.isfinite(extra[1]) else False,
                extra[2],
                int(round(extra[3])) if math.isfinite(extra[3]) else 0,
                int(round(extra[4])) if math.isfinite(extra[4]) else 0,
                extra[5],
                selected_center_cost,
                blocked_duration,
                extra[8],
                extra[9],
                extra[10],
                extra[11],
                extra[12],
                extra[13],
                extra[14],
            )
        )
    print(f"bag: {bag_dir}")
    print(f"duration: {(end_ns - start_ns) / 1e9:.2f}s; tracking samples: {len(tracking)}")
    for topic in ("/odometry/filtered", "/imu/data", "/scan_filtered"):
        timestamps = [timestamp for timestamp, _ in records.get(topic, [])]
        periods_ms = [
            (two - one) / 1e6 for one, two in zip(timestamps, timestamps[1:])
            if two > one
        ]
        if periods_ms:
            print(
                f"{topic} arrival: n={len(timestamps)}, "
                f"period p50/p95/max={percentile(periods_ms, .50):.1f}/"
                f"{percentile(periods_ms, .95):.1f}/{max(periods_ms):.1f}ms, "
                f"rate~={1000.0 / statistics.median(periods_ms):.1f}Hz"
            )
    selected_costs = [
        sample.selected_trajectory_cost
        for sample in tracking
        if math.isfinite(sample.selected_trajectory_cost)
    ]
    if selected_costs:
        print(
            "selected trajectory footprint cost: "
            f"p90/max={percentile(selected_costs, .90):.0f}/{max(selected_costs):.0f}, "
            f"inscribed-or-worse(>=253)={sum(value >= 253 for value in selected_costs)}"
        )
    current_center_costs = [sample.cost for sample in tracking if math.isfinite(sample.cost)]
    if current_center_costs:
        print(
            "current robot-center cost: "
            f"p90/max={percentile(current_center_costs, .90):.0f}/"
            f"{max(current_center_costs):.0f}, "
            f"inscribed-or-worse(>=253)={sum(value >= 253 for value in current_center_costs)}"
        )
    predicted_center_costs = [
        sample.selected_center_cost
        for sample in tracking
        if math.isfinite(sample.selected_center_cost)
    ]
    if predicted_center_costs:
        print(
            "selected trajectory center cost: "
            f"p90/max={percentile(predicted_center_costs, .90):.0f}/"
            f"{max(predicted_center_costs):.0f}, "
            f"inscribed-or-worse(>=253)={sum(value >= 253 for value in predicted_center_costs)}"
        )

    corner_unwind_cycles = sum(sample.avoidance_state == 4 for sample in tracking)
    corner_unwind_timeouts = sum(sample.avoidance_state == 5 for sample in tracking)
    if corner_unwind_cycles or corner_unwind_timeouts:
        print(
            "bounded corner unwind: "
            f"cycles={corner_unwind_cycles}, timeouts={corner_unwind_timeouts}"
        )
    execution_watchdog_cycles = sum(sample.avoidance_state == 6 for sample in tracking)
    if execution_watchdog_cycles:
        print(f"execution-stall watchdog failures: cycles={execution_watchdog_cycles}")
    rotation_breakaway_cycles = sum(sample.avoidance_state == 8 for sample in tracking)
    if rotation_breakaway_cycles:
        print(f"IMU-confirmed rotation breakaway: cycles={rotation_breakaway_cycles}")
    reverse_escape_cycles = sum(sample.avoidance_state == 7 for sample in tracking)
    if reverse_escape_cycles:
        print(f"bounded safe-reverse escape: cycles={reverse_escape_cycles}")

    adaptive_samples = [
        sample for sample in tracking
        if math.isfinite(sample.effective_lookahead)
        and math.isfinite(sample.lookahead_bend)
    ]
    if adaptive_samples:
        active = [
            sample for sample in adaptive_samples
            if sample.effective_lookahead < 0.349
        ]
        print(
            "V11.8 adaptive corner lookahead: "
            f"active={len(active)}/{len(adaptive_samples)}, "
            f"lookahead min/p10={min(sample.effective_lookahead for sample in adaptive_samples):.3f}/"
            f"{percentile([sample.effective_lookahead for sample in adaptive_samples], .10):.3f}m, "
            f"bend p90/max="
            f"{math.degrees(percentile([sample.lookahead_bend for sample in adaptive_samples], .90)):.1f}/"
            f"{math.degrees(max(sample.lookahead_bend for sample in adaptive_samples)):.1f}deg"
        )

    filtered_samples = [
        sample for sample in tracking
        if math.isfinite(sample.path_heading_error)
        and math.isfinite(sample.filtered_heading_error)
    ]
    if filtered_samples:
        tangent_carrot_conflicts = sum(
            sample.path_heading_error * sample.carrot_angle < 0.0
            and abs(sample.path_heading_error) >= math.radians(5.0)
            and abs(sample.carrot_angle) >= math.radians(5.0)
            for sample in filtered_samples
        )
        filter_lag = [
            abs(unwrap_delta(sample.carrot_angle, sample.filtered_heading_error))
            for sample in filtered_samples
        ]
        print(
            "V11.5 dynamic heading: "
            f"future-tangent/carrot opposite={tangent_carrot_conflicts}/"
            f"{len(filtered_samples)}, filtered lag p90/max="
            f"{math.degrees(percentile(filter_lag, .90)):.1f}/"
            f"{math.degrees(max(filter_lag)):.1f}deg"
        )

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

    # Compare the final command delivered to the chassis with the wheel RPM
    # returned by the two motor controllers.  This is the decisive split for a
    # "path is visible but the robot does not move" report: if cmd_vel_safe is
    # already zero, the stop happened above the chassis; if it is non-zero but
    # both wheels stay still, the fault is in the actuator/serial/physical layer.
    wheel_radius = 0.05
    wheel_base = 0.269
    feedback = records.get("/wheel_feedback", [])
    measured_motion: list[tuple[int, float, float]] = []
    for timestamp, message in feedback:
        if len(message.data) < 4:
            continue
        left_rpm = float(message.data[2])
        right_rpm = float(message.data[3])
        left_speed = left_rpm * 2.0 * math.pi * wheel_radius / 60.0
        right_speed = right_rpm * 2.0 * math.pi * wheel_radius / 60.0
        measured_motion.append(
            (
                timestamp,
                (left_speed + right_speed) / 2.0,
                (right_speed - left_speed) / wheel_base,
            )
        )

    if measured_motion:
        measured_linear = [abs(item[1]) for item in measured_motion]
        measured_angular = [abs(item[2]) for item in measured_motion]
        feedback_periods_ms = [
            (two[0] - one[0]) / 1e6
            for one, two in zip(measured_motion, measured_motion[1:])
            if two[0] > one[0]
        ]
        print(
            "wheel feedback: "
            f"n={len(measured_motion)}, rate~="
            f"{1000.0 / statistics.median(feedback_periods_ms) if feedback_periods_ms else 0.0:.1f}Hz, "
            f"|v| p90/max={percentile(measured_linear, .90):.3f}/"
            f"{max(measured_linear):.3f}, |w| p90/max="
            f"{percentile(measured_angular, .90):.3f}/{max(measured_angular):.3f}"
        )

        safe_commands = records.get("/cmd_vel_safe", [])
        safe_index = 0
        compared = 0
        stalled = 0
        stall_intervals: list[tuple[int, int]] = []
        stall_start: int | None = None
        previous_timestamp: int | None = None
        for timestamp, measured_v, measured_w in measured_motion:
            while (
                safe_index + 1 < len(safe_commands)
                and safe_commands[safe_index + 1][0] <= timestamp
            ):
                safe_index += 1
            has_fresh_command = (
                safe_commands
                and safe_commands[safe_index][0] <= timestamp
                and timestamp - safe_commands[safe_index][0] <= 300_000_000
            )
            is_stalled = False
            if has_fresh_command:
                command = safe_commands[safe_index][1]
                command_v = float(command.linear.x)
                command_w = float(command.angular.z)
                commanded_motion = abs(command_v) >= 0.03 or abs(command_w) >= 0.10
                wheel_motion = abs(measured_v) >= 0.01 or abs(measured_w) >= 0.03
                compared += 1
                is_stalled = commanded_motion and not wheel_motion
                stalled += is_stalled

            contiguous = (
                previous_timestamp is not None
                and timestamp - previous_timestamp <= 300_000_000
            )
            if is_stalled:
                if stall_start is None or not contiguous:
                    stall_start = timestamp
            elif stall_start is not None:
                stall_intervals.append((stall_start, previous_timestamp or timestamp))
                stall_start = None
            previous_timestamp = timestamp
        if stall_start is not None and previous_timestamp is not None:
            stall_intervals.append((stall_start, previous_timestamp))

        long_stalls = [
            (one, two) for one, two in stall_intervals
            if two - one >= 1_000_000_000
        ]
        print(
            "safe-command/wheel execution: "
            f"near-zero feedback under motion command={stalled}/{compared}, "
            f">=1.0s stall intervals={len(long_stalls)}"
        )
        for one, two in long_stalls[:8]:
            print(
                f"  actuator stall at +{(one - start_ns) / 1e9:.2f}s "
                f"for {(two - one) / 1e9:.2f}s"
            )

    # Wheel encoders are not a sufficient physical-motion witness for an
    # in-place turn: a low-RPM motor/gear state can advance encoder odometry
    # while the chassis remains stuck on static friction. Pair the command with
    # the IMU and report these intervals independently of wheel feedback.
    imu_samples = records.get("/imu/data", [])
    safe_commands = records.get("/cmd_vel_safe", [])
    if imu_samples and safe_commands:
        command_index = 0
        stall_start: int | None = None
        previous_timestamp: int | None = None
        physical_stalls: list[tuple[int, int]] = []
        compared = 0
        still_samples = 0
        for timestamp, message in imu_samples:
            while (
                command_index + 1 < len(safe_commands)
                and safe_commands[command_index + 1][0] <= timestamp
            ):
                command_index += 1
            command_timestamp, command = safe_commands[command_index]
            command_fresh = (
                command_timestamp <= timestamp
                and timestamp - command_timestamp <= 300_000_000
            )
            pure_rotation = command_fresh and (
                abs(float(command.linear.x)) < 0.03
                and abs(float(command.angular.z)) >= 0.05
            )
            imu_w = float(message.angular_velocity.z)
            physically_still = (
                pure_rotation and math.isfinite(imu_w) and abs(imu_w) < 0.03
            )
            if pure_rotation:
                compared += 1
                still_samples += physically_still
            contiguous = (
                previous_timestamp is not None
                and timestamp - previous_timestamp <= 100_000_000
            )
            if physically_still:
                if stall_start is None or not contiguous:
                    stall_start = timestamp
            elif stall_start is not None:
                physical_stalls.append((stall_start, previous_timestamp or timestamp))
                stall_start = None
            previous_timestamp = timestamp
        if stall_start is not None and previous_timestamp is not None:
            physical_stalls.append((stall_start, previous_timestamp))
        long_physical_stalls = [
            (one, two) for one, two in physical_stalls
            if two - one >= 600_000_000
        ]
        print(
            "safe-command/IMU physical rotation: "
            f"still={still_samples}/{compared} pure-turn samples, "
            f">=0.6s stalls={len(long_physical_stalls)}"
        )
        for one, two in long_physical_stalls[:8]:
            print(
                f"  physical rotation stall at +{(one - start_ns) / 1e9:.2f}s "
                f"for {(two - one) / 1e9:.2f}s"
            )

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
        # For pre-V11.8 bags, replay the new geometric lookahead decision
        # without changing commands. This shows how selectively the corner
        # rule would have acted on the recorded planner path.
        pose_index = 0
        plan_index = 0
        simulated_lookahead = []
        simulated_bend = []
        for sample in tracking:
            while pose_index + 1 < len(map_poses) and map_poses[pose_index + 1][0] <= sample.timestamp:
                pose_index += 1
            while plan_index + 1 < len(plans) and plans[plan_index + 1][0] <= sample.timestamp:
                plan_index += 1
            plan_message = plans[plan_index][1]
            if not plan_message.poses:
                continue
            points = [
                (float(item.pose.position.x), float(item.pose.position.y))
                for item in plan_message.poses
            ]
            robot = map_poses[pose_index][1]
            near = tracking_point(points, robot[:2], 0.12)
            far = tracking_point(points, robot[:2], 0.35)
            near_bearing = math.atan2(near[1] - robot[1], near[0] - robot[0])
            far_bearing = math.atan2(far[1] - robot[1], far[0] - robot[0])
            bend = abs(unwrap_delta(far_bearing, near_bearing))
            ratio = max(0.0, min(1.0, (bend - 0.20) / (0.45 - 0.20)))
            simulated_bend.append(bend)
            simulated_lookahead.append(0.35 - ratio * (0.35 - 0.12))
        if simulated_lookahead:
            active = sum(value < 0.349 for value in simulated_lookahead)
            print(
                "simulated V11.8 corner lookahead on this bag: "
                f"active={active}/{len(simulated_lookahead)}, "
                f"lookahead min/p10/p50="
                f"{min(simulated_lookahead):.3f}/"
                f"{percentile(simulated_lookahead, .10):.3f}/"
                f"{percentile(simulated_lookahead, .50):.3f}m, "
                f"bend p90/max="
                f"{math.degrees(percentile(simulated_bend, .90)):.1f}/"
                f"{math.degrees(max(simulated_bend)):.1f}deg"
            )

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

    underdamped_episodes = []
    for number, episode in enumerate(tracking_episodes(tracking), 1):
        angular = [sample.angular for sample in episode]
        carrot_signed = [sample.carrot_angle for sample in episode]
        carrot = [abs(value) for value in carrot_signed]
        cross = [abs(sample.cross_track) for sample in episode]
        runs, changes = sign_runs(angular)
        carrot_runs, carrot_changes = sign_runs(
            carrot_signed, threshold=math.radians(3.0), minimum=3
        )
        stopped = sum(
            abs(sample.linear) < 1e-4 and abs(sample.angular) < 1e-4
            for sample in episode
        )
        near = [sample for sample in episode if sample.distance <= 0.20]
        near_runs, near_changes = sign_runs([sample.angular for sample in near])
        final_heading = [sample for sample in episode if sample.final_heading]
        # Three or more persistent error-sign reversals, or six or more command
        # reversals, is not dismissed merely because the final XY goal succeeds.
        # It is a practical real-robot indicator of an under-damped turn/track
        # transient and matches the V11.3 long-return observation.
        if carrot_changes >= 3 or changes >= 6:
            underdamped_episodes.append(number)
        print(
            f"episode {number}: {(episode[-1].timestamp - episode[0].timestamp) / 1e9:.1f}s, "
            f"distance {episode[0].distance:.3f}->{episode[-1].distance:.3f}m "
            f"(min {min(sample.distance for sample in episode):.3f}), "
            f"rotate={sum(sample.rotate for sample in episode) / len(episode):.1%}, "
            f"zero={stopped / len(episode):.1%}, angular changes={changes}, "
            f"stable heading-error sign changes={carrot_changes}, "
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

    if underdamped_episodes:
        print(
            "turning assessment: UNDERDAMPED_TRANSIENT in episode(s) "
            + ", ".join(map(str, underdamped_episodes))
            + "; goal success alone does not make these segments acceptable"
        )
    elif tracking:
        print("turning assessment: no repeated persistent sign reversal detected")

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

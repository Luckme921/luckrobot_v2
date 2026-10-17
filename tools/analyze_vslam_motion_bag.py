#!/usr/bin/env python3
"""Time-aligned comparison of wheel, EKF and cuVSLAM odometry.

This script intentionally uses the ROS 2 Humble rosbag2 Python API available on
the Jetson.  All decoded topics use standard ROS message types, so the Isaac
ROS interface package is not required on the host.
"""

import argparse
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOM_TOPICS = (
    "/wheel/odom",
    "/odometry/filtered",
    "/visual_slam/tracking/odometry",
    "/visual_slam/tracking/odometry_validated",
    "/odometry/vslam_candidate",
)

COMMAND_TOPICS = (
    "/cmd_vel_safe",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/cmd_vel_vslam_test",
    "/cmd_vel_test",
)

VSLAM_POSE_COV_TOPIC = "/visual_slam/tracking/vo_pose_covariance"


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def nearest_before(samples, stamp):
    before = [sample for sample in samples if sample[0] <= stamp]
    return before[-1] if before else samples[0]


def nearest_after(samples, stamp):
    after = [sample for sample in samples if sample[0] >= stamp]
    return after[0] if after else samples[-1]


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = set(ODOM_TOPICS) | set(COMMAND_TOPICS) | {
        VSLAM_POSE_COV_TOPIC,
        "/vslam/health",
    }
    message_classes = {
        topic: get_message(msg_type)
        for topic, msg_type in topic_types.items()
        if topic in wanted
    }
    samples = {topic: [] for topic in message_classes}
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in message_classes:
            continue
        msg = deserialize_message(raw, message_classes[topic])
        stamp = timestamp_ns * 1.0e-9
        if topic in ODOM_TOPICS:
            pose = msg.pose.pose
            twist = msg.twist.twist
            samples[topic].append(
                (
                    stamp,
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw_from_quaternion(pose.orientation),
                    float(twist.linear.x),
                    float(twist.angular.z),
                )
            )
        elif topic == VSLAM_POSE_COV_TOPIC:
            pose = msg.pose.pose
            covariance = msg.pose.covariance
            samples[topic].append(
                (
                    stamp,
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw_from_quaternion(pose.orientation),
                    float(covariance[0]),
                    float(covariance[7]),
                    float(covariance[35]),
                )
            )
        elif topic in COMMAND_TOPICS:
            samples[topic].append((stamp, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/vslam/health":
            samples[topic].append((stamp, bool(msg.data)))
    return samples


def summarize_odom(topic, samples, motion_start, motion_end):
    start = nearest_before(samples, motion_start)
    # Include the bounded command's braking tail, but not the long diagnostic
    # postamble that can accumulate unrelated idle drift.
    end = nearest_after(samples, motion_end + 1.0)
    final = samples[-1]

    dx = end[1] - start[1]
    dy = end[2] - start[2]
    c = math.cos(start[3])
    s = math.sin(start[3])
    forward = c * dx + s * dy
    lateral = -s * dx + c * dy
    yaw_delta = wrap_angle(end[3] - start[3])

    gaps_ms = [
        (right[0] - left[0]) * 1000.0
        for left, right in zip(samples, samples[1:])
        if right[0] >= left[0]
    ]
    motion_samples = [
        sample for sample in samples
        if motion_start - 0.25 <= sample[0] <= motion_end + 1.0
    ]
    motion_gaps_ms = [
        (right[0] - left[0]) * 1000.0
        for left, right in zip(motion_samples, motion_samples[1:])
        if right[0] >= left[0]
    ]
    duration = samples[-1][0] - samples[0][0]
    rate = (len(samples) - 1) / duration if duration > 0.0 else math.nan
    idle_dx = final[1] - end[1]
    idle_dy = final[2] - end[2]
    idle_drift = math.hypot(idle_dx, idle_dy)
    idle_yaw = wrap_angle(final[3] - end[3])

    return {
        "topic": topic,
        "count": len(samples),
        "rate": rate,
        "gap_p95_ms": percentile(gaps_ms, 95),
        "gap_max_ms": max(gaps_ms) if gaps_ms else math.nan,
        "motion_gap_max_ms": max(motion_gaps_ms) if motion_gaps_ms else math.nan,
        "peak_linear": max((abs(sample[4]) for sample in motion_samples), default=math.nan),
        "peak_angular": max((abs(sample[5]) for sample in motion_samples), default=math.nan),
        "forward": forward,
        "lateral": lateral,
        "distance": math.hypot(dx, dy),
        "yaw_delta": yaw_delta,
        "idle_drift": idle_drift,
        "idle_yaw": idle_yaw,
        "start_time_offset": start[0] - motion_start,
        "end_time_offset": end[0] - motion_end,
    }


def integrate_command(commands, start, end, index):
    active = [item for item in commands if start <= item[0] <= end]
    if len(active) < 2:
        return math.nan
    total = 0.0
    for left, right in zip(active, active[1:]):
        dt = right[0] - left[0]
        total += 0.5 * (left[index] + right[index]) * dt
    return total


def summarize_vslam_covariance(samples, motion_start, motion_end):
    active = [
        sample for sample in samples
        if motion_start - 0.25 <= sample[0] <= motion_end + 1.0
    ]
    if not active:
        return None
    values = np.asarray([sample[4:7] for sample in active], dtype=float)
    values = np.maximum(values, 0.0)
    return {
        "count": len(active),
        "std_median": np.sqrt(np.median(values[:, :2], axis=0)),
        "std_max": np.sqrt(np.max(values[:, :2], axis=0)),
        "yaw_std_median": math.degrees(math.sqrt(float(np.median(values[:, 2])))),
        "yaw_std_max": math.degrees(math.sqrt(float(np.max(values[:, 2])))),
        "all_zero": bool(np.all(values == 0.0)),
    }


def motion_windows(commands):
    windows = []
    start = None
    last_active = None
    for sample in commands:
        active = abs(sample[1]) > 0.01 or abs(sample[2]) > 0.02
        if active:
            if start is None:
                start = sample[0]
            last_active = sample[0]
        elif start is not None and last_active is not None:
            if sample[0] - last_active >= 0.30:
                if last_active - start >= 0.20:
                    windows.append((start, last_active + 0.50))
                start = None
                last_active = None
    if start is not None and last_active is not None and last_active - start >= 0.20:
        windows.append((start, last_active + 0.50))
    return windows


def accumulated_motion(samples, windows):
    distance = 0.0
    yaw = 0.0
    used = 0
    for left, right in zip(samples, samples[1:]):
        midpoint = 0.5 * (left[0] + right[0])
        if not any(start <= midpoint <= end for start, end in windows):
            continue
        if right[0] <= left[0] or right[0] - left[0] > 0.25:
            continue
        distance += math.hypot(right[1] - left[1], right[2] - left[2])
        yaw += abs(wrap_angle(right[3] - left[3]))
        used += 1
    return distance, yaw, used


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    args = parser.parse_args()

    samples = read_bag(args.bag_dir)
    command_topic = next((topic for topic in COMMAND_TOPICS if samples.get(topic)), None)
    if command_topic is None:
        wheel_fallback = samples.get("/wheel/odom", [])
        if not wheel_fallback:
            raise SystemExit("No recorded velocity command or wheel odometry topic")
        command_topic = "/wheel/odom twist (fallback)"
        commands = [(item[0], item[4], item[5]) for item in wheel_fallback]
    else:
        commands = samples[command_topic]
    active = [item for item in commands if abs(item[1]) > 0.005 or abs(item[2]) > 0.005]
    if not active:
        raise SystemExit("No non-zero motion command in bag")

    motion_start = active[0][0]
    motion_end = active[-1][0]
    windows = motion_windows(commands)
    linear_integral = integrate_command(commands, motion_start, motion_end, 1)
    angular_integral = integrate_command(commands, motion_start, motion_end, 2)

    print(f"bag={args.bag_dir}")
    print(f"command_topic={command_topic}")
    print(f"motion_window_s={motion_end - motion_start:.3f}")
    print(f"motion_segments={len(windows)}")
    print(f"command_integral_linear_m={linear_integral:.4f}")
    print(f"command_integral_yaw_rad={angular_integral:.4f}")
    print()

    summaries = {}
    for topic in ODOM_TOPICS:
        topic_samples = samples.get(topic, [])
        if not topic_samples:
            print(f"{topic}: NO_DATA")
            continue
        summary = summarize_odom(topic, topic_samples, motion_start, motion_end)
        summaries[topic] = summary
        print(
            f"{topic}: n={summary['count']} rate={summary['rate']:.3f}Hz "
            f"gap_p95={summary['gap_p95_ms']:.2f}ms gap_max={summary['gap_max_ms']:.2f}ms "
            f"motion_gap_max={summary['motion_gap_max_ms']:.2f}ms"
        )
        print(
            f"  forward={summary['forward']:.4f}m lateral={summary['lateral']:.4f}m "
            f"distance={summary['distance']:.4f}m yaw={summary['yaw_delta']:.4f}rad "
            f"({math.degrees(summary['yaw_delta']):.2f}deg)"
        )
        print(
            f"  peak_linear={summary['peak_linear']:.4f}m/s "
            f"peak_angular={summary['peak_angular']:.4f}rad/s"
        )
        print(
            f"  post_window_idle_drift={summary['idle_drift']:.4f}m "
            f"idle_yaw={math.degrees(summary['idle_yaw']):.3f}deg"
        )

    visual_covariance = summarize_vslam_covariance(
        samples.get(VSLAM_POSE_COV_TOPIC, []), motion_start, motion_end
    )
    if visual_covariance:
        print()
        print("===== cuVSLAM reported pose covariance =====")
        print(
            "motion_samples="
            f"{visual_covariance['count']} all_zero={visual_covariance['all_zero']}"
        )
        print(
            "position_std_median="
            f"x={visual_covariance['std_median'][0]:.6f}m "
            f"y={visual_covariance['std_median'][1]:.6f}m "
            "position_std_max="
            f"x={visual_covariance['std_max'][0]:.6f}m "
            f"y={visual_covariance['std_max'][1]:.6f}m"
        )
        print(
            "yaw_std_median="
            f"{visual_covariance['yaw_std_median']:.6f}deg "
            f"yaw_std_max={visual_covariance['yaw_std_max']:.6f}deg"
        )

    wheel = summaries.get("/wheel/odom")
    visual = summaries.get("/visual_slam/tracking/odometry")
    guarded = summaries.get("/visual_slam/tracking/odometry_validated")
    filtered = summaries.get("/odometry/filtered")
    candidate = summaries.get("/odometry/vslam_candidate")
    if wheel and visual:
        print()
        print("===== VSLAM relative to wheel =====")
        if abs(wheel["forward"]) > 0.05:
            scale = visual["forward"] / wheel["forward"]
            print(f"forward_scale_ratio={scale:.5f} error={(scale - 1.0) * 100.0:+.2f}%")
        print(
            "yaw_difference="
            f"{math.degrees(wrap_angle(visual['yaw_delta'] - wheel['yaw_delta'])):+.3f}deg"
        )
        print(f"visual_lateral_minus_wheel={visual['lateral'] - wheel['lateral']:+.4f}m")
    if filtered and visual:
        print("===== VSLAM relative to wheel/IMU EKF =====")
        print(
            "yaw_difference="
            f"{math.degrees(wrap_angle(visual['yaw_delta'] - filtered['yaw_delta'])):+.3f}deg"
        )

    print()
    print("===== Dynamic motion-consistency gate =====")
    failures = []
    wheel_samples = samples.get("/wheel/odom", [])
    visual_samples = samples.get("/visual_slam/tracking/odometry", [])
    if not wheel_samples or not visual_samples:
        failures.append("missing wheel or raw VSLAM odometry")
    else:
        wheel_path, wheel_yaw, _ = accumulated_motion(wheel_samples, windows)
        visual_path, visual_yaw, _ = accumulated_motion(visual_samples, windows)
        print(
            f"wheel_path={wheel_path:.4f}m visual_path={visual_path:.4f}m "
            f"wheel_abs_yaw={math.degrees(wheel_yaw):.2f}deg "
            f"visual_abs_yaw={math.degrees(visual_yaw):.2f}deg"
        )
        if wheel_path >= 0.20:
            path_ratio = visual_path / wheel_path
            print(f"path_ratio={path_ratio:.4f} (required 0.65..1.35)")
            if not 0.65 <= path_ratio <= 1.35:
                failures.append("visual/wheel path ratio outside 0.65..1.35")
        if wheel_yaw >= math.radians(20.0):
            yaw_ratio = visual_yaw / wheel_yaw
            print(f"yaw_ratio={yaw_ratio:.4f} (required 0.65..1.35)")
            if not 0.65 <= yaw_ratio <= 1.35:
                failures.append("visual/wheel yaw ratio outside 0.65..1.35")

    health = samples.get("/vslam/health", [])
    if health:
        healthy_fraction = sum(int(item[1]) for item in health) / len(health)
        print(f"health_true_fraction={healthy_fraction:.3f}")
        if healthy_fraction < 0.90:
            failures.append("VSLAM health true fraction below 0.90")
    else:
        failures.append("missing /vslam/health")

    if guarded and wheel:
        print("===== Guarded VSLAM relative to wheel =====")
        if abs(wheel["forward"]) > 0.05:
            scale = guarded["forward"] / wheel["forward"]
            print(f"forward_scale_ratio={scale:.5f} error={(scale - 1.0) * 100.0:+.2f}%")
        print(
            "yaw_difference="
            f"{math.degrees(wrap_angle(guarded['yaw_delta'] - wheel['yaw_delta'])):+.3f}deg"
        )
    if candidate and wheel:
        print("===== VSLAM candidate EKF relative to wheel =====")
        if abs(wheel["forward"]) > 0.05:
            scale = candidate["forward"] / wheel["forward"]
            print(f"forward_scale_ratio={scale:.5f} error={(scale - 1.0) * 100.0:+.2f}%")
        print(
            "yaw_difference="
            f"{math.degrees(wrap_angle(candidate['yaw_delta'] - wheel['yaw_delta'])):+.3f}deg"
        )
    print("RESULT=" + ("PASS" if not failures else "FAIL"))
    for failure in failures:
        print(f"  - {failure}")


if __name__ == "__main__":
    main()

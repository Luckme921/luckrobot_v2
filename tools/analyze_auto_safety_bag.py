#!/usr/bin/env python3
"""Inspect safety stops, costmap exposure, and compute load in an exploration bag."""

from __future__ import annotations

import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


BEGIN_RE = re.compile(
    r"Begin navigating from current location \(([-+0-9.]+), ([-+0-9.]+)\) "
    r"to \(([-+0-9.]+), ([-+0-9.]+)\)"
)
LOCK_RE = re.compile(
    r"V[\d.]+ goal locked: .* known-white goal \(([-+0-9.]+), "
    r"([-+0-9.]+)\).*global cost ([0-9]+)"
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2),
    )


def compose(a, b):
    """Compose planar parent->middle and middle->child transforms."""
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ath + bth)


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list[int]

    @classmethod
    def from_message(cls, msg):
        return cls(
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            [int(value) for value in msg.data],
        )

    def update(self, msg) -> None:
        x0, y0 = int(msg.x), int(msg.y)
        width, height = int(msg.width), int(msg.height)
        values = [int(value) for value in msg.data]
        for row in range(height):
            gy = y0 + row
            if not 0 <= gy < self.height:
                continue
            for column in range(width):
                gx = x0 + column
                if 0 <= gx < self.width:
                    self.data[gy * self.width + gx] = values[row * width + column]

    def cost(self, x: float, y: float) -> int:
        gx = math.floor((x - self.origin_x) / self.resolution)
        gy = math.floor((y - self.origin_y) / self.resolution)
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return -2
        return self.data[gy * self.width + gx]


def finite_scan_min(msg, half_angle=None):
    values = []
    for index, value in enumerate(msg.ranges):
        value = float(value)
        if not math.isfinite(value) or value < float(msg.range_min) or value > float(msg.range_max):
            continue
        if half_angle is not None:
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if abs(wrapped) > half_angle:
                continue
        values.append(value)
    return min(values) if values else math.inf


def median(values):
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.inf


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {Path(sys.argv[0]).name} BAG_DIRECTORY", file=sys.stderr)
        return 2

    bag_dir = Path(sys.argv[1])
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    # map_msgs is not part of rosbags' Humble core store.
    typestore.register(get_types_from_msg(
        "std_msgs/Header header\n"
        "uint32 x\nuint32 y\nuint32 width\nuint32 height\nint8[] data\n",
        "map_msgs/msg/OccupancyGridUpdate",
    ))
    wanted = {
        "/rosout",
        "/tf",
        "/global_costmap/costmap",
        "/global_costmap/costmap_updates",
        "/local_costmap/costmap",
        "/local_costmap/costmap_updates",
        "/plan",
        "/cmd_vel_nav",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/scan_filtered",
        "/scan_camera",
        "/wheel/odom",
        "/odometry/filtered",
    }

    grids = {"global": None, "local": None}
    transforms = {}
    robot_samples = []
    plans = []
    logs = []
    commands = {topic: [] for topic in ("/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe")}
    scans = {"/scan_filtered": [], "/scan_camera": []}
    odometry_timestamps = {"/wheel/odom": [], "/odometry/filtered": []}
    last_robot_sample = 0

    with Reader(bag_dir) as reader:
        start_ns, end_ns = reader.start_time, reader.end_time
        connections = [connection for connection in reader.connections if connection.topic in wanted]
        for connection, timestamp, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, connection.msgtype)
            topic = connection.topic
            if topic == "/rosout":
                logs.append((timestamp, msg.name, msg.msg))
            elif topic == "/tf":
                for item in msg.transforms:
                    parent = item.header.frame_id.lstrip("/")
                    child = item.child_frame_id.lstrip("/")
                    transforms[(parent, child)] = (
                        float(item.transform.translation.x),
                        float(item.transform.translation.y),
                        yaw_from_quaternion(item.transform.rotation),
                    )
            elif topic == "/global_costmap/costmap":
                grids["global"] = Grid.from_message(msg)
            elif topic == "/global_costmap/costmap_updates" and grids["global"] is not None:
                grids["global"].update(msg)
            elif topic == "/local_costmap/costmap":
                grids["local"] = Grid.from_message(msg)
            elif topic == "/local_costmap/costmap_updates" and grids["local"] is not None:
                grids["local"].update(msg)
            elif topic == "/plan":
                points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
                length = sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))
                costs = [grids["global"].cost(x, y) for x, y in points] if grids["global"] else []
                plans.append((timestamp, len(points), length, max(costs) if costs else -2,
                              points[-1] if points else (math.nan, math.nan)))
            elif topic in commands:
                commands[topic].append((timestamp, float(msg.linear.x), float(msg.angular.z)))
            elif topic in scans:
                scans[topic].append((timestamp, finite_scan_min(msg), finite_scan_min(msg, math.radians(30))))
            elif topic in odometry_timestamps:
                odometry_timestamps[topic].append(timestamp)

            if timestamp - last_robot_sample >= 100_000_000:
                map_odom = transforms.get(("map", "odom"))
                odom_base = transforms.get(("odom", "base_footprint"))
                if map_odom is not None and odom_base is not None:
                    map_pose = compose(map_odom, odom_base)
                    global_cost = grids["global"].cost(map_pose[0], map_pose[1]) if grids["global"] else -2
                    local_cost = grids["local"].cost(odom_base[0], odom_base[1]) if grids["local"] else -2
                    robot_samples.append((timestamp, *map_pose, global_cost, local_cost))
                    last_robot_sample = timestamp

    patterns = {
        "ekf_rate_miss": "Failed to meet update rate",
        "bt_rate_miss": "Behavior Tree tick rate",
        "rpp_collision": "detected collision ahead",
        "controller_patience": "Controller patience exceeded",
        "failed_progress": "Failed to make progress",
        "collision_approach": "Robot to approach",
        "collision_stop": "Robot to stop",
        "nav_succeeded": "Goal succeeded",
        "nav_aborted": "Aborting handle",
    }
    print(f"bag={bag_dir} duration={(end_ns-start_ns)/1e9:.1f}s")
    print("log events: " + " ".join(
        f"{name}={sum(needle in message for _, _, message in logs)}"
        for name, needle in patterns.items()
    ))

    if robot_samples:
        global_costs = [item[4] for item in robot_samples if item[4] >= 0]
        local_costs = [item[5] for item in robot_samples if item[5] >= 0]
        print(
            "robot sampled cost: "
            f"global max={max(global_costs, default=-1)} >40={sum(c > 40 for c in global_costs)} "
            f"local max={max(local_costs, default=-1)} >=99={sum(c >= 99 for c in local_costs)}"
        )

    for topic, values in commands.items():
        moving = [(abs(linear), abs(angular)) for _, linear, angular in values if abs(linear) > 1e-4 or abs(angular) > 1e-4]
        print(
            f"{topic}: samples={len(values)} moving={len(moving)} "
            f"linear_max={max((v[0] for v in moving), default=0):.3f} "
            f"angular_max={max((v[1] for v in moving), default=0):.3f}"
        )

    for topic, values in scans.items():
        print(
            f"{topic}: samples={len(values)} median_nearest={median(v[1] for v in values):.3f}m "
            f"median_front30={median(v[2] for v in values):.3f}m"
        )

    for topic, values in odometry_timestamps.items():
        gaps = [(b - a) / 1e9 for a, b in zip(values, values[1:])]
        sorted_gaps = sorted(gaps)
        p99 = sorted_gaps[min(len(sorted_gaps) - 1, int(0.99 * len(sorted_gaps)))] if sorted_gaps else math.inf
        print(
            f"{topic}: samples={len(values)} rate={len(values)/((end_ns-start_ns)/1e9):.1f}Hz "
            f"median_gap={median(gaps)*1000:.1f}ms p99_gap={p99*1000:.1f}ms "
            f"max_gap={max(gaps, default=math.inf)*1000:.1f}ms"
        )

    print("\nNavigation intervals:")
    begins = []
    for timestamp, _, message in logs:
        match = BEGIN_RE.search(message)
        if match:
            begins.append((timestamp, *map(float, match.groups())))
    for index, begin in enumerate(begins):
        timestamp, sx, sy, gx, gy = begin
        stop = begins[index + 1][0] if index + 1 < len(begins) else end_ns
        interval_logs = [message for t, _, message in logs if timestamp <= t < stop]
        interval_robot = [item for item in robot_samples if timestamp <= item[0] < stop]
        interval_plans = [item for item in plans if timestamp <= item[0] < stop]
        interval_lidar = [item for item in scans["/scan_filtered"] if timestamp <= item[0] < stop]
        interval_camera = [item for item in scans["/scan_camera"] if timestamp <= item[0] < stop]
        gcost = max((item[4] for item in interval_robot), default=-2)
        lcost = max((item[5] for item in interval_robot), default=-2)
        endpoint_error = math.nan
        if interval_robot:
            endpoint_error = math.hypot(interval_robot[-1][1] - gx, interval_robot[-1][2] - gy)
        plan_max = max((item[3] for item in interval_plans), default=-2)
        result = "SUCCEEDED" if any("Goal succeeded" in item for item in interval_logs) else (
            "ABORTED" if any("Aborting handle" in item or "Goal failed" in item for item in interval_logs) else "OPEN"
        )
        print(
            f" {index+1:02d} t={(timestamp-start_ns)/1e9:6.1f}s "
            f"({sx:+.2f},{sy:+.2f})->({gx:+.2f},{gy:+.2f}) {result:9s} "
            f"end_err={endpoint_error:.2f}m plan_max={plan_max} robot_gmax={gcost} robot_lmax={lcost} "
            f"lidar_min={min((v[1] for v in interval_lidar), default=math.inf):.2f}m "
            f"camera_min={min((v[1] for v in interval_camera), default=math.inf):.2f}m"
        )

    print("\nExploration goal prechecks:")
    for timestamp, _, message in logs:
        match = LOCK_RE.search(message)
        if not match:
            continue
        gx, gy, goal_cost = match.groups()
        prior = [item for item in plans if 0 <= timestamp - item[0] <= 2_000_000_000]
        checked = prior[-1] if prior else None
        if checked is None:
            print(
                f" t={(timestamp-start_ns)/1e9:6.1f}s goal=({gx},{gy}) "
                f"cost={goal_cost} precheck=<missing>"
            )
        else:
            print(
                f" t={(timestamp-start_ns)/1e9:6.1f}s goal=({gx},{gy}) "
                f"cost={goal_cost} precheck_max={checked[3]} "
                f"path={checked[2]:.2f}m age={(timestamp-checked[0])/1e9:.3f}s"
            )

    print("\nSafety/control timeline:")
    # Keep the detailed timeline focused on motion and safety decisions.  EKF
    # and BT deadline misses are counted above; printing every one obscures the
    # causal sequence around a stop.
    needles = (
        patterns["rpp_collision"],
        patterns["controller_patience"],
        patterns["failed_progress"],
        patterns["collision_approach"],
        patterns["collision_stop"],
        patterns["nav_succeeded"],
        patterns["nav_aborted"],
        "Robot start cell is unsafe",
        "Navigation on DFS branch",
        "No translation or rotation progress",
    )
    for timestamp, node, message in logs:
        if any(needle in message for needle in needles):
            print(f" t={(timestamp-start_ns)/1e9:7.2f}s [{node}] {message}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

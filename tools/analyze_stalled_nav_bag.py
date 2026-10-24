#!/usr/bin/env python3
"""Summarize stopped Nav2 bag around the latest navigation attempts."""
import sqlite3
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main(path):
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {name: (topic_id, get_message(kind)) for topic_id, name, kind in db.execute(
        "SELECT id,name,type FROM topics"
    )}
    status_id, status_cls = topics["/navigate_to_pose/_action/status"]
    status_rows = db.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (status_id,),
    ).fetchall()
    for stamp, blob in status_rows:
        msg = deserialize_message(blob, status_cls)
        print("status", round(stamp / 1e9, 3), [s.status for s in msg.status_list])
    if not status_rows:
        return
    t0 = status_rows[0][0]
    t1 = status_rows[-1][0]
    for name in (
        "/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe", "/odometry/filtered",
        "/local_costmap/costmap", "/plan", "/scan_camera", "/scan_filtered",
    ):
        if name not in topics:
            continue
        topic_id, cls = topics[name]
        rows = db.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp",
            (topic_id, t0 - 10_000_000_000, t1 + 2_000_000_000),
        ).fetchall()
        print(name, "count", len(rows))
        if not rows:
            continue
        if name.startswith("/cmd_vel"):
            vals = []
            for stamp, blob in rows:
                msg = deserialize_message(blob, cls)
                vals.append((stamp, msg.linear.x, msg.angular.z))
            nonzero = [x for x in vals if abs(x[1]) > 0.01 or abs(x[2]) > 0.01]
            print("  nonzero", len(nonzero), "max_abs_v", round(max(abs(x[1]) for x in vals), 3),
                  "max_abs_w", round(max(abs(x[2]) for x in vals), 3))
            print("  last10", [(round((s - t0) / 1e9, 1), round(v, 2), round(w, 2)) for s, v, w in vals[-10:]])
        elif name == "/odometry/filtered":
            pts = []
            for stamp, blob in rows:
                p = deserialize_message(blob, cls).pose.pose.position
                pts.append((stamp, p.x, p.y))
            print("  first", pts[0], "last", pts[-1])
            print("  range", round(max(x for _, x, _ in pts) - min(x for _, x, _ in pts), 3),
                  round(max(y for _, _, y in pts) - min(y for _, _, y in pts), 3))
        elif name == "/local_costmap/costmap":
            for stamp, blob in rows[-3:]:
                msg = deserialize_message(blob, cls)
                counts = {v: msg.data.count(v) for v in (0, 100, -1)}
                print("  map", round(stamp / 1e9, 2), msg.info.width, msg.info.height,
                      msg.info.resolution, counts)
        elif name == "/plan":
            print("  last path lengths", [len(deserialize_message(blob, cls).poses) for _, blob in rows[-5:]])
        elif name.startswith("/scan"):
            msg = deserialize_message(rows[-1][1], cls)
            finite = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
            print("  last finite", len(finite), "nearest", round(min(finite), 3) if finite else None)


if __name__ == "__main__":
    main(sys.argv[1])

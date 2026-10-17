#!/usr/bin/env python3
"""Fail fast when navigation localization or camera-planning TF is stale."""

import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class HealthCheck(Node):
    def __init__(self):
        super().__init__('nav_tf_health_check')
        self.latest = {}
        self.counts = {}
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda msg: self.save('ekf', msg.header.stamp),
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_camera',
                                 lambda msg: self.save('camera', msg.header.stamp),
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.on_tf,
                                 qos_profile_sensor_data)

    def save(self, key, stamp):
        self.latest[key] = stamp_seconds(stamp)
        self.counts[key] = self.counts.get(key, 0) + 1

    def on_tf(self, message):
        for transform in message.transforms:
            edge = (transform.header.frame_id.lstrip('/'),
                    transform.child_frame_id.lstrip('/'))
            if edge == ('odom', 'base_footprint'):
                self.save('odom_tf', transform.header.stamp)
            elif edge == ('map', 'odom'):
                self.save('map_tf', transform.header.stamp)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--max-age', type=float, default=1.0)
    parser.add_argument('--max-tf-age', type=float, default=4.0,
                        help='TF broadcaster stamps lagged ~2-3 s in healthy bags')
    args = parser.parse_args()
    rclpy.init()
    node = HealthCheck()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        failed = False
        for key in ('ekf', 'odom_tf', 'map_tf', 'camera'):
            count = node.counts.get(key, 0)
            age = now - node.latest[key] if key in node.latest else float('inf')
            age_limit = args.max_tf_age if key.endswith('_tf') else args.max_age
            # A small negative age is acceptable for TF future dating.
            ok = count >= 2 and -0.5 <= age <= age_limit
            print('%s: %s count=%d stamp_age=%.2fs limit=%.1fs' %
                  (key, 'OK' if ok else 'STALE/MISSING', count, age, age_limit),
                  flush=True)
            failed |= not ok
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Compare the production and height-aware camera LaserScan topics."""

import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


class ScanStats(Node):
    def __init__(self, topics):
        super().__init__('check_depth_height_scan')
        self.samples = {topic: [] for topic in topics}
        self._subscriptions = [
            self.create_subscription(
                LaserScan,
                topic,
                lambda message, topic=topic: self.on_scan(topic, message),
                qos_profile_sensor_data,
            )
            for topic in topics
        ]

    def on_scan(self, topic, message):
        finite = []
        front = []
        finite_support = []
        front_support = []
        has_support = len(message.intensities) == len(message.ranges)
        nearest_value = math.inf
        nearest_support = math.nan
        front_nearest_value = math.inf
        front_nearest_support = math.nan
        for index, raw in enumerate(message.ranges):
            value = float(raw)
            if not math.isfinite(value) or value < message.range_min or value > message.range_max:
                continue
            finite.append(value)
            if has_support:
                support = float(message.intensities[index])
                finite_support.append(support)
                if value < nearest_value:
                    nearest_value = value
                    nearest_support = support
            angle = message.angle_min + index * message.angle_increment
            if abs(angle) <= math.radians(30.0):
                front.append(value)
                if has_support:
                    support = float(message.intensities[index])
                    front_support.append(support)
                    if value < front_nearest_value:
                        front_nearest_value = value
                        front_nearest_support = support
        self.samples[topic].append(
            (
                time.monotonic(),
                message.header.frame_id,
                len(message.ranges),
                len(finite),
                min(finite) if finite else math.inf,
                min(front) if front else math.inf,
                statistics.median(finite_support) if finite_support else math.nan,
                statistics.median(front_support) if front_support else math.nan,
                nearest_support,
                front_nearest_support,
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument(
        '--topics', nargs='+', default=['/scan_camera', '/scan_camera_height']
    )
    args = parser.parse_args()

    rclpy.init()
    node = ScanStats(args.topics)
    deadline = time.monotonic() + max(1.0, args.duration)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('===== depth obstacle coverage comparison =====')
    for topic in args.topics:
        samples = node.samples[topic]
        if not samples:
            print(f'{topic}: NO_DATA')
            continue
        elapsed = max(1.0e-6, samples[-1][0] - samples[0][0])
        finite_fraction = [item[3] / max(1, item[2]) for item in samples]
        nearest = [item[4] for item in samples if math.isfinite(item[4])]
        front = [item[5] for item in samples if math.isfinite(item[5])]
        surface_support = [item[6] for item in samples if math.isfinite(item[6])]
        front_support = [item[7] for item in samples if math.isfinite(item[7])]
        nearest_support = [item[8] for item in samples if math.isfinite(item[8])]
        front_nearest_support = [
            item[9] for item in samples if math.isfinite(item[9])
        ]
        print(
            f'{topic}: n={len(samples)} rate={(len(samples)-1)/elapsed:.2f}Hz '
            f'frame={samples[-1][1]} beams={samples[-1][2]}'
        )
        print(
            '  finite_beams_median=%.1f%% nearest_median=%s nearest_p05=%s '
            'front30_median=%s surface_support_median=%s '
            'front30_support_median=%s nearest_support_median=%s '
            'front30_nearest_support_median=%s'
            % (
                100.0 * statistics.median(finite_fraction),
                'none' if not nearest else f'{statistics.median(nearest):.3f}m',
                'none' if not nearest else f'{percentile(nearest, 0.05):.3f}m',
                'none' if not front else f'{statistics.median(front):.3f}m',
                'unavailable'
                if not surface_support
                else f'{statistics.median(surface_support):.1f}',
                'unavailable'
                if not front_support
                else f'{statistics.median(front_support):.1f}',
                'unavailable'
                if not nearest_support
                else f'{statistics.median(nearest_support):.1f}',
                'unavailable'
                if not front_nearest_support
                else f'{statistics.median(front_nearest_support):.1f}',
            )
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Check Gemini stereo rates, timing, metadata and calibration without a bag."""

import argparse
import bisect
import math
import statistics
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def percentile(values, probability):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def stream_rate(stamps):
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return math.nan
    return (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])


def monotonic_violations(stamps):
    return sum(current <= previous for previous, current in zip(stamps, stamps[1:]))


def interval_jitter_ms(stamps):
    """Return absolute frame-period deviations from the median period."""
    intervals = [
        (current - previous) / 1e6
        for previous, current in zip(stamps, stamps[1:])
        if current > previous
    ]
    if not intervals:
        return []
    median_interval = statistics.median(intervals)
    return [abs(interval - median_interval) for interval in intervals]


def nearest_deltas_ms(left_stamps, right_stamps):
    right = sorted(right_stamps)
    deltas = []
    for value in sorted(left_stamps):
        index = bisect.bisect_left(right, value)
        candidates = []
        if index < len(right):
            candidates.append(abs(right[index] - value))
        if index:
            candidates.append(abs(right[index - 1] - value))
        if candidates:
            deltas.append(min(candidates) / 1e6)
    return deltas


def describe_info(label, info):
    if info is None:
        print(f'{label}_camera_info=MISSING')
        return None
    fx = info.k[0]
    baseline = abs(info.p[3] / info.p[0]) if abs(info.p[0]) > 1e-9 else math.nan
    print(
        f'{label}_camera_info={info.width}x{info.height} '
        f'frame={info.header.frame_id} fx={fx:.3f} '
        f'baseline_from_P={baseline:.6f}m distortion={info.distortion_model}'
    )
    return baseline


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--left', default='/vslam/stereo/left/image_raw')
    parser.add_argument('--right', default='/vslam/stereo/right/image_raw')
    parser.add_argument(
        '--expected-rate', type=float, default=30.0,
        help='Expected rate of each filtered stereo stream (30 normally, 15 in laser interleave mode)',
    )
    args = parser.parse_args()
    if args.duration < 3.0:
        parser.error('--duration must be at least 3 seconds')

    rclpy.init()
    node = rclpy.create_node('luckrobot_vslam_stereo_sync_check')
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=30,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    left_stamps = []
    right_stamps = []
    left_meta = None
    right_meta = None
    left_info = None
    right_info = None

    def on_left(message):
        nonlocal left_meta
        left_stamps.append(stamp_ns(message))
        left_meta = (message.width, message.height, message.encoding, message.header.frame_id)

    def on_right(message):
        nonlocal right_meta
        right_stamps.append(stamp_ns(message))
        right_meta = (message.width, message.height, message.encoding, message.header.frame_id)

    def on_left_info(message):
        nonlocal left_info
        left_info = message

    def on_right_info(message):
        nonlocal right_info
        right_info = message

    subscriptions = [
        node.create_subscription(Image, args.left, on_left, qos),
        node.create_subscription(Image, args.right, on_right, qos),
        node.create_subscription(CameraInfo, args.left.replace('image_raw', 'camera_info'), on_left_info, qos),
        node.create_subscription(CameraInfo, args.right.replace('image_raw', 'camera_info'), on_right_info, qos),
    ]
    # Keep explicit references until shutdown; some rclpy releases otherwise
    # allow subscription objects to be garbage-collected.
    assert subscriptions

    print(f'Collecting stereo headers for {args.duration:.1f}s ...', flush=True)
    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    deltas = nearest_deltas_ms(left_stamps, right_stamps)
    left_rate = stream_rate(left_stamps)
    right_rate = stream_rate(right_stamps)
    left_bad = monotonic_violations(left_stamps)
    right_bad = monotonic_violations(right_stamps)
    left_jitter = interval_jitter_ms(left_stamps)
    right_jitter = interval_jitter_ms(right_stamps)

    print('===== VSLAM stereo preflight =====')
    print(f'left_samples={len(left_stamps)} rate={left_rate:.3f}Hz nonmonotonic={left_bad}')
    print(f'right_samples={len(right_stamps)} rate={right_rate:.3f}Hz nonmonotonic={right_bad}')
    if left_jitter:
        print(
            'left_period_jitter_ms: p95={:.3f} max={:.3f}'.format(
                percentile(left_jitter, 0.95), max(left_jitter)
            )
        )
    if right_jitter:
        print(
            'right_period_jitter_ms: p95={:.3f} max={:.3f}'.format(
                percentile(right_jitter, 0.95), max(right_jitter)
            )
        )
    if left_meta:
        print('left_image={}x{} encoding={} frame={}'.format(*left_meta))
    if right_meta:
        print('right_image={}x{} encoding={} frame={}'.format(*right_meta))
    if deltas:
        print(
            'stereo_delta_ms: median={:.3f} p95={:.3f} max={:.3f} '
            'within_0p1ms={:.1f}%'.format(
                statistics.median(deltas),
                percentile(deltas, 0.95),
                max(deltas),
                100.0 * sum(delta <= 0.1 for delta in deltas) / len(deltas),
            )
        )
    else:
        print('stereo_delta_ms=MISSING')

    describe_info('left', left_info)
    right_baseline = describe_info('right', right_info)

    failures = []
    if len(left_stamps) < 10 or len(right_stamps) < 10:
        failures.append('missing stereo images')
    minimum_rate = args.expected_rate * 0.90
    if not math.isfinite(left_rate) or left_rate < minimum_rate:
        failures.append(
            f'left rate below {minimum_rate:.1f} Hz ({args.expected_rate:.1f} Hz target)'
        )
    if not math.isfinite(right_rate) or right_rate < minimum_rate:
        failures.append(
            f'right rate below {minimum_rate:.1f} Hz ({args.expected_rate:.1f} Hz target)'
        )
    if left_bad or right_bad:
        failures.append('non-monotonic image timestamps')
    if not left_jitter or percentile(left_jitter, 0.95) > 2.0:
        failures.append('left frame-period jitter p95 above 2 ms')
    if not right_jitter or percentile(right_jitter, 0.95) > 2.0:
        failures.append('right frame-period jitter p95 above 2 ms')
    if not deltas or percentile(deltas, 0.95) > 0.1:
        failures.append('stereo capture offset p95 above 0.1 ms')
    if left_meta and right_meta and left_meta[:3] != right_meta[:3]:
        failures.append('left/right image format mismatch')
    if left_info is None or right_info is None:
        failures.append('missing camera_info')
    if right_baseline is None or not math.isfinite(right_baseline) or right_baseline < 0.01:
        failures.append('invalid/non-stereo right projection baseline')

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print('RESULT=FAIL: ' + '; '.join(failures))
        return 4
    print('RESULT=PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

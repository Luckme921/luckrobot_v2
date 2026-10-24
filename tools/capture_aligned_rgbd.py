#!/usr/bin/env python3
"""Capture one time-matched RGB/depth pair for isolated camera-alignment QA.

This is a passive probe: it subscribes to images and never publishes commands.
The Orbbec camera must already be running with depth_registration:=true and
align_target_stream:=COLOR.
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def stamp_seconds(message):
    stamp = message.header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("aligned_rgbd_capture")
    colors, depths = [], []
    color_sub = node.create_subscription(
        Image, "/camera/color/image_raw", lambda msg: colors.append(msg),
        qos_profile_sensor_data,
    )
    depth_sub = node.create_subscription(
        Image, "/camera/depth/image_raw", lambda msg: depths.append(msg),
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline and (len(colors) < 12 or len(depths) < 12):
        rclpy.spin_once(node, timeout_sec=0.1)
    if not colors or not depths:
        raise RuntimeError(f"Missing image stream: color={len(colors)} depth={len(depths)}")

    pairs = [(abs(stamp_seconds(c) - stamp_seconds(d)), c, d)
             for c in colors[-20:] for d in depths[-20:]]
    delta, color_msg, depth_msg = min(pairs, key=lambda item: item[0])
    if delta > 0.033:
        raise RuntimeError(f"Nearest RGB/depth pair is {delta * 1000:.1f} ms apart")
    if color_msg.header.frame_id != depth_msg.header.frame_id:
        raise RuntimeError("Depth is not registered to the color optical frame")

    bridge = CvBridge()
    color = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    if depth.dtype != np.uint16 or color.shape[:2] != depth.shape[:2]:
        raise RuntimeError(f"Unexpected images: color={color.shape}, depth={depth.shape}/{depth.dtype}")

    valid = (depth > 0) & (depth < 3000)
    depth_view = np.clip(depth.astype(np.float32), 300, 3000)
    depth_view = ((3000 - depth_view) * (255.0 / 2700)).astype(np.uint8)
    depth_view = cv2.applyColorMap(depth_view, cv2.COLORMAP_TURBO)
    depth_view[~valid] = 0
    comparison = np.concatenate([color, depth_view], axis=1)

    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(prefix) + "_color.png", color):
        raise RuntimeError("Failed to save color image")
    if not cv2.imwrite(str(prefix) + "_depth_mm.png", depth):
        raise RuntimeError("Failed to save depth image")
    if not cv2.imwrite(str(prefix) + "_comparison.png", comparison):
        raise RuntimeError("Failed to save comparison image")
    values = depth[valid]
    report = {
        "color_frame": color_msg.header.frame_id,
        "depth_frame": depth_msg.header.frame_id,
        "pair_dt_ms": round(delta * 1000, 3),
        "size": [int(color.shape[1]), int(color.shape[0])],
        "valid_under_3m_fraction": round(float(valid.mean()), 4),
        "median_valid_depth_mm": int(np.median(values)) if values.size else None,
        "frames_received": {"color": len(colors), "depth": len(depths)},
    }
    prefix.with_name(prefix.name + "_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False))
    node.destroy_subscription(color_sub)
    node.destroy_subscription(depth_sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

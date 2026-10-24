#!/usr/bin/env python3
"""Passive RGB-D semantic candidate probe (not connected to Nav2).

The Gemini camera is physically inverted. Detection runs on an upright copy,
while depth lookup and 3-D projection use the original camera image coordinates.
Ultralytics is imported lazily; review its license before production use.
"""

import argparse
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np


def raw_box(upright_box, width, height):
    x1, y1, x2, y2 = map(float, upright_box)
    return [width - x2, height - y2, width - x1, height - y1]


def depth_location(depth_mm, box, intrinsics, min_valid_pixels=80, raw_mask=None):
    """Estimate a foreground surface, not the median of a background-filled box."""
    height, width = depth_mm.shape
    x1, y1, x2, y2 = box
    x1 = max(0, int(np.floor(x1)))
    y1 = max(0, int(np.floor(y1)))
    x2 = min(width, int(np.ceil(x2)))
    y2 = min(height, int(np.ceil(y2)))
    if x2 - x1 < 8 or y2 - y1 < 8:
        return None
    if raw_mask is None:
        # Box-only detectors need an interior crop to reduce edge contamination.
        mx, my = int((x2 - x1) * 0.12), int((y2 - y1) * 0.12)
        x1, y1, x2, y2 = x1 + mx, y1 + my, x2 - mx, y2 - my
        region_mask = None
    else:
        if raw_mask.shape != depth_mm.shape:
            raise ValueError("Segmentation mask and registered depth dimensions differ")
        region_mask = raw_mask[y1:y2, x1:x2].astype(np.uint8)
        region_mask = cv2.erode(region_mask, np.ones((3, 3), np.uint8)) > 0
    roi = depth_mm[y1:y2, x1:x2]
    valid = (roi >= 300) & (roi <= 4000)
    if region_mask is not None:
        valid &= region_mask
    if np.count_nonzero(valid) < min_valid_pixels:
        return None
    values = roi[valid]
    # A chair box contains substantial background through its legs and seat.
    # Select the near-depth mode, not the box-wide median or quartile.
    surface_mm = float(np.percentile(values, 10))
    band = valid & (np.abs(roi.astype(np.float32) - surface_mm) <= 100)
    if np.count_nonzero(band) < min_valid_pixels:
        return None
    yy, xx = np.where(band)
    u = float(np.median(xx) + x1)
    v = float(np.median(yy) + y1)
    z = float(np.median(roi[band])) / 1000.0
    fx, fy, cx, cy = intrinsics
    return {
        "pixel_raw": [round(u, 1), round(v, 1)],
        "xyz_camera_optical_m": [round((u - cx) * z / fx, 3),
                                 round((v - cy) * z / fy, 3), round(z, 3)],
        "support_pixels": int(np.count_nonzero(band)),
        "depth_iqr_m": round(float(np.subtract(*np.percentile(roi[band], [75, 25]))) / 1000, 3),
    }


def detect_pair(color_bgr, depth_mm, intrinsics, model, confidence):
    if color_bgr.shape[:2] != depth_mm.shape:
        raise ValueError("RGB and aligned depth dimensions differ")
    height, width = depth_mm.shape
    upright = cv2.rotate(color_bgr, cv2.ROTATE_180)
    result = model(upright, conf=confidence, retina_masks=True, verbose=False)[0]
    candidates = []
    for index, detection in enumerate(result.boxes):
        detection_confidence = float(detection.conf)
        if detection_confidence <= max(0.70, confidence):
            continue
        box_upright = [float(value) for value in detection.xyxy[0]]
        box_raw = raw_box(box_upright, width, height)
        raw_mask = None
        if result.masks is not None and index < len(result.masks.data):
            upright_mask = result.masks.data[index].cpu().numpy().astype(np.uint8)
            if upright_mask.shape != depth_mm.shape:
                upright_mask = cv2.resize(upright_mask, (width, height), interpolation=cv2.INTER_NEAREST)
            raw_mask = cv2.rotate(upright_mask, cv2.ROTATE_180)
        location = depth_location(depth_mm, box_raw, intrinsics, raw_mask=raw_mask)
        if location is None:
            continue
        candidates.append({
            "class": result.names[int(detection.cls)],
            "confidence": detection_confidence,
            "box_upright": [round(value, 1) for value in box_upright],
            "box_raw": [round(value, 1) for value in box_raw],
            "depth_source": "instance_mask" if raw_mask is not None else "box_interior",
            **location,
        })
    return candidates


class CandidateTracker:
    """Conservative per-object IDs; stale tracks expire instead of becoming a map."""

    def __init__(self, confirm_frames=3, stale_seconds=0.8):
        self.confirm_frames = confirm_frames
        self.stale_seconds = stale_seconds
        self.tracks = {}
        self.next_id = 1

    def update(self, candidates, stamp_seconds):
        removed = [track_id for track_id, track in self.tracks.items()
                   if stamp_seconds - track["last_seen"] > self.stale_seconds]
        for track_id in removed:
            del self.tracks[track_id]
        used = set()
        confirmed = []
        for item in candidates:
            position = np.asarray(item["xyz_camera_optical_m"])
            pixel = np.asarray(item["pixel_raw"])
            possibilities = []
            for track_id, track in self.tracks.items():
                if track_id in used or track["class"] != item["class"]:
                    continue
                spatial = float(np.linalg.norm(position - track["position"]))
                image = float(np.linalg.norm(pixel - track["pixel"]))
                if spatial < 0.35 and image < 80:
                    possibilities.append((spatial + image / 500, track_id))
            if possibilities:
                _, track_id = min(possibilities)
                track = self.tracks[track_id]
                track["hits"] = track["hits"] + 1 if stamp_seconds - track["last_seen"] < 0.25 else 1
            else:
                track_id = self.next_id
                self.next_id += 1
                track = {"class": item["class"], "hits": 1}
                self.tracks[track_id] = track
            track.update(position=position, pixel=pixel, last_seen=stamp_seconds)
            used.add(track_id)
            item["track_id"] = track_id
            item["track_hits"] = track["hits"]
            if track["hits"] >= self.confirm_frames:
                confirmed.append(item)
        return confirmed, removed


def transform_point(point, transform):
    """Apply a geometry_msgs Transform to a point without extra dependencies."""
    q = transform.rotation
    vector = np.asarray(point, dtype=np.float64)
    quat_vector = np.array([q.x, q.y, q.z], dtype=np.float64)
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm < 1e-12:
        raise ValueError("Invalid TF quaternion")
    quat_vector /= np.sqrt(norm)
    scalar = q.w / np.sqrt(norm)
    rotated = vector + 2 * np.cross(quat_vector,
                                    np.cross(quat_vector, vector) + scalar * vector)
    return [round(float(rotated[0] + transform.translation.x), 3),
            round(float(rotated[1] + transform.translation.y), 3),
            round(float(rotated[2] + transform.translation.z), 3)]


def run_offline(args, model):
    color = cv2.imread(str(args.color), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None or depth.dtype != np.uint16:
        raise RuntimeError("Need a readable BGR image and uint16 depth-mm PNG")
    intrinsics = (args.fx, args.fy, args.cx, args.cy)
    return detect_pair(color, depth, intrinsics, model, args.confidence)


def run_live(args, model):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener
    from rclpy.duration import Duration
    from rclpy.time import Time

    rclpy.init()
    node = rclpy.create_node("semantic_rgbd_candidate_probe")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    bridge = CvBridge()
    colors, depths = deque(maxlen=8), deque(maxlen=8)
    camera_info = []
    node.create_subscription(Image, args.color_topic, colors.append, qos_profile_sensor_data)
    node.create_subscription(Image, args.depth_topic, depths.append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.info_topic, lambda msg: camera_info.append(msg), qos_profile_sensor_data)
    tracker = CandidateTracker(args.confirm_frames, args.track_stale_seconds)
    processed = set()
    started = time.monotonic()
    report_file = None
    if args.output_jsonl:
        args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
        report_file = args.output_jsonl.open("w", encoding="utf-8")
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not colors or not depths or not camera_info:
                continue
            color_msg = colors[-1]
            stamp = (color_msg.header.stamp.sec, color_msg.header.stamp.nanosec)
            if stamp in processed:
                continue
            processed.add(stamp)
            if len(processed) > 100:
                processed.clear()
            depth_msg = min(depths, key=lambda msg: abs(
                (msg.header.stamp.sec - stamp[0]) + (msg.header.stamp.nanosec - stamp[1]) * 1e-9))
            dt = abs((depth_msg.header.stamp.sec - stamp[0]) +
                     (depth_msg.header.stamp.nanosec - stamp[1]) * 1e-9)
            if dt > 0.033 or color_msg.header.frame_id != depth_msg.header.frame_id:
                continue
            color = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            info = camera_info[-1]
            if info.header.frame_id != color_msg.header.frame_id or depth.dtype != np.uint16:
                continue
            intrinsics = (info.k[0], info.k[4], info.k[2], info.k[5])
            candidates = detect_pair(color, depth, intrinsics, model, args.confidence)
            frame_seconds = stamp[0] + stamp[1] * 1e-9
            confirmed, removed = tracker.update(candidates, frame_seconds)
            tf_status = "no_confirmed_candidate"
            if confirmed:
                try:
                    stamped_transform = tf_buffer.lookup_transform(
                        args.target_frame, color_msg.header.frame_id,
                        Time.from_msg(color_msg.header.stamp),
                        timeout=Duration(seconds=0.1))
                    for item in confirmed:
                        item["xyz_" + args.target_frame + "_m"] = transform_point(
                            item["xyz_camera_optical_m"], stamped_transform.transform)
                    tf_status = "ok"
                except TransformException as error:
                    tf_status = str(error)
            report = {"stamp": stamp, "frame": color_msg.header.frame_id,
                      "pair_dt_ms": round(dt * 1000, 2),
                      "candidates": candidates, "confirmed": confirmed,
                      "removed_track_ids": removed,
                      "tf_status": tf_status}
            line = json.dumps(report)
            if report_file:
                report_file.write(line + "\n")
                report_file.flush()
            if not args.quiet:
                print(line, flush=True)
    finally:
        if report_file:
            report_file.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-family", choices=("yolo", "yoloe"), default="yolo")
    parser.add_argument("--classes", nargs="*", help="YOLOE text-prompt classes; omit for prompt-free weights")
    parser.add_argument("--confidence", type=float, default=0.70,
                        help="Minimum detector threshold; semantic candidates must be strictly above 0.70")
    parser.add_argument("--color", type=Path)
    parser.add_argument("--depth", type=Path)
    parser.add_argument("--fx", type=float, default=365.41540527)
    parser.add_argument("--fy", type=float, default=365.44766235)
    parser.add_argument("--cx", type=float, default=319.62155151)
    parser.add_argument("--cy", type=float, default=241.0037384)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--track-stale-seconds", type=float, default=0.8)
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--info-topic", default="/camera/color/camera_info")
    parser.add_argument("--target-frame", default="base_footprint")
    parser.add_argument("--output-jsonl", type=Path)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    if not 0.70 <= args.confidence < 1.0:
        parser.error("--confidence must be in [0.70, 1.0); candidates at exactly 0.70 are rejected")
    if (args.color is None) != (args.depth is None):
        parser.error("--color and --depth must be supplied together")
    from ultralytics import YOLO, YOLOE
    model = YOLOE(str(args.model)) if args.model_family == "yoloe" else YOLO(str(args.model))
    if args.classes:
        if args.model_family != "yoloe":
            parser.error("--classes requires --model-family yoloe")
        model.set_classes(args.classes)
    if args.color:
        print(json.dumps(run_offline(args, model), indent=2))
    else:
        run_live(args, model)


if __name__ == "__main__":
    main()

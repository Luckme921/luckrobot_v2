#!/usr/bin/env python3
"""Render every detector box on an upright RGB frame for human review."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from semantic_rgbd_candidate import depth_location, raw_box


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-family", choices=("yolo", "yoloe"), default="yolo")
    parser.add_argument("--classes", nargs="*", help="YOLOE text-prompt classes; omit for prompt-free weights")
    parser.add_argument("--color", type=Path, required=True)
    parser.add_argument("--depth", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--fx", type=float, default=365.41540527)
    parser.add_argument("--fy", type=float, default=365.44766235)
    parser.add_argument("--cx", type=float, default=319.62155151)
    parser.add_argument("--cy", type=float, default=241.0037384)
    args = parser.parse_args()

    from ultralytics import YOLO, YOLOE

    color = cv2.imread(str(args.color), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None or depth.dtype != np.uint16:
        raise RuntimeError("Expected readable RGB image and uint16 depth-mm PNG")
    if color.shape[:2] != depth.shape:
        raise RuntimeError("RGB and depth sizes differ; use registered depth")
    upright = cv2.rotate(color, cv2.ROTATE_180)
    model = YOLOE(str(args.model)) if args.model_family == "yoloe" else YOLO(str(args.model))
    if args.classes:
        if args.model_family != "yoloe":
            parser.error("--classes requires --model-family yoloe")
        model.set_classes(args.classes)
    result = model(upright, conf=args.confidence, retina_masks=True, verbose=False)[0]
    detections = []
    height, width = depth.shape
    for index, detection in enumerate(result.boxes, 1):
        box = [float(value) for value in detection.xyxy[0]]
        raw = raw_box(box, width, height)
        upright_mask = None
        if result.masks is not None and index - 1 < len(result.masks.data):
            upright_mask = result.masks.data[index - 1].cpu().numpy().astype(np.uint8)
            if upright_mask.shape != depth.shape:
                upright_mask = cv2.resize(upright_mask, (width, height), interpolation=cv2.INTER_NEAREST)
        raw_mask = cv2.rotate(upright_mask, cv2.ROTATE_180) if upright_mask is not None else None
        position = depth_location(depth, raw, (args.fx, args.fy, args.cx, args.cy), raw_mask=raw_mask)
        label = result.names[int(detection.cls)]
        confidence = round(float(detection.conf), 3)
        distance = position["xyz_camera_optical_m"][2] if position else None
        text = f"#{index} {label} {confidence:.2f}"
        if distance is not None:
            text += f" {distance:.2f}m"
        x1, y1, x2, y2 = [int(round(value)) for value in box]
        cv2.rectangle(upright, (x1, y1), (x2, y2), (255, 0, 255), 2)
        if upright_mask is not None:
            contours, _ = cv2.findContours(upright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(upright, contours, -1, (0, 255, 0), 1)
        cv2.rectangle(upright, (x1, max(0, y1 - 22)),
                      (min(width - 1, x1 + max(140, len(text) * 10)), y1),
                      (50, 0, 50), -1)
        cv2.putText(upright, text, (x1 + 2, max(16, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)
        detections.append({"id": index, "class": label, "confidence": confidence,
                           "box_upright": [round(value, 1) for value in box],
                           "depth_source": "instance_mask" if raw_mask is not None else "box_interior",
                           "surface_camera_optical_m":
                               position["xyz_camera_optical_m"] if position else None})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.out), upright):
        raise RuntimeError("Could not write annotated image")
    args.out.with_suffix(".json").write_text(
        json.dumps(detections, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"image": str(args.out), "detections": detections}, ensure_ascii=False))


if __name__ == "__main__":
    main()

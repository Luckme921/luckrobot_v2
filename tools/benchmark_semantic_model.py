#!/usr/bin/env python3
"""Measure warm single-frame semantic inference on the target Jetson."""

import argparse
import json
import resource
import statistics
import time
from pathlib import Path

import cv2
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-family", choices=("yolo", "yoloe"), default="yolo")
    parser.add_argument("--classes", nargs="*")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()

    from ultralytics import YOLO, YOLOE

    image = cv2.imread(str(args.image))
    if image is None:
        raise RuntimeError(f"Cannot read {args.image}")
    image = cv2.rotate(image, cv2.ROTATE_180)
    model = YOLOE(str(args.model)) if args.model_family == "yoloe" else YOLO(str(args.model))
    if args.classes:
        if args.model_family != "yoloe":
            parser.error("--classes requires --model-family yoloe")
        model.set_classes(args.classes)
    if not torch.cuda.is_available():
        raise RuntimeError("Jetson CUDA is not available")

    def infer():
        result = model.predict(image, device=0, imgsz=640, conf=0.25,
                               retina_masks=True, verbose=False)[0]
        torch.cuda.synchronize()
        return result

    for _ in range(args.warmup):
        infer()
    torch.cuda.reset_peak_memory_stats()
    timings = []
    detections = []
    for _ in range(args.frames):
        started = time.perf_counter()
        result = infer()
        timings.append((time.perf_counter() - started) * 1000)
        detections.append(len(result.boxes))
    timings.sort()
    print(json.dumps({
        "model": args.model.name,
        "classes": args.classes,
        "frames": args.frames,
        "median_ms": round(statistics.median(timings), 2),
        "p95_ms": round(timings[min(len(timings)-1, int(len(timings)*0.95))], 2),
        "peak_cuda_mb": round(torch.cuda.max_memory_allocated() / 1024**2, 1),
        "max_process_rss_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1),
        "detections_per_frame": detections[-1],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Capture compact, target-based camera/2D-lidar calibration samples.

The robot and calibration board must remain stationary while each sample is
captured.  One synchronized-ish snapshot is saved per board pose instead of a
large raw-image rosbag.  Spatial calibration is intentionally performed later,
offline, after the target dimensions are known and the samples pass QA.
"""

import argparse
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
import tarfile
import threading
import time

import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def header_dict(msg):
    return {
        "stamp_ns": stamp_ns(msg),
        "frame_id": msg.header.frame_id,
    }


def camera_info_dict(msg: CameraInfo):
    return {
        **header_dict(msg),
        "height": int(msg.height),
        "width": int(msg.width),
        "distortion_model": msg.distortion_model,
        "d": list(msg.d),
        "k": list(msg.k),
        "r": list(msg.r),
        "p": list(msg.p),
        "binning_x": int(msg.binning_x),
        "binning_y": int(msg.binning_y),
        "roi": {
            "x_offset": int(msg.roi.x_offset),
            "y_offset": int(msg.roi.y_offset),
            "height": int(msg.roi.height),
            "width": int(msg.roi.width),
            "do_rectify": bool(msg.roi.do_rectify),
        },
    }


def image_array(msg: Image):
    """Decode the common Orbbec image encodings without cv_bridge."""
    enc = msg.encoding.lower()
    if enc in ("rgb8", "bgr8"):
        dtype, channels = np.uint8, 3
    elif enc in ("rgba8", "bgra8"):
        dtype, channels = np.uint8, 4
    elif enc in ("mono8", "8uc1"):
        dtype, channels = np.uint8, 1
    elif enc in ("mono16", "16uc1"):
        dtype, channels = np.dtype(">u2" if msg.is_bigendian else "<u2"), 1
    elif enc in ("32fc1",):
        dtype, channels = np.dtype(">f4" if msg.is_bigendian else "<f4"), 1
    else:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    item_size = np.dtype(dtype).itemsize
    if msg.step % item_size:
        raise ValueError(f"invalid image step {msg.step} for {msg.encoding}")
    row_items = msg.step // item_size
    raw = np.frombuffer(bytes(msg.data), dtype=dtype)
    expected = int(msg.height) * row_items
    if raw.size < expected:
        raise ValueError(f"short image buffer: {raw.size} < {expected}")
    rows = raw[:expected].reshape(int(msg.height), row_items)
    useful = rows[:, : int(msg.width) * channels]
    if channels == 1:
        return useful.reshape(int(msg.height), int(msg.width)).copy()
    return useful.reshape(int(msg.height), int(msg.width), channels).copy()


def save_image_npz(path: Path, msg: Image):
    np.savez_compressed(
        path,
        image=image_array(msg),
        encoding=np.asarray(msg.encoding),
        frame_id=np.asarray(msg.header.frame_id),
        stamp_ns=np.asarray(stamp_ns(msg), dtype=np.int64),
    )


def save_scan_npz(path: Path, msg: LaserScan):
    np.savez_compressed(
        path,
        ranges=np.asarray(msg.ranges, dtype=np.float32),
        intensities=np.asarray(msg.intensities, dtype=np.float32),
        angle_min=np.asarray(msg.angle_min),
        angle_max=np.asarray(msg.angle_max),
        angle_increment=np.asarray(msg.angle_increment),
        time_increment=np.asarray(msg.time_increment),
        scan_time=np.asarray(msg.scan_time),
        range_min=np.asarray(msg.range_min),
        range_max=np.asarray(msg.range_max),
        frame_id=np.asarray(msg.header.frame_id),
        stamp_ns=np.asarray(stamp_ns(msg), dtype=np.int64),
    )


def detect_checkerboard(image, encoding, cols, rows):
    if cv2 is None:
        raise RuntimeError("缺少 python3-opencv；请先执行 sudo apt install python3-opencv")
    enc = encoding.lower()
    if image.ndim == 2:
        gray = image.astype(np.uint8) if image.dtype != np.uint8 else image
        preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    elif enc == "rgb8":
        preview = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    elif enc in ("rgba8",):
        preview = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        gray = cv2.cvtColor(image, cv2.COLOR_RGBA2GRAY)
    elif enc in ("bgra8",):
        preview = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    else:
        preview = image[:, :, :3].copy()
        gray = cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY)

    pattern = (cols, rows)
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    else:
        found, corners = cv2.findChessboardCorners(
            gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if found:
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01),
            )
    if not found or corners is None or len(corners) != cols * rows:
        raise RuntimeError(f"未检测到完整 {cols}x{rows} 内角点；调整距离、照明或姿态")

    grid = corners.reshape(rows, cols, 2)
    spacings = []
    if cols > 1:
        spacings.extend(np.linalg.norm(grid[:, 1:] - grid[:, :-1], axis=2).ravel())
    if rows > 1:
        spacings.extend(np.linalg.norm(grid[1:] - grid[:-1], axis=2).ravel())
    median_spacing = float(np.median(np.asarray(spacings)))
    cv2.drawChessboardCorners(preview, pattern, corners, True)
    return corners.reshape(-1, 2), median_spacing, preview


def tf_dict(msg: TFMessage):
    values = []
    for item in msg.transforms:
        values.append(
            {
                "stamp_ns": stamp_ns(item),
                "parent": item.header.frame_id,
                "child": item.child_frame_id,
                "translation": [
                    item.transform.translation.x,
                    item.transform.translation.y,
                    item.transform.translation.z,
                ],
                "rotation_xyzw": [
                    item.transform.rotation.x,
                    item.transform.rotation.y,
                    item.transform.rotation.z,
                    item.transform.rotation.w,
                ],
            }
        )
    return values


class CalibrationCapture(Node):
    def __init__(self, args):
        super().__init__("camera_lidar_calibration_capture")
        self.args = args
        self.lock = threading.Lock()
        self.messages = {}
        self.histories = {
            "color": deque(maxlen=30),
            "depth": deque(maxlen=30),
            "scan": deque(maxlen=30),
            "scan_filtered": deque(maxlen=30),
            "scan_camera": deque(maxlen=30),
        }
        self.tf_static = {}

        self.create_subscription(Image, args.color_topic, self._history_cb("color"), qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self._history_cb("depth"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.color_info_topic, self._latest_cb("color_info"), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.depth_info_topic, self._latest_cb("depth_info"), qos_profile_sensor_data)
        self.create_subscription(LaserScan, args.scan_topic, self._history_cb("scan"), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, args.filtered_scan_topic, self._history_cb("scan_filtered"), qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, args.camera_scan_topic, self._history_cb("scan_camera"), qos_profile_sensor_data
        )

        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, tf_qos)

    def _history_cb(self, name):
        def callback(msg):
            with self.lock:
                self.histories[name].append(msg)

        return callback

    def _latest_cb(self, name):
        def callback(msg):
            with self.lock:
                self.messages[name] = msg

        return callback

    def _tf_static_cb(self, msg):
        with self.lock:
            for item in msg.transforms:
                self.tf_static[item.child_frame_id] = item

    @staticmethod
    def _nearest(history, target_ns):
        if not history:
            return None
        return min(history, key=lambda value: abs(stamp_ns(value) - target_ns))

    def readiness(self):
        with self.lock:
            missing = []
            for name in ("color", "depth", "scan"):
                if not self.histories[name]:
                    missing.append(name)
            for name in ("color_info", "depth_info"):
                if name not in self.messages:
                    missing.append(name)
            return missing

    def capture(self, output_dir: Path, index: int):
        with self.lock:
            if self.readiness_unlocked():
                raise RuntimeError("required topics are not ready: " + ", ".join(self.readiness_unlocked()))

            color = self.histories["color"][-1]
            target_ns = stamp_ns(color)
            depth = self._nearest(self.histories["depth"], target_ns)
            scan = self._nearest(self.histories["scan"], target_ns)
            scan_filtered = self._nearest(self.histories["scan_filtered"], target_ns)
            scan_camera = self._nearest(self.histories["scan_camera"], target_ns)
            color_info = self.messages["color_info"]
            depth_info = self.messages["depth_info"]
            static_tf = list(self.tf_static.values())

        critical = [color, depth, scan]
        spread_ms = (max(stamp_ns(x) for x in critical) - min(stamp_ns(x) for x in critical)) / 1e6
        if spread_ms > self.args.max_spread_ms:
            raise RuntimeError(
                f"color/depth/lidar timestamp spread {spread_ms:.1f} ms exceeds "
                f"{self.args.max_spread_ms:.1f} ms"
            )

        color_array = image_array(color)
        corners, square_pixels, checkerboard_preview = detect_checkerboard(
            color_array,
            color.encoding,
            self.args.checkerboard_cols,
            self.args.checkerboard_rows,
        )
        if square_pixels < self.args.min_square_pixels:
            raise RuntimeError(
                f"棋盘仅约 {square_pixels:.2f} px/格，小于 {self.args.min_square_pixels:.2f} px；请靠近"
            )

        pose_dir = output_dir / f"pose_{index:02d}"
        tmp_dir = output_dir / f".pose_{index:02d}.tmp"
        if tmp_dir.exists():
            shutil.rmtree(tmp_dir)
        tmp_dir.mkdir(parents=False)

        try:
            save_image_npz(tmp_dir / "color.npz", color)
            save_image_npz(tmp_dir / "depth.npz", depth)
            cv2.imwrite(str(tmp_dir / "checkerboard_preview.jpg"), checkerboard_preview)
            np.savez_compressed(
                tmp_dir / "checkerboard_corners.npz",
                corners_px=corners.astype(np.float32),
                square_pixel_median=np.asarray(square_pixels),
                pattern_cols=np.asarray(self.args.checkerboard_cols),
                pattern_rows=np.asarray(self.args.checkerboard_rows),
            )
            save_scan_npz(tmp_dir / "scan_raw.npz", scan)
            if scan_filtered is not None:
                save_scan_npz(tmp_dir / "scan_filtered.npz", scan_filtered)
            if scan_camera is not None:
                save_scan_npz(tmp_dir / "scan_camera.npz", scan_camera)

            metadata = {
                "capture_index": index,
                "captured_at_local": datetime.now().astimezone().isoformat(),
                "timestamp_spread_ms": spread_ms,
                "checkerboard_square_pixel_median": square_pixels,
                "topics": vars(self.args),
                "color": header_dict(color),
                "depth": header_dict(depth),
                "scan_raw": header_dict(scan),
                "scan_filtered": header_dict(scan_filtered) if scan_filtered else None,
                "scan_camera": header_dict(scan_camera) if scan_camera else None,
                "color_camera_info": camera_info_dict(color_info),
                "depth_camera_info": camera_info_dict(depth_info),
                "tf_static": tf_dict(TFMessage(transforms=static_tf)),
            }
            (tmp_dir / "metadata.json").write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            if pose_dir.exists():
                shutil.rmtree(pose_dir)
            tmp_dir.rename(pose_dir)
        except Exception:
            shutil.rmtree(tmp_dir, ignore_errors=True)
            raise
        return spread_ms

    def readiness_unlocked(self):
        missing = []
        for name in ("color", "depth", "scan"):
            if not self.histories[name]:
                missing.append(name)
        for name in ("color_info", "depth_info"):
            if name not in self.messages:
                missing.append(name)
        return missing


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=int, default=12, help="number of target poses (default: 12)")
    parser.add_argument("--max-spread-ms", type=float, default=150.0)
    parser.add_argument("--output", help="output directory; default is ~/camera_lidar_calib_TIMESTAMP")
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--color-info-topic", default="/camera/color/camera_info")
    parser.add_argument("--depth-info-topic", default="/camera/depth/camera_info")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--filtered-scan-topic", default="/scan_filtered")
    parser.add_argument("--camera-scan-topic", default="/scan_camera")
    parser.add_argument("--checkerboard-cols", type=int, default=11, help="inner-corner columns")
    parser.add_argument("--checkerboard-rows", type=int, default=8, help="inner-corner rows")
    parser.add_argument("--square-size-mm", type=float, default=10.0)
    parser.add_argument("--board-width-mm", type=float, default=150.0)
    parser.add_argument("--board-height-mm", type=float, default=150.0)
    parser.add_argument("--board-thickness-mm", type=float, default=0.0)
    parser.add_argument("--board-bottom-mm", type=float, default=80.0)
    parser.add_argument("--min-square-pixels", type=float, default=4.0)
    return parser


def main():
    args = build_parser().parse_args()
    if args.poses < 6:
        raise SystemExit("--poses 至少为 6，推荐 12")
    if args.max_spread_ms <= 0:
        raise SystemExit("--max-spread-ms 必须大于 0")
    if args.checkerboard_cols < 3 or args.checkerboard_rows < 3:
        raise SystemExit("棋盘内角点数量不合法")
    if min(args.square_size_mm, args.board_width_mm, args.board_height_mm) <= 0:
        raise SystemExit("棋盘尺寸必须大于 0")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output).expanduser() if args.output else Path.home() / f"camera_lidar_calib_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = CalibrationCapture(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("相机—2D 雷达靶标采集（机器人全程不得移动）")
    print(f"输出目录：{output_dir}")
    print(
        f"棋盘：{args.checkerboard_cols}x{args.checkerboard_rows} 内角点，"
        f"方格 {args.square_size_mm:.3f} mm；板 {args.board_width_mm:.1f}x"
        f"{args.board_height_mm:.1f} mm，下边缘离地 {args.board_bottom_mm:.1f} mm"
    )
    print("等待彩色、深度、原始雷达和相机内参……")
    try:
        deadline = time.monotonic() + 20.0
        while True:
            missing = node.readiness()
            if not missing:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("20 秒内未收到：" + ", ".join(missing))
            time.sleep(0.25)

        print("数据已就绪。每次只移动标定板，放稳后等待 2 秒，再按 Enter。")
        print("输入 q 可提前结束；错误样本输入 r 可删除最后一组并重采。")
        index = 1
        while index <= args.poses:
            command = input(f"[{index}/{args.poses}] 标定板放稳后按 Enter：").strip().lower()
            if command == "q":
                break
            if command == "r":
                previous = output_dir / f"pose_{index - 1:02d}"
                if index > 1 and previous.exists():
                    shutil.rmtree(previous)
                    index -= 1
                    print(f"已删除 pose_{index:02d}，请重新采集。")
                else:
                    print("没有可删除的上一组。")
                continue
            try:
                spread_ms = node.capture(output_dir, index)
            except Exception as exc:
                print(f"本组未保存：{exc}")
                continue
            print(f"已保存 pose_{index:02d}；关键消息时间跨度 {spread_ms:.1f} ms")
            index += 1

        captured = index - 1
        summary = {
            "format_version": 1,
            "captured_poses": captured,
            "requested_poses": args.poses,
            "completed": captured == args.poses,
            "target": {
                "checkerboard_inner_corners_cols": args.checkerboard_cols,
                "checkerboard_inner_corners_rows": args.checkerboard_rows,
                "square_size_mm": args.square_size_mm,
                "board_width_mm": args.board_width_mm,
                "board_height_mm": args.board_height_mm,
                "board_thickness_mm": args.board_thickness_mm,
                "board_bottom_mm": args.board_bottom_mm,
            },
            "note": "Target dimensions and pose labels must be supplied separately before solving extrinsics.",
        }
        (output_dir / "capture_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    except KeyboardInterrupt:
        captured = len(list(output_dir.glob("pose_*")))
        print(f"\n收到中断，保留 {captured} 组已采样数据。")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    archive = output_dir.with_suffix(".tar.gz")
    with tarfile.open(archive, "w:gz") as handle:
        handle.add(output_dir, arcname=output_dir.name)
    print(f"采集包：{archive}")
    print("请同时提供棋盘内角点数量、方格边长、背板尺寸和每组摆放示意。")


if __name__ == "__main__":
    main()

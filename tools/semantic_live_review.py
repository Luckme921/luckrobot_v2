#!/usr/bin/env python3
"""Local-only, passive RGB-D semantic review page with click-to-save feedback."""

import argparse
import gc
import json
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from align_depth_to_color import register_depth
from semantic_rgbd_candidate import depth_location, raw_box, transform_point


PAGE = """<!doctype html><html lang="zh"><meta charset="utf-8"><title>语义检测审核</title>
<style>body{background:#20242b;color:#eee;font:16px sans-serif;margin:20px} img{width:min(96vw,960px);height:auto;border:2px solid #555;cursor:crosshair}button{padding:10px;margin:5px;font-size:16px}input{font-size:16px;padding:8px}#status{margin:10px 0}#point{color:#8ee}</style>
<h2>语义检测审核（仅本机；不控制机器人）</h2><p>模型：<select id="model"></select><button onclick="switchModel()">切换模型</button><span id="modelHint"></span></p><div id="status">等待相机...</div>
<img id="view" width="640" height="480" alt="等待相机画面"><div id="point">先点击画面中的物体</div>
<p><button onclick="send('correct')">框和类别正确</button><button onclick="send('wrong_class')">类别错误</button><button onclick="send('missing')">漏检</button><button onclick="send('bad_depth')">深度错误</button><button onclick="resume()">取消暂停 / 继续</button></p>
<p>正确类别（可选）：<input id="label" placeholder="如：玩偶 / 书包"> 备注（可选）：<input id="note" placeholder="如：被椅子遮挡"></p>
<div id="result"></div>
<script>
let fid=null, displayedFrame=null, selectedFrame=null, point=null, paused=false, modelsLoaded=false;
const view=document.getElementById('view');
view.onload=()=>{displayedFrame=new URL(view.currentSrc).searchParams.get('id');};
view.onclick=async e=>{if(!displayedFrame)return;const r=view.getBoundingClientRect();point={x:Math.round((e.clientX-r.left)*640/r.width),y:Math.round((e.clientY-r.top)*480/r.height)};selectedFrame=displayedFrame;paused=true;try{const resp=await fetch('/pin',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({frame_id:selectedFrame})});if(!resp.ok)throw new Error((await resp.json()).error);document.getElementById('point').textContent=`已暂停在画面 ${selectedFrame}，坐标 (${point.x}, ${point.y})；填写反馈后点击按钮`;}catch(err){resume();document.getElementById('result').textContent='画面已过期，请重新点击：'+err;}};
function resume(){paused=false;selectedFrame=null;point=null;document.getElementById('point').textContent='先点击画面中的物体';}
async function switchModel(){const name=document.getElementById('model').value;resume();const r=await fetch('/switch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({model:name})});const data=await r.json();document.getElementById('result').textContent=r.ok?`正在切换至 ${name}；可能等待数秒`:`切换失败：${data.error}`;}
async function refresh(){try{const s=await (await fetch('/state',{cache:'no-store'})).json();const pick=document.getElementById('model');if(!modelsLoaded){for(const [key,label] of Object.entries(s.models)){const o=document.createElement('option');o.value=key;o.textContent=label;pick.appendChild(o);}modelsLoaded=true;pick.value=s.active_model||s.requested_model;}document.getElementById('modelHint').textContent=s.frame_model==='yolo11s-box'?'仅检测框；距离估计不宜用于建图':'';document.getElementById('status').textContent=`画面模型 ${s.frame_model||'加载中'} ｜${s.switching?'切换中，画面可能暂时停顿':s.error||'运行中'} ｜检测 ${s.detections||0} 个 ｜推理 ${s.inference_ms||0} ms ｜${paused?'已暂停供标注':'实时画面'}`;if(!paused&&s.id&&s.id!==fid){fid=s.id;view.src='/frame.jpg?id='+encodeURIComponent(fid);}}catch(e){document.getElementById('status').textContent='连接中断';}}
async function send(kind){if(!selectedFrame||!point){document.getElementById('result').textContent='请先点击画面中的目标';return;}const body={frame_id:selectedFrame,kind,x:point.x,y:point.y,label:document.getElementById('label').value,note:document.getElementById('note').value};const r=await fetch('/feedback',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await r.json();document.getElementById('result').textContent=r.ok?`已保存 #${data.feedback_id}（${data.detected_class||'无检测框'}）`:`未保存：${data.error}`;if(r.ok)resume();}
setInterval(refresh,500);refresh();
</script></html>"""


class ReviewState:
    def __init__(self, output_dir, models, initial_model):
        self.output_dir = output_dir
        self.lock = threading.Lock()
        self.frames = deque(maxlen=15)
        self.pinned = {}
        self.error = ""
        self.feedback_id = 0
        self.models = models
        self.active_model = None
        self.requested_model = initial_model
        self.switching = True
        self.inference_ms = 0

    def add(self, frame):
        with self.lock:
            self.frames.append(frame)
            self.error = ""
            self.switching = False

    def get(self, frame_id=None):
        with self.lock:
            if not self.frames:
                return None
            if frame_id is None:
                return self.frames[-1]
            return self.pinned.get(frame_id) or next((f for f in self.frames if f["id"] == frame_id), None)

    def pin(self, frame_id):
        frame = self.get(frame_id)
        if frame is None:
            raise ValueError("画面已过期，请重新点击")
        with self.lock:
            if len(self.pinned) >= 8:
                self.pinned.pop(next(iter(self.pinned)))
            self.pinned[frame_id] = frame

    def save_feedback(self, payload):
        if payload.get("kind") not in {"correct", "wrong_class", "missing", "bad_depth"}:
            raise ValueError("无效反馈类型")
        point = (int(payload["x"]), int(payload["y"]))
        if not (0 <= point[0] < 640 and 0 <= point[1] < 480):
            raise ValueError("点击位置超出画面")
        frame = self.get(payload.get("frame_id"))
        if frame is None:
            raise ValueError("画面已过期，请在当前画面重新点击")
        hits = [d for d in frame["detections"] if d["box"][0] <= point[0] <= d["box"][2]
                and d["box"][1] <= point[1] <= d["box"][3]]
        selected = min(hits, key=lambda d: (d["box"][2]-d["box"][0])*(d["box"][3]-d["box"][1])) if hits else None
        with self.lock:
            self.feedback_id += 1
            feedback_id = self.feedback_id
        stem = f"{frame['id']}_{feedback_id:03d}"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_dir / f"{stem}_color.png"), frame["color"])
        cv2.imwrite(str(self.output_dir / f"{stem}_depth_mm.png"), frame["depth"])
        cv2.imwrite(str(self.output_dir / f"{stem}_annotated.jpg"), frame["annotated"])
        record = {"feedback_id": feedback_id, "frame_id": frame["id"], "stamp": frame["stamp"],
                  "model": frame["model"],
                  "kind": payload["kind"], "click_upright": point,
                  "correct_label": str(payload.get("label", ""))[:80],
                  "note": str(payload.get("note", ""))[:200], "selected_detection": selected,
                  "all_detections": frame["detections"], "files_prefix": stem}
        with self.lock:
            with (self.output_dir / "feedback.jsonl").open("a", encoding="utf-8") as out:
                out.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.pinned.pop(frame["id"], None)
        return {"feedback_id": feedback_id, "detected_class": selected["class"] if selected else None}

    def request_switch(self, model_name):
        if model_name not in self.models:
            raise ValueError("未知或不可用的模型")
        with self.lock:
            self.requested_model = model_name
            self.switching = model_name != self.active_model


def make_handler(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, code, body, mime):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                self.respond(200, PAGE.encode(), "text/html; charset=utf-8")
            elif url.path == "/state":
                frame = state.get()
                body = {"id": frame["id"] if frame else None,
                        "stamp": frame["stamp"] if frame else None,
                        "detections": len(frame["detections"]) if frame else 0,
                        "error": state.error, "models": state.models,
                        "active_model": state.active_model,
                        "frame_model": frame["model"] if frame else None,
                        "requested_model": state.requested_model,
                        "switching": state.switching,
                        "inference_ms": state.inference_ms}
                self.respond(200, json.dumps(body).encode(), "application/json")
            elif url.path == "/frame.jpg":
                frame = state.get(parse_qs(url.query).get("id", [None])[0])
                if frame is None:
                    self.respond(404, b"frame expired", "text/plain")
                else:
                    self.respond(200, frame["jpeg"], "image/jpeg")
            else:
                self.respond(404, b"not found", "text/plain")

        def do_POST(self):
            if self.path not in {"/feedback", "/pin", "/switch"}:
                self.respond(404, b"{}", "application/json")
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 4096:
                    raise ValueError("反馈过大或为空")
                payload = json.loads(self.rfile.read(size))
                if self.path == "/switch":
                    state.request_switch(payload["model"])
                    result = {"requested": payload["model"]}
                elif self.path == "/pin":
                    state.pin(payload["frame_id"])
                    result = {"pinned": payload["frame_id"]}
                else:
                    result = state.save_feedback(payload)
                self.respond(200, json.dumps(result, ensure_ascii=False).encode(), "application/json")
            except (ValueError, KeyError, TypeError) as error:
                self.respond(400, json.dumps({"error": str(error)}, ensure_ascii=False).encode(), "application/json")
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--compare-seg", type=Path)
    parser.add_argument("--compare-box", type=Path)
    parser.add_argument("--initial-model", default="yolo11s-seg")
    parser.add_argument("--classes", nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path, default=Path.home() / "semantic_experiments" / f"live_review_{datetime.now():%Y%m%d_%H%M%S}")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--observation-interval", type=float, default=0.5,
                        help="Seconds between passive RGB-D observation records")
    parser.add_argument("--allow-raw-depth", action="store_true",
                        help="Register production 640x400 raw depth into color pixels in this process only")
    args = parser.parse_args()

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformException, TransformListener
    from rclpy.duration import Duration
    from rclpy.time import Time
    from ultralytics import YOLO, YOLOE
    import torch

    specs = {"yoloe-text": (args.model, "YOLOE-11s 开放类别+轮廓", YOLOE)}
    if args.compare_seg and args.compare_seg.is_file():
        specs["yolo11s-seg"] = (args.compare_seg, "YOLO11s 固定80类+轮廓", YOLO)
    if args.compare_box and args.compare_box.is_file():
        specs["yolo11s-box"] = (args.compare_box, "YOLO11s 固定80类+框", YOLO)
    if args.initial_model not in specs:
        parser.error(f"Initial model {args.initial_model!r} is not available")
    state = ReviewState(args.output_dir, {key: spec[1] for key, spec in specs.items()}, args.initial_model)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"REVIEW_URL=http://127.0.0.1:{args.port} OUTPUT={args.output_dir}", flush=True)
    model = None
    active_name = None
    rclpy.init()
    node = rclpy.create_node("semantic_live_review")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    bridge = CvBridge()
    colors, depths, infos, depth_infos = (deque(maxlen=6), deque(maxlen=6),
                                         deque(maxlen=3), deque(maxlen=3))
    node.create_subscription(Image, "/camera/color/image_raw", colors.append, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_raw", depths.append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/camera/color/camera_info", infos.append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info", depth_infos.append, qos_profile_sensor_data)
    last_stamp, last_inference = None, 0.0
    last_observation = 0.0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    observation_file = (args.output_dir / "observations.jsonl").open("a", encoding="utf-8")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if state.requested_model != active_name:
                desired = state.requested_model
                previous_name = active_name
                state.switching = True
                state.error = ""
                active_name = None
                result = None
                model = None
                gc.collect()
                torch.cuda.empty_cache()
                try:
                    path, _, model_type = specs[desired]
                    model = model_type(str(path))
                    if desired == "yoloe-text":
                        model.set_classes(args.classes)
                    active_name = desired
                    state.active_model = desired
                    last_stamp = None
                except Exception as error:
                    state.error = f"模型加载失败：{error}"
                    state.switching = False
                    state.active_model = None
                    state.requested_model = previous_name or desired
                    time.sleep(2)
                    continue
            if not (colors and depths and infos):
                continue
            color_msg = colors[-1]
            stamp = color_msg.header.stamp
            frame_id = f"{stamp.sec}_{stamp.nanosec:09d}"
            if frame_id == last_stamp or time.monotonic() - last_inference < args.interval:
                continue
            last_stamp, last_inference = frame_id, time.monotonic()
            processing_start = time.monotonic()
            depth_msg = min(depths, key=lambda d: abs((d.header.stamp.sec-stamp.sec) + (d.header.stamp.nanosec-stamp.nanosec)*1e-9))
            dt = abs((depth_msg.header.stamp.sec-stamp.sec) + (depth_msg.header.stamp.nanosec-stamp.nanosec)*1e-9)
            if dt > 0.033:
                state.error = "彩色/深度时间差超过 33 ms"
                continue
            color = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            info = infos[-1]
            if depth.dtype != np.uint16 or info.header.frame_id != color_msg.header.frame_id:
                state.error = "深度编码或彩色相机标定无效"
                continue
            registration = "sdk_aligned"
            registration_ms = 0.0
            if depth_msg.header.frame_id != color_msg.header.frame_id or depth.shape != color.shape[:2]:
                if not args.allow_raw_depth or not depth_infos:
                    state.error = "原始深度未配准；旁路需要 --allow-raw-depth"
                    continue
                depth_info = depth_infos[-1]
                if depth_info.header.frame_id != depth_msg.header.frame_id:
                    state.error = "深度相机标定坐标系不匹配"
                    continue
                try:
                    depth_to_color = tf_buffer.lookup_transform(
                        color_msg.header.frame_id, depth_msg.header.frame_id,
                        Time.from_msg(depth_msg.header.stamp), timeout=Duration(seconds=0.0))
                    registration_start = time.monotonic()
                    depth = register_depth(depth, depth_info, info, depth_to_color.transform)
                    registration_ms = round((time.monotonic() - registration_start) * 1000, 1)
                    registration = "software_raw_depth"
                except (TransformException, ValueError) as error:
                    state.error = f"旁路深度配准失败：{error}"[:180]
                    continue
            upright = cv2.rotate(color, cv2.ROTATE_180)
            annotated = upright.copy()
            infer_start = time.monotonic()
            result = model.predict(upright, imgsz=640, device=0, retina_masks=True, conf=0.25, verbose=False)[0]
            state.inference_ms = round((time.monotonic() - infer_start) * 1000, 1)
            postprocess_start = time.monotonic()
            intrinsics = (info.k[0], info.k[4], info.k[2], info.k[5])
            detections = []
            for index, box in enumerate(result.boxes):
                xyxy = [float(v) for v in box.xyxy[0]]
                raw = raw_box(xyxy, depth.shape[1], depth.shape[0])
                mask = None
                if result.masks is not None and index < len(result.masks.data):
                    upright_mask = result.masks.data[index].cpu().numpy().astype(np.uint8)
                    if upright_mask.shape != depth.shape:
                        upright_mask = cv2.resize(upright_mask, (640, 480), interpolation=cv2.INTER_NEAREST)
                    mask = cv2.rotate(upright_mask, cv2.ROTATE_180)
                    contours, _ = cv2.findContours(upright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)
                location = depth_location(depth, raw, intrinsics, raw_mask=mask)
                label = result.names[int(box.cls)]
                conf = float(box.conf)
                x1, y1, x2, y2 = map(int, xyxy)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 0, 255), 2)
                distance = f" {location['xyz_camera_optical_m'][2]:.2f}m" if location else " depth?"
                cv2.putText(annotated, f"#{index+1} {label} {conf:.2f}{distance}",
                            (max(0, x1), max(16, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
                cv2.putText(annotated, f"#{index+1} {label} {conf:.2f}{distance}",
                            (max(0, x1), max(16, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                detections.append({"id": index+1, "class": label, "confidence": conf,
                                   "box": [round(v, 1) for v in xyxy], "surface": location})
            postprocess_ms = round((time.monotonic() - postprocess_start) * 1000, 1)
            jpeg_start = time.monotonic()
            success, encoded = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            jpeg_ms = round((time.monotonic() - jpeg_start) * 1000, 1)
            if success:
                state.add({"id": frame_id, "stamp": frame_id, "color": color.copy(),
                           "depth": depth.copy(), "annotated": annotated,
                           "jpeg": encoded.tobytes(), "detections": detections,
                           "model": active_name})
            if time.monotonic() - last_observation >= args.observation_interval:
                last_observation = time.monotonic()
                transforms, tf_status = {}, {}
                for target in ("base_footprint", "map"):
                    try:
                        transforms[target] = tf_buffer.lookup_transform(
                            target, color_msg.header.frame_id, Time.from_msg(stamp),
                            timeout=Duration(seconds=0.0)).transform
                        tf_status[target] = "ok"
                    except TransformException as error:
                        tf_status[target] = str(error)[:160]
                observations = []
                for detection in detections:
                    # Keep low-confidence boxes in the review UI, but never export
                    # them as semantic-map candidates. Exactly 0.70 is rejected.
                    if detection["confidence"] <= 0.70:
                        continue
                    entry = {"class": detection["class"],
                             "confidence": detection["confidence"],
                             "surface": detection["surface"]}
                    if detection["surface"]:
                        xyz = detection["surface"]["xyz_camera_optical_m"]
                        for target, transform in transforms.items():
                            entry[f"xyz_{target}_m"] = transform_point(xyz, transform)
                    observations.append(entry)
                observation_file.write(json.dumps({
                    "stamp": [stamp.sec, stamp.nanosec], "camera_frame": color_msg.header.frame_id,
                    "model": active_name, "rgb_depth_dt_ms": round(dt * 1000, 2),
                    "registration": registration, "registration_ms": registration_ms,
                    "inference_ms": state.inference_ms, "postprocess_ms": postprocess_ms,
                    "jpeg_ms": jpeg_ms,
                    "processing_ms": round((time.monotonic() - processing_start) * 1000, 1),
                    "registered_valid_percent": round(100 * np.count_nonzero(depth) / depth.size, 1),
                    "tf_status": tf_status, "observations": observations,
                    "persistent_map_write": False}, ensure_ascii=False) + "\n")
                observation_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        observation_file.close()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

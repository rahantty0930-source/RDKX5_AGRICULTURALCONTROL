#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apple_lock_target_ros.py

功能：
1. 订阅 RDK X5 StereoNet 输出的矫正图像和深度图。
2. 调用 RDK X5 BPU 量化后的 YOLOv8 苹果检测模型。
3. 识别 red_apple / green_apple。
4. 根据检测框中心区域深度和相机内参计算 camera_xyz_m。
5. 对目标进行多帧稳定性判断，稳定后锁定并发布 /apple_target_json。
6. 发布带检测框和坐标文字的 /apple_detect_raw，供 hobot_codec_republish 网页推流。

说明：
- 该脚本不包含模型文件，请自行将 .bin 模型放到 MODEL_PATH。
- 默认模型输出按 YOLOv8 形状 (1, 6, 8400, 1) 或类似格式解析。
"""

import argparse
import ast
import json
import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception:
    import hobot_dnn.pyeasy_dnn as dnn


@dataclass
class Detection:
    box: list
    score: float
    cls_id: int
    name: str
    xyz: list | None = None
    center_uv: list | None = None


CLASS_NAMES = ["green_apple", "red_apple"]


def image_msg_to_bgr(msg: Image):
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if enc == "bgr8":
        return data.reshape(h, w, 3).copy()
    if enc == "rgb8":
        rgb = data.reshape(h, w, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if enc == "nv12":
        nv12 = data.reshape(h * 3 // 2, w)
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    if enc == "mono8":
        gray = data.reshape(h, w)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def depth_msg_to_array(msg: Image):
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()

    if enc in ["mono16", "16uc1"]:
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).copy()
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
    return None


def bgr_to_nv12(bgr):
    h, w = bgr.shape[:2]
    i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y_size = h * w
    uv_size = y_size // 4

    y = i420[:y_size].reshape(h, w)
    u = i420[y_size:y_size + uv_size].reshape(h // 2, w // 2)
    v = i420[y_size + uv_size:].reshape(h // 2, w // 2)

    uv = np.zeros((h // 2, w), dtype=np.uint8)
    uv[:, 0::2] = u
    uv[:, 1::2] = v
    return np.ascontiguousarray(np.vstack((y, uv)))


def iou_xyxy(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return inter / max(1e-6, area_a + area_b - inter)


def nms(dets, iou_thres):
    dets = sorted(dets, key=lambda d: d.score, reverse=True)
    keep = []
    while dets:
        best = dets.pop(0)
        keep.append(best)
        dets = [d for d in dets if iou_xyxy(best.box, d.box) < iou_thres]
    return keep


class AppleLockTargetNode(Node):
    def __init__(self, args):
        super().__init__("apple_lock_target_ros")
        self.args = args
        self.model = dnn.load(args.model)[0]

        self.latest_img = None
        self.latest_depth = None
        self.latest_header = None

        self.fx = 257.85458374
        self.fy = 257.85458374
        self.cx = 314.43139648
        self.cy = 159.97924805

        self.target_history = deque(maxlen=args.lock_frames)
        self.locked = False
        self.locked_info = None
        self.last_print_time = 0.0
        self.busy = False

        self.pub_img = self.create_publisher(Image, args.pub_image_topic, 10)
        self.pub_json = self.create_publisher(String, args.pub_json_topic, 10)

        self.create_subscription(Image, args.image_topic, self.cb_img, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.cb_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.cb_info, 10)

        self.timer = self.create_timer(args.period, self.process)

        self.get_logger().info("苹果目标锁定识别节点已启动")
        self.get_logger().info(f"模型: {args.model}")
        self.get_logger().info(f"图像输入: {args.image_topic}")
        self.get_logger().info(f"深度输入: {args.depth_topic}")
        self.get_logger().info(f"打标图输出: {args.pub_image_topic}")
        self.get_logger().info(f"JSON 输出: {args.pub_json_topic}")

    def cb_img(self, msg):
        img = image_msg_to_bgr(msg)
        if img is not None:
            self.latest_img = img
            self.latest_header = msg.header

    def cb_depth(self, msg):
        depth = depth_msg_to_array(msg)
        if depth is not None:
            self.latest_depth = depth

    def cb_info(self, msg):
        if len(msg.k) >= 9:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def run_model(self, bgr):
        resized = cv2.resize(bgr, (self.args.model_w, self.args.model_h))
        nv12 = bgr_to_nv12(resized)

        try:
            outputs = self.model.forward(nv12)
        except Exception:
            outputs = self.model.forward([nv12])

        pred = np.array(outputs[0].buffer).squeeze()
        if pred.ndim == 2 and pred.shape[0] == 6:
            pred = pred.T
        elif pred.ndim == 2 and pred.shape[1] == 6:
            pass
        else:
            pred = pred.reshape(-1, 6)

        h0, w0 = bgr.shape[:2]
        sx = w0 / self.args.model_w
        sy = h0 / self.args.model_h
        dets = []

        for p in pred:
            cx, cy, bw, bh = map(float, p[:4])
            scores = p[4:]
            cls_id = int(np.argmax(scores))
            score = float(scores[cls_id])
            if score < self.args.conf_thres:
                continue

            x1 = int(max(0, min(w0 - 1, (cx - bw / 2) * sx)))
            y1 = int(max(0, min(h0 - 1, (cy - bh / 2) * sy)))
            x2 = int(max(0, min(w0 - 1, (cx + bw / 2) * sx)))
            y2 = int(max(0, min(h0 - 1, (cy + bh / 2) * sy)))
            if x2 <= x1 or y2 <= y1:
                continue

            name = CLASS_NAMES[cls_id] if cls_id < len(CLASS_NAMES) else str(cls_id)
            dets.append(Detection([x1, y1, x2, y2], score, cls_id, name))

        return nms(dets, self.args.nms_thres)

    def get_xyz(self, box, depth):
        x1, y1, x2, y2 = box
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        h, w = depth.shape[:2]
        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        r = self.args.depth_roi
        roi = depth[max(0, v-r):min(h, v+r+1), max(0, u-r):min(w, u+r+1)]

        if roi.dtype == np.uint16:
            valid = roi[(roi > 50) & (roi < 5000)]
            if valid.size == 0:
                return None
            z = float(np.median(valid)) / 1000.0
        else:
            valid = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 5.0)]
            if valid.size == 0:
                return None
            z = float(np.median(valid))

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return [x, y, z], [u, v]

    def update_lock(self, best: Detection):
        if self.locked:
            return
        if best is None or best.xyz is None:
            self.target_history.clear()
            return
        if best.name != self.args.target_class:
            self.target_history.clear()
            return

        self.target_history.append(best)
        if len(self.target_history) < self.args.lock_frames:
            return

        names = [d.name for d in self.target_history]
        if any(n != self.args.target_class for n in names):
            self.target_history.clear()
            return

        xyzs = np.array([d.xyz for d in self.target_history], dtype=np.float32)
        std = xyzs.std(axis=0)
        mean = xyzs.mean(axis=0)
        score = float(np.mean([d.score for d in self.target_history]))

        stable = (
            std[0] <= self.args.std_x and
            std[1] <= self.args.std_y and
            std[2] <= self.args.std_z
        )

        if stable:
            self.locked = True
            self.locked_info = {
                "ready_for_arm": True,
                "locked": True,
                "mode": "multi_frame_target_lock",
                "name": self.args.target_class,
                "score": round(score, 3),
                "camera_xyz_m": [round(float(v), 3) for v in mean.tolist()],
                "xyz_std_m": [round(float(v), 3) for v in std.tolist()],
                "note": "target_locked_do_not_move_camera_or_apple"
            }
            self.get_logger().warn(f"目标已锁定: {self.locked_info}")

    def publish_json(self, best: Detection | None):
        if self.locked and self.locked_info:
            data = self.locked_info
        elif best and best.xyz:
            data = {
                "ready_for_arm": False,
                "locked": False,
                "mode": "tracking_before_lock",
                "name": best.name,
                "score": round(best.score, 3),
                "camera_xyz_m": [round(float(v), 3) for v in best.xyz],
                "note": "target_not_locked_yet"
            }
        else:
            data = {
                "ready_for_arm": False,
                "locked": False,
                "mode": "searching",
                "name": "none",
                "note": "no_valid_apple_detected"
            }
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_json.publish(msg)

    def publish_image(self, img):
        msg = Image()
        if self.latest_header is not None:
            msg.header = self.latest_header
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def draw(self, img, dets):
        for det in dets:
            x1, y1, x2, y2 = det.box
            color = (0, 0, 255) if det.name == "red_apple" else (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            if det.center_uv:
                cv2.circle(img, tuple(det.center_uv), 4, color, -1)
            cv2.putText(img, f"{det.name} {det.score:.2f}", (x1, max(20, y1 - 30)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            if det.xyz:
                x, y, z = det.xyz
                cv2.putText(img, f"X={x:.2f} Y={y:.2f} Z={z:.2f}m", (x1, max(45, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

        if self.locked and self.locked_info:
            x, y, z = self.locked_info["camera_xyz_m"]
            txt = f"LOCKED {self.locked_info['name']} X={x:.2f} Y={y:.2f} Z={z:.2f}m"
            cv2.putText(img, txt, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            cv2.putText(img, "SEARCHING / STABILIZING", (20, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return img

    def process(self):
        if self.busy:
            return
        if self.latest_img is None or self.latest_depth is None:
            return
        self.busy = True
        try:
            img = self.latest_img.copy()
            depth = self.latest_depth.copy()
            dets = self.run_model(img)

            valid_dets = []
            for det in dets:
                xyz_uv = self.get_xyz(det.box, depth)
                if xyz_uv is not None:
                    det.xyz, det.center_uv = xyz_uv
                valid_dets.append(det)

            best = None
            candidates = [d for d in valid_dets if d.xyz is not None]
            if candidates:
                best = max(candidates, key=lambda d: d.score)

            self.update_lock(best)
            self.publish_json(best)
            out = self.draw(img, valid_dets)
            self.publish_image(out)

            now = time.time()
            if now - self.last_print_time > 0.5:
                self.last_print_time = now
                if self.locked and self.locked_info:
                    print(f"\rLOCKED {self.locked_info}", end="", flush=True)
                elif best and best.xyz:
                    print(f"\rTRACK {best.name} score={best.score:.2f} xyz={best.xyz}", end="", flush=True)
                else:
                    print("\rSEARCHING...", end="", flush=True)
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.busy = False


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model", default="/home/sunrise/apple_project/best_bayese_640x640_nv12.bin")
    p.add_argument("--image-topic", default="/StereoNetNode/rectified_image")
    p.add_argument("--depth-topic", default="/StereoNetNode/stereonet_depth")
    p.add_argument("--camera-info-topic", default="/StereoNetNode/camera_info")
    p.add_argument("--pub-image-topic", default="/apple_detect_raw")
    p.add_argument("--pub-json-topic", default="/apple_target_json")
    p.add_argument("--model-w", type=int, default=640)
    p.add_argument("--model-h", type=int, default=640)
    p.add_argument("--conf-thres", type=float, default=0.35)
    p.add_argument("--nms-thres", type=float, default=0.45)
    p.add_argument("--period", type=float, default=0.08)
    p.add_argument("--depth-roi", type=int, default=8)
    p.add_argument("--target-class", default="red_apple")
    p.add_argument("--lock-frames", type=int, default=8)
    p.add_argument("--std-x", type=float, default=0.08)
    p.add_argument("--std-y", type=float, default=0.08)
    p.add_argument("--std-z", type=float, default=0.10)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AppleLockTargetNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

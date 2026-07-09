#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apple_live_distance_ros.py

不锁定目标，实时检测苹果并显示距离变化。
适合演示将苹果拉近/拉远时 Z 距离实时变化。
输出：
- /apple_detect_raw: 带框图像，bgr8
- /apple_live_json: 当前最佳目标 JSON
"""

import argparse
import json
import time
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

CLASS_NAMES = ["green_apple", "red_apple"]

@dataclass
class Detection:
    box: list
    score: float
    cls_id: int
    name: str
    xyz: list | None = None
    center_uv: list | None = None


def image_msg_to_bgr(msg):
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "bgr8":
        return data.reshape(h, w, 3).copy()
    if enc == "rgb8":
        return cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc == "nv12":
        nv12 = data.reshape(h * 3 // 2, w)
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    return None


def depth_msg_to_array(msg):
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


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2-x1) * max(0, y2-y1)
    area_a = max(0, a[2]-a[0]) * max(0, a[3]-a[1])
    area_b = max(0, b[2]-b[0]) * max(0, b[3]-b[1])
    return inter / max(1e-6, area_a + area_b - inter)


def nms(dets, thres):
    dets = sorted(dets, key=lambda d: d.score, reverse=True)
    keep = []
    while dets:
        best = dets.pop(0)
        keep.append(best)
        dets = [d for d in dets if iou(best.box, d.box) < thres]
    return keep


class AppleLiveDistance(Node):
    def __init__(self, args):
        super().__init__("apple_live_distance_ros")
        self.args = args
        self.model = dnn.load(args.model)[0]
        self.latest_img = None
        self.latest_depth = None
        self.latest_header = None
        self.fx = 257.85458374
        self.fy = 257.85458374
        self.cx = 314.43139648
        self.cy = 159.97924805
        self.busy = False
        self.last_print = 0

        self.pub_img = self.create_publisher(Image, args.pub_image_topic, 10)
        self.pub_json = self.create_publisher(String, args.pub_json_topic, 10)
        self.create_subscription(Image, args.image_topic, self.cb_img, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.cb_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.cb_info, 10)
        self.timer = self.create_timer(args.period, self.process)

        self.get_logger().info("实时距离演示节点已启动：NO LOCK")
        self.get_logger().info(f"输出打标图: {args.pub_image_topic}")

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
        sx, sy = w0 / self.args.model_w, h0 / self.args.model_h
        dets = []
        for p in pred:
            cx, cy, bw, bh = map(float, p[:4])
            scores = p[4:]
            cls_id = int(np.argmax(scores))
            score = float(scores[cls_id])
            if score < self.args.conf_thres:
                continue
            x1 = int(max(0, min(w0-1, (cx - bw/2) * sx)))
            y1 = int(max(0, min(h0-1, (cy - bh/2) * sy)))
            x2 = int(max(0, min(w0-1, (cx + bw/2) * sx)))
            y2 = int(max(0, min(h0-1, (cy + bh/2) * sy)))
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
        u = max(0, min(w-1, u))
        v = max(0, min(h-1, v))
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

    def process(self):
        if self.busy or self.latest_img is None or self.latest_depth is None:
            return
        self.busy = True
        try:
            img = self.latest_img.copy()
            depth = self.latest_depth.copy()
            dets = self.run_model(img)
            best = None
            for det in dets:
                xyz_uv = self.get_xyz(det.box, depth)
                if xyz_uv is None:
                    continue
                det.xyz, det.center_uv = xyz_uv
                x, y, z = det.xyz
                u, v = det.center_uv
                color = (0, 0, 255) if det.name == "red_apple" else (0, 255, 0)
                x1, y1, x2, y2 = det.box
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.circle(img, (u, v), 4, color, -1)
                cv2.putText(img, f"{det.name} {det.score:.2f}", (x1, max(20, y1-30)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                cv2.putText(img, f"X={x:.2f} Y={y:.2f} Z={z:.2f}m", (x1, max(45, y1-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
                if best is None or det.score > best.score:
                    best = det

            cv2.putText(img, "LIVE DISTANCE - NO LOCK", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            if best and best.xyz:
                x, y, z = best.xyz
                cv2.putText(img, f"{best.name} Z={z:.3f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0,255,255), 2)
                out = {"mode": "live_no_lock", "name": best.name, "score": round(best.score, 3), "camera_xyz_m": [round(x,3), round(y,3), round(z,3)]}
                msg = String()
                msg.data = json.dumps(out, ensure_ascii=False)
                self.pub_json.publish(msg)
                now = time.time()
                if now - self.last_print > 0.3:
                    self.last_print = now
                    print(f"\rLIVE {best.name} X={x:.3f} Y={y:.3f} Z={z:.3f}m     ", end="", flush=True)
            else:
                cv2.putText(img, "No apple detected", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0,0,255), 2)
            self.publish_image(img)
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
    p.add_argument("--pub-json-topic", default="/apple_live_json")
    p.add_argument("--model-w", type=int, default=640)
    p.add_argument("--model-h", type=int, default=640)
    p.add_argument("--conf-thres", type=float, default=0.35)
    p.add_argument("--nms-thres", type=float, default=0.45)
    p.add_argument("--period", type=float, default=0.08)
    p.add_argument("--depth-roi", type=int, default=8)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = AppleLiveDistance(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n已退出实时距离演示")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

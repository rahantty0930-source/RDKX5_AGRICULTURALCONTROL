#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_demo_panel.py

订阅 /apple_target_json，在终端实时显示：
- 识别类别
- 成熟度判断
- 置信度
- X/Y/Z
- 锁定状态
- 采摘建议
不保存任何文件，适合录制演示视频。
"""

import argparse
import ast
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class VisionDemoPanel(Node):
    def __init__(self, args):
        super().__init__("vision_demo_panel")
        self.args = args
        self.create_subscription(String, args.topic, self.cb, qos_profile_sensor_data)
        print(f"等待 {args.topic} ...")

    @staticmethod
    def parse(text):
        try:
            return json.loads(text)
        except Exception:
            try:
                return ast.literal_eval(text)
            except Exception:
                return {}

    def cb(self, msg):
        data = self.parse(msg.data)
        name = data.get("name", "unknown")
        score = float(data.get("score", 0.0))
        locked = bool(data.get("locked", False))
        ready = bool(data.get("ready_for_arm", False))
        xyz = data.get("camera_xyz_m", [0, 0, 0])
        if not isinstance(xyz, list) or len(xyz) < 3:
            xyz = [0, 0, 0]
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        dist = math.sqrt(x*x + y*y + z*z)

        if name == "red_apple":
            maturity = "成熟苹果"
        elif name == "green_apple":
            maturity = "未成熟苹果"
        else:
            maturity = "未知目标"

        pickable = name == "red_apple" and locked and ready
        if pickable:
            action = "建议采摘"
        elif name == "green_apple":
            action = "不采摘，继续观察"
        elif not locked:
            action = "等待目标稳定锁定"
        else:
            action = "暂不采摘"

        os.system("clear")
        print("==========================================")
        print("      RDK X5 苹果视觉识别与三维定位演示")
        print("==========================================")
        print(f"识别目标：{name}")
        print(f"成熟判断：{maturity}")
        print(f"置信度：  {score:.3f}")
        print("")
        print(f"X 坐标：  {x:.3f} m")
        print(f"Y 坐标：  {y:.3f} m")
        print(f"Z 距离：  {z:.3f} m")
        print(f"空间距离：{dist:.3f} m")
        print("")
        print(f"目标锁定：{locked}")
        print(f"视觉就绪：{ready}")
        print(f"采摘判断：{pickable}")
        print("")
        print(f"系统决策：{action}")
        print("")
        print("说明：当前为视觉前端演示，可通过串口扩展到 STM32 执行机构")
        print("==========================================")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/apple_target_json")
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = VisionDemoPanel(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

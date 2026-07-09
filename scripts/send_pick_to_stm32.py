#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_pick_to_stm32.py

订阅 /apple_target_json，当识别结果满足：
- name == red_apple
- locked == true
- ready_for_arm == true

则通过串口向 STM32 发送一次：
PICK,x,y,z\n
STM32 端只需要判断字符串是否以 PICK 开头，即可执行预设动作。
"""

import argparse
import ast
import json
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class PickSignalSender(Node):
    def __init__(self, args):
        super().__init__("send_pick_to_stm32")
        self.args = args
        self.sent = False
        self.last_send_time = 0.0

        self.ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
        self.create_subscription(String, args.topic, self.cb, qos_profile_sensor_data)

        self.get_logger().info("RDK -> STM32 PICK 信号发送节点已启动")
        self.get_logger().info(f"订阅话题: {args.topic}")
        self.get_logger().info(f"串口: {args.port}, 波特率: {args.baudrate}")

    @staticmethod
    def parse_msg(text):
        try:
            return json.loads(text)
        except Exception:
            try:
                return ast.literal_eval(text)
            except Exception:
                return {}

    def cb(self, msg):
        data = self.parse_msg(msg.data)
        name = data.get("name", "")
        locked = bool(data.get("locked", False))
        ready = bool(data.get("ready_for_arm", False))
        xyz = data.get("camera_xyz_m", [0, 0, 0])

        if name != self.args.target_class:
            return
        if not locked or not ready:
            return
        if not isinstance(xyz, list) or len(xyz) < 3:
            return
        if self.sent and not self.args.repeat:
            return
        if self.args.repeat and time.time() - self.last_send_time < self.args.repeat_interval:
            return

        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        cmd = f"PICK,{x:.3f},{y:.3f},{z:.3f}\n"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()
        self.sent = True
        self.last_send_time = time.time()

        print("\n======================================")
        print("已向 STM32 发送采摘信号：")
        print(cmd.strip())
        print("======================================")

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/apple_target_json")
    p.add_argument("--port", default="/dev/ttyS1")
    p.add_argument("--baudrate", type=int, default=9600)
    p.add_argument("--target-class", default="red_apple")
    p.add_argument("--repeat", action="store_true", help="允许重复发送。默认只发送一次。")
    p.add_argument("--repeat-interval", type=float, default=5.0)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = PickSignalSender(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

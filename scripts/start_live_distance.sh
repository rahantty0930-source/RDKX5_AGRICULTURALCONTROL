#!/bin/bash
# 启动实时距离演示，不锁定目标
cd /home/sunrise/apple_project
source /opt/tros/humble/setup.bash
python3 apple_live_distance_ros.py

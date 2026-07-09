#!/bin/bash
# 启动苹果识别、三维定位和多帧锁定节点
cd /home/sunrise/apple_project
source /opt/tros/humble/setup.bash
python3 apple_lock_target_ros.py

#!/bin/bash
# 识别锁定红苹果后，向 STM32 发送 PICK,x,y,z
cd /home/sunrise/apple_project
source /opt/tros/humble/setup.bash
python3 send_pick_to_stm32.py --port /dev/ttyS1 --baudrate 9600

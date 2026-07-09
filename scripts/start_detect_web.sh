#!/bin/bash
# 将 /apple_detect_raw 转换为 /image_jpeg，用于网页推流显示识别打标画面
source /opt/tros/humble/setup.bash
pkill -f hobot_codec_republish

ros2 run hobot_codec hobot_codec_republish --ros-args \
  -p channel:=1 \
  -p in_mode:=ros \
  -p out_mode:=ros \
  -p sub_topic:=/apple_detect_raw \
  -p pub_topic:=/image_jpeg \
  -p in_format:=bgr8 \
  -p out_format:=jpeg \
  -p enc_qp:=10.0 \
  -p jpg_quality:=50.0 \
  -p input_framerate:=10 \
  -p output_framerate:=-1 \
  -p dump_output:=false

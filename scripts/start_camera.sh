#!/bin/bash
# 启动 GS130WI 双目相机 + StereoNet 深度节点
source /opt/tros/humble/setup.bash

ros2 launch hobot_stereonet stereonet_model_web_visual_v2.4_int8.launch.py \
  use_mipi_cam:=True \
  mipi_image_width:=640 \
  mipi_image_height:=352 \
  mipi_image_framerate:=30.0 \
  mipi_rotation:=90.0 \
  mipi_gdc_enable:=True \
  mipi_lpwm_enable:=True \
  mipi_channel:=0 \
  mipi_channel2:=2 \
  mipi_cal_rotation:=0.0 \
  stereo_image_topic:=/image_combine_raw \
  camera_info_topic:=/image_combine_raw/right/camera_info

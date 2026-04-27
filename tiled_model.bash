#!/bin/bash


source /opt/ros/jazzy/setup.bash
source ~/AGDeltaX/install/setup.bash

ros2 launch yolo_bringup yolo.launch.py \
  model:=/home/fresnostate/AGDeltaX/src/yolo_ros/tiled_weed_model/my_model.pt \
  model_type:=YOLO \
  threshold:=0.5 \
  input_image_topic:=/camera/camera/color/image_raw \
  image_reliability:=2 \
  input_depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  depth_image_reliability:=2 \
  input_depth_info_topic:=/camera/camera/aligned_depth_to_color/camera_info \
  depth_info_reliability:=2 \
  target_frame:=camera_color_optical_frame \
  use_3d:=False \
  use_debug:=True \
  device:=cpu






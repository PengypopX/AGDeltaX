#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/AGDeltaX/install/setup.bash

ros2 launch yolo_bringup yolo.launch.py \
model:=/home/fresnostate/AGDeltaX/src/yolo_ros/yolo_weeds_model_v1/my_model.pt \
model_type:=YOLO \
threshold:=0.5 \
input_image_topic:="/image_raw" \
use_3d:=False \
use_debug:=True \
device:=cpu


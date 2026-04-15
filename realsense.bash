# Source ROS + your workspace
source /opt/ros/jazzy/setup.bash
source ~/AGDeltaX/install/setup.bash


ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  json_file_path:=/home/fresnostate/AGDeltaX/default.json \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=2






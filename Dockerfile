# Use the official ROS 2 Jazzy Desktop image (Ubuntu 24.04 base)
FROM osrf/ros:jazzy-desktop

# Set environment variables so installation runs non-interactively
ENV DEBIAN_FRONTEND=noninteractive

# Set the workspace directory
WORKDIR /ros2_ws

# Install essential build tools, python pip, and RealSense ROS 2 binaries
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    python3-pip \
    ros-jazzy-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Create the src directory and clone the necessary repositories
# 1. AGDeltaX (Your main project)
# 2. yolo_ros (Required for yolo_bringup and yolo_msgs)
RUN mkdir -p src \
    && git clone https://github.com/PengypopX/AGDeltaX.git src/AGDeltaX \
    && git clone https://github.com/mgonzs13/yolo_ros.git src/yolo_ros

# Initialize and update rosdep, then install all repository-specific dependencies
RUN apt-get update \
    && rosdep init || true \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Hack to fix the AGDeltaX setup.py FileNotFoundError
# 1. Install pip requirements directly
# 2. Pre-create build directories and copy requirements.txt into them
RUN /bin/bash -c ' \
    for req in $(find src -name "requirements.txt"); do \
        pip3 install -r $req --break-system-packages; \
        pkg_name=$(basename $(dirname $req)); \
        mkdir -p build/$pkg_name; \
        cp $req build/$pkg_name/; \
    done'

# Install uv package manager required by yolo_ros
RUN pip3 install uv --break-system-packages

# Build the entire ROS 2 workspace (AGDeltaX + yolo_ros)
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Automatically source both the ROS 2 base environment and the local workspace environment in new bash sessions
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command to open an interactive bash shell
CMD ["bash"]

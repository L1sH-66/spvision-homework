#!/usr/bin/env bash
set -eo pipefail
. /etc/os-release
[[ "$VERSION_ID" == '22.04' ]] || { echo '需要 Ubuntu 22.04'; exit 1; }
[[ "$(uname -m)" == 'x86_64' ]] || { echo '原版仿真器仅提供 x86_64/Python 3.10 二进制'; exit 1; }
sudo apt update
sudo apt install -y build-essential cmake python3-colcon-common-extensions python3-rosdep \
  ros-humble-pluginlib ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs ros-humble-std-srvs ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 ros-humble-rclcpp-action ros-humble-rclpy \
  ros-humble-visualization-msgs ros-humble-rosbag2 ros-humble-action-msgs \
  qtbase5-dev libopencv-dev libyaml-cpp-dev python3-pygame python3-numpy \
  python3-pil python3-yaml python3-opencv fonts-noto-cjk

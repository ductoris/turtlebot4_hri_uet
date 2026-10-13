#!/usr/bin/env bash
# ==============================================================================
# Script cài đặt tất cả dependencies cần thiết cho TurtleBot 4 ROS 2 Humble
# ==============================================================================
set -e

echo "[INFO] Đang cập nhật apt repository..."
sudo apt-get update

echo "[INFO] Đang cài đặt các packages TurtleBot 4, Nav2 và SLAM..."
sudo apt-get install -y \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz \
  ros-humble-turtlebot4-description \
  ros-humble-turtlebot4-msgs \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard

echo "[INFO] Hoàn tất cài đặt dependencies!"
echo "[INFO] Kiểm tra gói vừa cài:"
ros2 pkg list | grep -E 'turtlebot4|slam_toolbox|nav2'

#!/usr/bin/env bash
# ==============================================================================
# Script điều khiển TurtleBot 4 qua bàn phím (teleop_twist_keyboard)
# ==============================================================================
echo "[INFO] Khởi động teleop keyboard cho TurtleBot 4 (topic /cmd_vel)..."
echo "Hướng dẫn phím:"
echo "  i : tiến"
echo "  , : lùi"
echo "  j : xoay trái"
echo "  l : xoay phải"
echo "  k : dừng lại"
echo "  q/z : tăng/giảm tốc độ tuyến tính"
echo "  w/x : tăng/giảm tốc độ góc"
echo "LƯU Ý KHI MAPPING: Đi chậm (0.15 - 0.2 m/s), không quay gắt!"
echo "--------------------------------------------------------"

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

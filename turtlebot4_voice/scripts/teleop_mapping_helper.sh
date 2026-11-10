#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# teleop_mapping_helper.sh:
# Hướng dẫn và script hỗ trợ quét bản đồ bằng SLAM Toolbox và lưu bản đồ bằng map_saver_cli.

echo "=========================================================="
echo " HƯỚNG DẪN QUÉT BẢN ĐỒ SLAM TOOLBOX CHO OFFICE SIMULATION"
echo "=========================================================="
echo ""
echo "Bước 1: Khởi chạy mô phỏng cùng SLAM Toolbox:"
echo "   ros2 launch turtlebot4_voice sim_bringup.launch.py slam:=true rviz:=true"
echo ""
echo "Bước 2: Trong terminal mới, điều khiển robot chạy khắp các phòng để quét LiDAR:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo "   (Sử dụng các phím i, j, k, l để tiến, rẽ trái, dừng, rẽ phải với tốc độ vừa phải ~0.2 m/s)"
echo ""
echo "Bước 3: Khi bản đồ đã rõ nét 4 phòng và sảnh, lưu bản đồ bằng map_saver_cli:"
echo "   ros2 run nav2_map_server map_saver_cli -f \$(ros2 pkg prefix turtlebot4_voice)/share/turtlebot4_voice/maps/office_map"
echo ""
echo "Bản đồ cũng có thể tự động tạo lại bất kỳ lúc nào khớp 100% với SDF bằng:"
echo "   python3 \$(ros2 pkg prefix turtlebot4_voice)/lib/turtlebot4_voice/scripts/generate_office_map.py"
echo "=========================================================="

#!/bin/bash
# scripts/kill_all_sim.sh: Dọn dẹp triệt để mọi tiến trình ROS 2, Gazebo, RViz và FastRTPS treo

echo "🛑 Đang dọn dẹp toàn bộ tiến trình mô phỏng và ROS 2..."
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "voice_nav_node" 2>/dev/null || true
pkill -9 -f "speech_node" 2>/dev/null || true
pkill -9 -f "voice_dashboard_gui" 2>/dev/null || true
pkill -9 -f "regions_marker_node" 2>/dev/null || true
pkill -9 -f "initial_pose_publisher" 2>/dev/null || true
pkill -9 -f "dummy_hardware_services" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true

echo "🧹 Đang dọn dẹp các tệp shared memory FastRTPS trong /dev/shm..."
rm -f /dev/shm/fastrtps* /dev/shm/sem.*

echo "✅ Hệ thống đã được làm sạch hoàn toàn! Sẵn sàng khởi chạy lại."

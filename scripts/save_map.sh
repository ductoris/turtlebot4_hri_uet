#!/usr/bin/env bash
# ==============================================================================
# Script lưu bản đồ từ SLAM Toolbox qua nav2_map_server map_saver_cli
# ==============================================================================
set -e

# Xác định đường dẫn thư mục maps trong package turbot4_simulation
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"
MAP_DIR="$PKG_ROOT/maps"

mkdir -p "$MAP_DIR"
MAP_NAME="${1:-map}"
OUTPUT_PATH="$MAP_DIR/$MAP_NAME"

echo "[INFO] Đang lưu bản đồ vào: $OUTPUT_PATH.yaml và $OUTPUT_PATH.pgm..."
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_PATH" --ros-args -p use_sim_time:=true

if [ -f "$OUTPUT_PATH.yaml" ] && [ -f "$OUTPUT_PATH.pgm" ]; then
    echo "[SUCCESS] Bản đồ đã được lưu thành công!"
    echo "--- Nội dung file $OUTPUT_PATH.yaml ---"
    cat "$OUTPUT_PATH.yaml"
    echo "----------------------------------------"
else
    echo "[ERROR] Lưu bản đồ thất bại! Hãy đảm bảo SLAM đang chạy và topic /map có dữ liệu."
    exit 1
fi

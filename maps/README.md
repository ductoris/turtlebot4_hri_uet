# Thư mục lưu bản đồ (maps)

Thư mục này chứa file bản đồ sau khi hoàn thành Giai đoạn 2 (Mapping với SLAM Toolbox):
- `map.yaml`: Metadata bản đồ (resolution, origin, threshold, image file)
- `map.pgm`: Tệp ảnh dạng occupancy grid (đen = vật cản, trắng = vùng trống, xám = chưa quét)

Lệnh lưu bản đồ:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/workspaces/ur_gz/src/turbot4_simulation/maps/map --ros-args -p use_sim_time:=true
```
Hoặc dùng script:
```bash
./scripts/save_map.sh
```

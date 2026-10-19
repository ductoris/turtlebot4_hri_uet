# Lab 05 – TurtleBot 4: Localization, Autonomous Navigation & YOLO

ROS 2 Humble · Ubuntu 22.04 · Ignition Gazebo (Fortress) · Nav2 · YOLOv8n

Package `tb4_autonomy_yolo` cho robot TurtleBot 4 mô phỏng trong một **phòng khách mini 8 × 6 m**
(bàn, ghế, kệ, sofa, tủ, người, bình chữa cháy):

| Phần | Nội dung | Thành phần |
|------|----------|------------|
| 1 | Tạo bản đồ & thế giới mô phỏng | `worlds/home_room.sdf`, `maps/home_room.{pgm,yaml}` |
| 2 | Localization (AMCL) + Nav2 + RViz2 | `launch/sim.launch.py`, `launch/nav.launch.py` |
| 3 | Điều hướng tự động qua 3 waypoint | `nav_node.py` |
| 4 | Nhận diện vật thể YOLOv8n | `yolo_node.py` |

```
practice05_tb4/
├── README.md
├── docs/
│   └── BAO_CAO_DAN_Y.md          # dàn ý báo cáo + vấn đáp
└── tb4_autonomy_yolo/            # ROS 2 package (ament_python)
    ├── package.xml  setup.py  setup.cfg  resource/
    ├── launch/   sim.launch.py  nav.launch.py
    ├── worlds/   home_room.sdf   (world tên "warehouse")
    ├── maps/     home_room.pgm  home_room.yaml
    ├── scripts/  generate_world_and_map.py
    └── tb4_autonomy_yolo/  nav_node.py  yolo_node.py
```

---

## 1. Cài đặt

```bash
# ROS packages
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge \
  ros-humble-teleop-twist-keyboard python3-colcon-common-extensions

# YOLO (pip). cv_bridge của Humble biên dịch với NumPy 1.x => ghim numpy<2
pip3 install ultralytics "numpy<2"
# (tuỳ chọn) tải sẵn trọng số để lần chạy đầu không phải chờ mạng
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

> World dùng model từ **Gazebo Fuel** (người, ghế, bình chữa cháy) nên **lần chạy đầu cần Internet**
> để tải; sau đó được cache ở `~/.ignition/fuel`.

## 2. Tạo package (nếu làm từ đầu)

```bash
cd ~/workspaces/ur_gz/src/practice05_tb4
ros2 pkg create --build-type ament_python tb4_autonomy_yolo \
  --dependencies rclpy nav2_simple_commander sensor_msgs geometry_msgs cv_bridge
```

Sau đó:
- `package.xml`: dùng `<exec_depend>` cho các phụ thuộc (đã có sẵn trong repo này).
- `setup.py`: khai báo `entry_points` và cài `launch/`, `worlds/`, `maps/` vào `share/`:

```python
entry_points={'console_scripts': [
    'nav_node = tb4_autonomy_yolo.nav_node:main',
    'yolo_node = tb4_autonomy_yolo.yolo_node:main',
]},
```

## 3. Build

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select tb4_autonomy_yolo --symlink-install
source install/setup.bash
```

## 4. Chạy (mỗi lệnh một terminal, đều đã `source` workspace)

### Terminal 1 – Gazebo + TurtleBot 4
```bash
ros2 launch tb4_autonomy_yolo sim.launch.py            # robot spawn tại (0,0,yaw 0)
```
Đợi tới khi robot xuất hiện và `ros2 topic list` có `/scan`, `/oakd/rgb/preview/image_raw`.
Robot đang ở dock: nếu cần, `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"`.

### Terminal 2 – Localization (map_server + AMCL) + Nav2
```bash
ros2 launch tb4_autonomy_yolo nav.launch.py
# dùng bản đồ khác:  ... nav.launch.py map:=$HOME/map.yaml
```

### Terminal 3 – RViz2
```bash
ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true
```
Trong RViz: `Fixed Frame = map`; thêm display **Map** (`/map`), **LaserScan** (`/scan`),
**Path** (`/plan`), **Image** (`/yolo/image_annotated`).

### Terminal 4 – YOLO
```bash
ros2 run tb4_autonomy_yolo yolo_node --ros-args -p use_sim_time:=true
# đổi topic / ngưỡng:
ros2 run tb4_autonomy_yolo yolo_node --ros-args \
  -p image_topic:=/oakd/rgb/preview/image_raw -p conf_threshold:=0.3 -p show_window:=true
```

### Terminal 5 – Điều hướng tự động
```bash
ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true
```
`nav_node` tự đặt initial pose → chờ Nav2 active → đi Goal 1 → 2 → 3, in khoảng cách còn lại và kết quả.

## 5. Thêm vật thể vào Gazebo để YOLO nhận diện

**Cách 1 – cố định trong world:** sửa `FUEL_OBJECTS` trong
`tb4_autonomy_yolo/scripts/generate_world_and_map.py` rồi chạy lại
`python3 scripts/generate_world_and_map.py` (sinh lại cả `.sdf` lẫn bản đồ, luôn khớp nhau).

**Cách 2 – thêm trực tiếp khi sim đang chạy** (tên world là `warehouse`):
```bash
ign service -s /world/warehouse/create --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire hydrant", name: "hydrant_x", pose: {position: {x: 2.0, y: -2.0}}'
```
Thay URI bằng `.../models/Standing person`, `.../models/Chair`, `.../models/Bookshelf`...
Class COCO mà YOLOv8n nhận diện trong bài: `person`, `chair`, `couch`, `dining table`, `fire hydrant`.

> Lưu ý: tên world trong SDF **phải là `warehouse`**, vì bridge của `turtlebot4_ignition_bringup`
> dùng đường dẫn `/world/warehouse/...` (nếu đổi tên, camera/lidar sẽ không lên ROS).

## 6. Lấy toạ độ (x, y, yaw) cho waypoint

**Cách A – `/clicked_point`:** RViz2 → nút **Publish Point** (thanh công cụ) → click lên bản đồ:
```bash
ros2 topic echo /clicked_point --once       # lấy x, y (yaw tự chọn)
```
**Cách B – `/amcl_pose`:** lái robot tới vị trí mong muốn rồi
```bash
ros2 topic echo /amcl_pose --once --field pose.pose
```
Đổi quaternion → yaw:
```bash
python3 -c "import math;z,w=0.7071,0.7071;print(math.degrees(2*math.atan2(z,w)))"
```
**Đặt Initial Pose:**
- *Trong RViz2:* nút **2D Pose Estimate**, click + kéo theo hướng robot.
- *Bằng code:* `INITIAL_POSE` trong `nav_node.py` (dùng `navigator.setInitialPose`).
- *Bằng CLI:*
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

Sau đó sửa `GOAL_1/2/3` ở đầu [nav_node.py](tb4_autonomy_yolo/tb4_autonomy_yolo/nav_node.py).

## 7. (Tuỳ chọn) Tự tạo bản đồ bằng SLAM thay vì dùng map sinh sẵn

```bash
ros2 launch tb4_autonomy_yolo sim.launch.py                       # T1
ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true   # T2
ros2 launch turtlebot4_viz view_robot.launch.py use_sim_time:=true    # T3
ros2 run teleop_twist_keyboard teleop_twist_keyboard                # T4: lái quanh phòng
ros2 run nav2_map_server map_saver_cli -f ~/workspaces/ur_gz/src/practice05_tb4/tb4_autonomy_yolo/maps/my_map
```
Rồi chạy Nav2 với `map:=.../my_map.yaml` (tắt SLAM trước).

## 8. Kiểm tra nhanh / Debug

| Triệu chứng | Cách xử lý |
|---|---|
| `ros2 topic hz /oakd/rgb/preview/image_raw` không có dữ liệu | World phải tên `warehouse`; chờ robot spawn xong |
| `nav_node` đứng ở "chờ Nav2 active" | Kiểm tra Terminal 2 chạy, đã `use_sim_time:=true` |
| Robot đi lệch / AMCL lạc | Đặt lại Initial Pose trong RViz (2D Pose Estimate) |
| Goal FAILED | Goal rơi vào vật cản/inflation → chọn điểm trống hơn trên bản đồ |
| `cv_bridge` lỗi `_ARRAY_API not found` | `pip3 install "numpy<2"` |
| `operator torchvision::nms does not exist` | torch và torchvision lệch bản (CPU vs CUDA). Cài cùng một index: `pip3 install --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cpu` |
| `colcon` báo `setuptools<80` | `pip3 install "setuptools>=77.0.3,<80"` |
| Không có cửa sổ cv2 (SSH) | `-p show_window:=false`, xem ảnh qua RViz |
| YOLO không thấy vật | hạ `conf_threshold` (0.25), tiến gần vật hơn (ảnh preview chỉ 250×250) |

## 9. Tài liệu
- Báo cáo & vấn đáp: [docs/BAO_CAO_DAN_Y.md](docs/BAO_CAO_DAN_Y.md)

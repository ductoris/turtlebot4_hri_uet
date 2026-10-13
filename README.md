# TURTLEBOT 4 SLAM & NAVIGATION2 SIMULATION PACKAGE
## Gói phần mềm mô phỏng, quét bản đồ SLAM Toolbox và điều hướng tự hành Nav2 cho TurtleBot 4 (ROS 2 Humble)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition%20Fortress-orange)
![Robot](https://img.shields.io/badge/Robot-TurtleBot%204-red)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

---

## 📌 MỤC LỤC
1. [Yêu cầu hệ thống](#1-yêu-cầu-hệ-thống)
2. [Cài đặt & Build Workspace](#2-cài-đặt--build-workspace)
3. [Hướng dẫn chạy chi tiết từng giai đoạn](#3-hướng-dẫn-chạy-chi-tiết-từng-giai-đoạn)
   - [Giai đoạn 1: Khởi chạy Mô phỏng Gazebo](#giai-đoạn-1-khởi-chạy-mô-phỏng-gazebo)
   - [Giai đoạn 2: Quét bản đồ với SLAM Toolbox & Lưu bản đồ](#giai-đoạn-2-quét-bản-đồ-với-slam-toolbox--lưu-bản-đồ)
   - [Giai đoạn 3: Định vị vị trí với AMCL (Localization)](#giai-đoạn-3-định-vị-vị-trí-với-amcl-localization)
   - [Giai đoạn 4: Điều hướng tự hành với Nav2 (Navigation)](#giai-đoạn-4-điều-hướng-tự-hành-với-nav2-navigation)
4. [Lưu & Load bản đồ](#4-lưu--load-bản-đồ)
5. [Xử lý lỗi thường gặp (Troubleshooting)](#5-xử-lý-lỗi-thường-gặp-troubleshooting)
6. [Kịch bản quay Video Demo](#6-kịch-bản-quay-video-demo)
7. [Checklist nộp bài & Lệnh Git](#7-checklist-nộp-bài--lệnh-git)

---

## 1. YÊU CẦU HỆ THỐNG

- **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish) - Native hoặc WSL2 (đã kích hoạt WSLg).
- **ROS 2:** ROS 2 Humble Desktop.
- **Mô phỏng:** Ignition Gazebo 6.18+ (Fortress).
- **Phần cứng đề xuất:**
  - CPU: Tối thiểu 4 cores (khuyến nghị 8 cores trở lên).
  - RAM: Tối thiểu 8 GB (khuyến nghị 16 GB).
  - GPU: Card đồ họa rời (NVIDIA) để chạy mượt mà đồ họa Gazebo và RViz2.

---

## 2. CÀI ĐẶT & BUILD WORKSPACE

### Bước 2.1: Cài đặt các gói phụ thuộc (Dependencies)
Mở một Terminal mới và chạy:

```bash
# Cập nhật apt
sudo apt update

# Cài đặt toàn bộ bộ công cụ TurtleBot 4, Nav2 và SLAM
sudo apt install -y \
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
```

### Bước 2.2: Cấu hình biến môi trường
Thêm vào file `~/.bashrc` để tự động nạp mỗi khi mở terminal mới:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### Bước 2.3: Build package `turbot4_simulation`
Đi đến thư mục workspace chứa package và build:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select turbot4_simulation --symlink-install
source install/setup.bash
```

---

## 3. HƯỚNG DẪN CHẠY CHI TIẾT TỪNG GIAI ĐOẠN

> [!IMPORTANT]
> Hãy mở các terminal riêng biệt theo đúng hướng dẫn dưới đây để dễ dàng theo dõi log và quản lý các node.

---

### GIAI ĐOẠN 1: KHỞI CHẠY MÔ PHỎNG GAZEBO

#### 🖥️ Terminal 1 — Khởi chạy Ignition Gazebo thế giới Warehouse:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse
```
*Lưu ý:* Khi cửa sổ Ignition Gazebo hiện lên, kiểm tra góc dưới bên trái nút **Play** (hình tam giác) đã chạy.

#### 🖥️ Terminal 2 — Khởi chạy RViz2 trực quan hóa:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### 🖥️ Terminal 3 — Kiểm tra hệ thống cảm biến & Test vận hành:
```bash
source /opt/ros/humble/setup.bash

# 1. Kiểm tra danh sách topic
ros2 topic list

# 2. Kiểm tra tần số quét cảm biến LiDAR RPLIDAR (~10 Hz)
ros2 topic hz /scan

# 3. Đọc thử 1 gói tin vị trí Odometry
ros2 topic echo /odom --once

# 4. Gửi thử lệnh nhích robot lên phía trước 0.2 m/s
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

### GIAI ĐOẠN 2: QUÉT BẢN ĐỒ VỚI SLAM TOOLBOX & LƯU BẢN ĐỒ

#### 🖥️ Terminal 1 — Gazebo Simulator (giữ nguyên từ Giai đoạn 1):
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse

```

#### 🖥️ Terminal 2 — Khởi chạy SLAM Toolbox:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=false use_sim_time:=true
```

#### 🖥️ Terminal 3 — Mở RViz2 hiển thị bản đồ đang quét:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_viz view_robot.launch.py
```
*(Trong RViz2, đảm bảo display `Map` đã bật và lắng nghe topic `/map`)*

#### 🖥️ Terminal 4 — Điều khiển robot khám phá môi trường:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```
**Chiến lược lái xe tạo map chuẩn:**
- Lái robot với tốc độ chậm ($0.15 \sim 0.2\,\text{m/s}$).
- Tránh quay ngoắt $180^\circ$ tại chỗ; quay bo cua nhẹ nhàng.
- Đi một vòng khép kín quanh nhà kho, sau đó lái robot quay trở về đúng vị trí xuất phát để SLAM kích hoạt **Loop Closure** khớp các bờ tường.

#### 🖥️ Terminal 5 — Lưu bản đồ sau khi quét xong:
```bash
source /opt/ros/humble/setup.bash
cd ~/workspaces/ur_gz/src/turbot4_simulation/maps
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p use_sim_time:=true
```
*(Sau lệnh này, hai tệp `map.yaml` và `map.pgm` sẽ được tạo ra).*

---

### GIAI ĐOẠN 3: ĐỊNH VỊ VỊ TRÍ VỚI AMCL (LOCALIZATION)

Sau khi lưu map, tắt các terminal SLAM và teleop (`Ctrl + C`). Khởi động lại môi trường sạch:

#### 🖥️ Terminal 1 — Khởi chạy Gazebo Warehouse:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse
```

#### 🖥️ Terminal 2 — Khởi chạy AMCL Localization với bản đồ đã quét:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_navigation localization.launch.py map:=$HOME/workspaces/ur_gz/src/turbot4_simulation/maps/map.yaml
```

#### 🖥️ Terminal 3 — Mở RViz2:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### 📌 Thao tác trên giao diện RViz2:
1. Nhấp chuột vào nút **2D Pose Estimate** trên thanh công cụ trên cùng.
2. Click vào vị trí xuất phát của robot trên bản đồ (gốc tọa độ nhà kho), kéo mũi tên theo đúng hướng nhìn của robot trong Gazebo.
3. **Quan sát:** Đám mây hạt (particle cloud màu xanh/đỏ) sẽ co cụm lại, các chấm tia laser đỏ (`/scan`) sẽ trùng khít hoàn hảo với các vách tường màu đen trên bản đồ.

---

### GIAI ĐOẠN 4: ĐIỀU HƯỚNG TỰ HÀNH VỚI NAV2 (NAVIGATION)

#### 🖥️ Terminal 1 — Gazebo Warehouse (giữ nguyên):
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse
```

#### 🖥️ Terminal 2 — Khởi chạy trọn bộ Nav2 (AMCL Localization + Navigation Planners & Controllers):
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 launch turbot4_simulation navigation.launch.py
```
*(Lệnh này tự động kết hợp `map_server`, `amcl` định vị bản đồ và toàn bộ stack Nav2: `bt_navigator`, `planner_server`, `controller_server`, `recoveries_server`).*

#### 🖥️ Terminal 3 — Mở RViz2:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### 📌 Thao tác điều hướng trên RViz2:
1. Nếu chưa định vị, dùng **2D Pose Estimate** để khớp vị trí ban đầu.
2. Nhấp vào nút **Nav2 Goal** trên thanh công cụ RViz2.
3. Click vào một vị trí đích mong muốn trong nhà kho và kéo hướng về đích.
4. **Quan sát:**
   - Nav2 Global Planner vẽ đường đi màu xanh lá cây (Global Path).
   - Local Planner điều khiển các bánh xe sinh ra `/cmd_vel`.
   - Robot tự động di chuyển, tự né các kệ hàng và dừng lại chính xác tại điểm đích.
5. Lặp lại gửi ít nhất **3 vị trí đích khác nhau** ở các góc nhà kho để ghi hình demo.

---

## 4. LƯU & LOAD BẢN ĐỒ

### Lưu bản đồ:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/workspaces/ur_gz/src/turbot4_simulation/maps/map --ros-args -p use_sim_time:=true
```

### Kiểm tra file `map.yaml`:
```yaml
image: map.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## 5. XỬ LÝ LỖI THƯỜNG GẶP (TROUBLESHOOTING)

| Hiện tượng lỗi | Nguyên nhân | Cách khắc phục |
|---|---|---|
| `Lookup would require extrapolation into the future` hoặc TF time error | Chưa đồng bộ thời gian mô phỏng giữa các node | Thêm `--ros-args -p use_sim_time:=true` vào các lệnh hoặc kiểm tra node `/clock`. |
| Robot không di chuyển khi ấn `Nav2 Goal` | Điểm đích nằm trong vùng Inflation Layer (vùng cấm) hoặc AMCL chưa hội tụ | Chọn điểm đích ra giữa lối đi thông thoáng. Dùng **2D Pose Estimate** đặt lại vị trí thật chuẩn. |
| Bản đồ bị tường đôi (Double Wall), bị lệch vẹo | Robot quay quá nhanh khiến odometry bị trượt và scan-matcher không kịp ghép đám mây điểm | Giảm tốc độ quay khi lái teleop. Đi hoàn thành một vòng lặp kín về điểm gốc để kích hoạt Loop Closure. |
| Không nhìn thấy bản đồ trong RViz2 | Chưa bật display `Map` hoặc gõ sai tên topic | Trong RViz2: Add -> By Topic -> `/map` (OccupancyGrid). Đặt Fixed Frame là `map`. |

---

## 6. KỊCH BẢN QUAY VIDEO DEMO

### Chuẩn bị trước khi quay:
- Chia màn hình: Nửa trái là **Ignition Gazebo**, nửa phải là **RViz2**.
- Terminal ẩn phía dưới hoặc xếp một góc nhỏ để thấy lệnh chạy.

### Các phân cảnh (Scenes):
1. **Cảnh 1 (0:00 - 0:30) - Giới thiệu & Khởi động:**
   - Quay màn hình Gazebo Warehouse và RViz2 hiển thị RobotModel, trục TF, tia LiDAR xoay 360 độ.
2. **Cảnh 2 (0:30 - 2:00) - Quá trình Quét bản đồ (Mapping):**
   - Mở Teleop bàn phím lái robot đi vòng quanh các dãy kệ.
   - Thấy rõ trên RViz2 các mảng tường màu đen và vùng an toàn màu trắng mở rộng dần.
   - Lái robot khép vòng (loop closure), bật terminal gõ lệnh `map_saver_cli` lưu thành công `map.yaml`.
3. **Cảnh 3 (2:00 - 2:45) - Load lại bản đồ & Định vị AMCL:**
   - Tắt SLAM, khởi động lại simulation cùng launch `nav_bringup`.
   - Bản đồ tĩnh hiện lên RViz2. Dùng công cụ **2D Pose Estimate** chấm vị trí.
   - Cho thấy đám mây hạt co cụm và các tia laser khớp hoàn toàn với tường.
4. **Cảnh 4 (2:45 - 4:30) - Điều hướng tự động Nav2 (3 Goals):**
   - **Goal 1:** Gửi điểm đích ở hành lang giữa -> Robot tự vẽ đường, né kệ hàng và đến đích thành công.
   - **Goal 2:** Gửi điểm đích ở góc xa của nhà kho -> Robot tiếp tục di chuyển mượt mà.
   - **Goal 3:** Gửi điểm đích quay trở lại vị trí xuất phát ban đầu -> Robot quay đầu về đích an toàn.

---

## 7. CHECKLIST NỘP BÀI & LỆNH GIT

### Checklist nộp bài:
- [ ] Source code package đầy đủ (`CMakeLists.txt`, `package.xml`, `launch/`, `maps/`, `report/`).
- [ ] Thư mục `maps/` chứa đủ 2 tệp: `map.yaml` và `map.pgm`.
- [ ] File PDF báo cáo ngắn gọn, rõ ràng theo đúng mẫu trong `report/`.
- [ ] Video demo đăng tải lên Google Drive (chế độ **Public - Bất kỳ ai có đường liên kết đều xem được**).
- [ ] GitHub Repository công khai (Public).

### Các lệnh Git mẫu để đẩy code lên GitHub:
```bash
cd ~/workspaces/ur_gz/src/turbot4_simulation

# 1. Khởi tạo Git repo
git init

# 2. Thêm toàn bộ các file
git add .

# 3. Tạo commit đầu tiên
git commit -m "feat: complete TurtleBot 4 simulation, SLAM mapping, and Nav2 navigation package"

# 4. Đổi nhánh chính thành main
git branch -M main

# 5. Liên kết tới remote repo của bạn
git remote add origin https://github.com/ductoris/turtlebot4_hri_uet.git

# 6. Đẩy code lên GitHub
git push -u origin main
```

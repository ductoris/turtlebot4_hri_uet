# Bài Thực Hành Số 05: TurtleBot 4 Localization, Autonomous Navigation & Nhận Diện Vật Thể Với YOLOv8

> **Môn học:** Điều Khiển Robot / Robot Di Động  
> **Môi trường:** Ubuntu 22.04 LTS | ROS 2 Humble | Ignition Gazebo Fortress | Python 3.10  
> **Robot:** TurtleBot 4 Standard (iRobot Create 3 Base + RPLIDAR A1 + OAK-D Pro Camera)

---

## 1. Giới Thiệu & Mục Tiêu Đề Tài

Dự án này là một ROS 2 Python package hoàn chỉnh mang tên `tb4_autonomy_yolo`, giải quyết trọn vẹn 4 mục tiêu cốt lõi của bài thực hành:
1. **Mô phỏng môi trường (SDF World & 2D Occupancy Grid Map):** Xây dựng phòng khách mini (`home_room`) gồm tường, bàn, ghế, sofa, kệ sách và người đứng. Bản đồ 2D (`map.pgm`, `map.yaml`) được sinh đồng bộ với thế giới mô phỏng.
2. **Định vị Robot (Localization with AMCL):** Sử dụng thuật toán Adaptive Monte Carlo Localization (AMCL) kết hợp quét laser 2D từ RPLIDAR để định vị chính xác robot trên bản đồ. Hỗ trợ đặt Initial Pose tự động qua code hoặc kéo thả công cụ **2D Pose Estimate** trên RViz2.
3. **Điều hướng tự hành nhập tọa độ tương tác (Interactive Navigation):** Lập trình node Python (`nav_node.py`) sử dụng thư viện `nav2_simple_commander`. Cho phép **người dùng trực tiếp gõ tọa độ $(X, Y, \text{yaw})$ bất kỳ từ bàn phím**, robot sẽ tự động lập quỹ đạo tránh vật cản và di chuyển đến đúng vị trí trong phòng. Đồng thời hỗ trợ cơ chế tự động thoát trạm sạc (**Auto Undock**) và **bảo toàn vị trí thực tế của robot (không bị reset về vị trí xuất phát khi bật lại node)**.
4. **Nhận diện vật thể thời gian thực với AI (YOLOv8 Detection):** Lập trình node Python (`yolo_node.py`) tích hợp mô hình **YOLOv8n**, nhận luồng ảnh RGB từ camera OAK-D, phát hiện vật thể (người, bàn ghế, đồ đạc), vẽ bounding box kèm độ tin cậy và hiển thị song song khi robot đang di chuyển.

---

## 2. Cấu Trúc Thư Mục Package (`tb4_autonomy_yolo`)

```text
tb4_autonomy_yolo/
├── launch/
│   ├── sim.launch.py           # Khởi động Ignition Gazebo với world home_room + spawn TurtleBot 4
│   ├── nav.launch.py           # Khởi động Localization (AMCL) và Nav2 stack
│   ├── rviz.launch.py          # Khởi động RViz2 chuyên dụng (RobotModel, Odometry, Map, LaserScan)
│   └── demo.launch.py          # Launch file all-in-one: Nav2 + AMCL + RViz2 + YOLO Node
├── maps/
│   ├── home_room.pgm           # Bản đồ ảnh bitmap 2D (resolution: 0.05 m/pixel)
│   └── home_room.yaml          # File cấu hình siêu dữ liệu Map Server (origin, thresholds)
├── rviz/
│   └── nav_yolo.rviz           # Cấu hình RViz2 chuẩn: Map, TF, Path, AMCL, Costmaps và Live YOLO Camera
├── scripts/
│   └── generate_world_and_map.py # Script sinh đồng bộ SDF world và PGM/YAML map
├── tb4_autonomy_yolo/
│   ├── __init__.py
│   ├── nav_node.py             # Node điều hướng tự hành: Auto-undock, nhập tọa độ, bảo toàn vị trí AMCL
│   └── yolo_node.py            # Node nhận diện vật thể YOLOv8 từ camera OAK-D (tối ưu CPU)
├── worlds/
│   └── home_room.sdf           # Thế giới mô phỏng phòng khách cho Ignition Gazebo (đầy đủ ma sát)
├── package.xml                 # Khai báo metadata và các dependencies ROS 2
├── setup.py                    # Khai báo cài đặt data_files và console_scripts
├── setup.cfg
└── README.md                   # Hướng dẫn chi tiết cài đặt và vận hành
```

---

## 3. Yêu Cầu Hệ Thống & Cài Đặt Dependencies

### 3.1 Cài đặt ROS 2 Humble & TurtleBot 4 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz \
  ros-humble-nav2-simple-commander \
  ros-humble-cv-bridge
```

### 3.2 Cài đặt PyTorch (CPU) & Ultralytics YOLOv8
Để tránh xung đột phiên bản NumPy và thư viện đồ họa C++ của ROS 2, hãy cài đặt phiên bản PyTorch CPU và NumPy < 2.0:
```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip3 install "numpy<2" ultralytics
```

---

## 4. Hướng Dẫn Build Workspace

Di chuyển vào thư mục workspace ROS 2 của bạn (ví dụ `~/workspaces/ur_gz`) và tiến hành build package:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash

# Build package với symlink-install
colcon build --symlink-install --packages-select tb4_autonomy_yolo

# Source môi trường sau khi build
source install/setup.bash
```

---

## 5. Hướng Dẫn Chạy Chương Trình & Kịch Bản Quay Video Demo

Bạn có thể chạy hệ thống theo **Cách 1 (Khuyến nghị để quay video nộp bài)** hoặc **Cách 2 (Module hóa từng bước)**:

---

### 🌟 CÁCH 1: KHỞI ĐỘNG NHANH & NHẬP TỌA ĐỘ TƯƠNG TÁC (Khuyến nghị cho Video Demo)

Cách này chỉ cần mở **3 Terminal**, giao diện RViz2 và cửa sổ nhận diện YOLO sẽ tự động mở sẵn sàng:

#### 1. Terminal 1: Khởi động mô phỏng Gazebo
```bash
cd /home/ductri/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tb4_autonomy_yolo sim.launch.py
```
> *Gazebo mở căn phòng khách 3D `home_room` với bàn ghế, sofa, kệ sách, người đứng và robot TurtleBot 4.*

#### 2. Terminal 2: Khởi động toàn bộ Nav2, AMCL, RViz2 và YOLO Detection
```bash
cd /home/ductri/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch tb4_autonomy_yolo demo.launch.py
```
> *Tự động nạp bản đồ, bật AMCL localization, mở giao diện RViz2 hiển thị RobotModel 3D + Odometry + LaserScan, và mở cửa sổ camera `YOLOv8n - TurtleBot 4`.*

#### 3. Terminal 3: Chạy Node nhập tọa độ tương tác bàn phím
```bash
cd /home/ductri/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true -p interactive:=true
```

Màn hình Terminal 3 sẽ hiển thị giao diện tương tác:
```text
====================================================================
🤖 CHẾ ĐỘ NHẬP TỌA ĐỘ TƯƠNG TÁC (INTERACTIVE NAVIGATION)
🗺️  Phạm vi bản đồ phòng hợp lệ: X ∈ [-2.5, 2.5] m, Y ∈ [-2.0, 2.0] m
💡 Gợi ý 3 tọa độ quan sát vật thể cho video demo:
   - Điểm 1 (Khu vực bàn ghế):      1.0  -0.3   0
   - Điểm 2 (Khu vực kệ sách):       0.6   1.2  90
   - Điểm 3 (Khu vực người & sofa): -0.6   0.5 -150
👉 Cú pháp: x y [yaw_độ]  (Ví dụ: 1.0 -0.3 0 hoặc gõ "q" để thoát)
====================================================================

📍 [Goal 1] Vị trí hiện tại của robot: X = 0.00 m, Y = 0.00 m
>> Nhập tọa độ Goal 1 (x y [yaw_độ]): 
```

**Thực hiện nhập 3 tọa độ liên tiếp để quay video:**
1. **Nhập Goal 1:** Gõ `1.0 -0.3 0` rồi nhấn `Enter`.
   * Robot tự động lùi thoát Dock sạc, quay xe và tiến đến khu vực bàn ghế.
   * Cả Gazebo và RViz2 di chuyển đồng bộ 100%.
   * Đến đích, robot dừng lại **4 giây** để cửa sổ YOLO nhận diện `chair`.
2. **Nhập Goal 2:** Gõ `0.6 1.2 90` rồi nhấn `Enter`.
   * Robot từ vị trí hiện tại (không bị reset) tự động tính đường đi lên phía bắc trước kệ sách.
   * Đến đích, dừng **4 giây** để YOLO nhận diện đồ vật khu vực này.
3. **Nhập Goal 3:** Gõ `-0.6 0.5 -150` rồi nhấn `Enter`.
   * Robot từ kệ sách quay đầu sang phía tây, tiến đến trước mặt người đứng và ghế sofa.
   * Đến đích, dừng **4 giây** để YOLO phát hiện `person`.
4. **Kết thúc:** Gõ `q` để thoát chương trình khi đã hoàn thành đủ 3 điểm đến liên tiếp.

---

### 🔹 CÁCH 2: CHẠY MODULE HÓA TỪNG BƯỚC (Dành cho việc kiểm tra độc lập)

Nếu muốn khởi động từng thành phần riêng biệt để tự tay kiểm tra **2D Pose Estimate** hoặc **Nav2 Goal** bằng chuột trên RViz2:

1. **Terminal 1: Gazebo Simulation**
   ```bash
   ros2 launch tb4_autonomy_yolo sim.launch.py
   ```
2. **Terminal 2: Nav2 & AMCL Localization**
   ```bash
   ros2 launch tb4_autonomy_yolo nav.launch.py rviz:=false
   ```
3. **Terminal 3: RViz2 Chuyên Dụng**
   ```bash
   ros2 launch tb4_autonomy_yolo rviz.launch.py
   ```
   * *Hiển thị RobotModel 3D, Odometry (`/odom`), LaserScan (`/scan`), AMCL Pose.*
   * *Có thể bấm `2D Pose Estimate` trên thanh công cụ RViz để kéo thả vị trí ban đầu bất kỳ.*
   * *Có thể bấm `Nav2 Goal` để click điểm đến tự do trên bản đồ bằng chuột.*
4. **Terminal 4: YOLOv8 Detection Node**
   ```bash
   ros2 run tb4_autonomy_yolo yolo_node --ros-args -p use_sim_time:=true
   ```
5. **Terminal 5: Điều khiển Robot**
   * *Chế độ nhập tọa độ từ bàn phím:*
     ```bash
     ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true -p interactive:=true
     ```
   * *Hoặc truyền trực tiếp trên dòng lệnh:*
     ```bash
     ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true -p x:=1.0 -p y:=-0.3 -p yaw_deg:=0
     ```
   * *Hoặc chạy tự động 3 waypoint mặc định:*
     ```bash
     ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true
     ```

---

## 6. Điểm Cải Tiến Kỹ Thuật Đáng Chú Ý

1. **Cơ chế Bảo toàn Vị trí AMCL (Không bị Reset về Start khi chạy lại Node):**
   * Trong phiên bản cũ, mỗi khi bật lại `nav_node`, chương trình gửi lại Initial Pose $(0, 0, 0)$, khiến RViz kéo giật robot về vạch xuất phát trong khi Gazebo robot đang ở vị trí khác $\rightarrow$ dẫn đến robot bị mất phương hướng và đâm vào tường.
   * `nav_node.py` mới đã được nâng cấp kiểm tra topic latched `/amcl_pose`. Nếu robot đã có vị trí định vị trên bản đồ, node sẽ **giữ nguyên 100% tọa độ thực tế hiện tại**, cho phép người dùng tắt/mở terminal nhiều lần mà vị trí giữa Gazebo và RViz luôn hoàn toàn đồng nhất.
2. **Khắc phục lỗi bánh xe đứng yên trong Gazebo (Auto Undock & Dock Clearing):**
   * Mô phỏng TurtleBot 4 mặc định spawn ở trạng thái `is_docked: True` với khối va chạm `standard_dock` chặn trước mũi xe và node an toàn `motion_control` chặn lệnh `/cmd_vel`.
   * `nav_node.py` tự động phát hiện nếu robot đang trên dock thì gửi action `/undock` (`irobot_create_msgs/action/Undock`) và gọi Ignition service xóa bỏ entity `standard_dock` khỏi phòng. Nếu robot đã tự do (`is_docked: False`), node sẽ bỏ qua bước này an toàn.
3. **Kiểm tra khoảng cách thực tế (Euclidean Distance Validation):**
   * Sử dụng `math.hypot(gx - cur_x, gy - cur_y)` dựa trên TF `map -> base_link` để tính chính xác khoảng cách đến đích, tránh trường hợp `feedback.distance_remaining == 0.0` dẫn đến kết thúc goal ảo.
4. **Bộ lọc kiểm tra biên phòng (Room Boundary Validation):**
   * Tọa độ người dùng nhập được kiểm tra chặt chẽ trong phạm vi an toàn $X \in [-2.5, 2.5]\text{ m}, Y \in [-2.0, 2.0]\text{ m}$ trước khi gửi tới Nav2.
5. **Tối ưu hóa hiệu năng YOLOv8 trên CPU:**
   * Giới hạn `torch.set_num_threads(2)`, resize ảnh `imgsz=480` và rate limiter ~8 FPS giúp YOLO chạy mượt mà mà chỉ chiếm ~40% CPU, không làm ảnh hưởng đến tần số điều khiển 20Hz của Nav2 Controller.

---

## 7. Danh Mục Topic & Action Chính

| Tên Topic / Action | Kiểu dữ liệu | Mô tả chức năng |
| :--- | :--- | :--- |
| `/oakd/rgb/preview/image_raw` | `sensor_msgs/msg/Image` | Luồng ảnh RGB từ camera OAK-D của TurtleBot 4 |
| `/yolo/image_annotated` | `sensor_msgs/msg/Image` | Luồng ảnh kết quả nhận diện (Bounding Box + Label + Conf) |
| `/scan` | `sensor_msgs/msg/LaserScan` | Dữ liệu quét LiDAR 360 độ từ RPLIDAR A1 |
| `/odom` | `nav_msgs/msg/Odometry` | Dữ liệu Odometry phản hồi từ bộ truyền động bánh xe |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Tọa độ và hướng ước lượng của Robot trên Map từ AMCL |
| `/plan` | `nav_msgs/msg/Path` | Quỹ đạo đường đi toàn cục do Navfn Planner vạch ra |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Vận tốc điều khiển dài và góc gửi tới Create 3 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action Server nhận tọa độ mục tiêu và thực thi điều hướng |
| `/undock` | `irobot_create_msgs/action/Undock` | Action giải phóng robot rời khỏi trạm sạc |

---

## 8. Hướng Dẫn Bố Cục Màn Hình Quay Video Demo

1. **Bố cục màn hình đề xuất (Chia 3 phần rõ nét):**
   * **Cửa sổ 1 (Nửa trái màn hình):** Cửa sổ **RViz2** hiển thị bản đồ phòng, RobotModel, LaserScan, vệt đường đi của robot và đám mây hạt AMCL.
   * **Cửa sổ 2 (Nửa phải - phía trên):** Cửa sổ **Ignition Gazebo** mô phỏng 3D robot chạy thực tế trong phòng khách, cùng với cửa sổ camera **YOLOv8** phát hiện bounding box live.
   * **Cửa sổ 3 (Nửa phải - phía dưới):** Cửa sổ **Terminal** chạy `interactive:=true` nơi bạn gõ các tọa độ `Goal 1`, `Goal 2`, `Goal 3`.
2. **Tiến trình trong video:**
   * **[0:00 - 0:15]:** Show Terminal 1 và 2 khởi chạy Gazebo và Nav2/RViz/YOLO.
   * **[0:15 - 0:30]:** Show vị trí robot đã được định vị ban đầu trên RViz2.
   * **[0:30 - 1:10]:** Tại Terminal 3, gõ Goal 1 `1.0 -0.3 0` $\rightarrow$ Robot lùi khỏi dock, di chuyển đến trước bàn ghế, camera YOLO nhận diện `chair`.
   * **[1:10 - 1:50]:** Gõ Goal 2 `0.6 1.2 90` $\rightarrow$ Robot từ vị trí hiện tại di chuyển đến trước kệ sách, camera YOLO nhận diện vật thể.
   * **[1:50 - 2:30]:** Gõ Goal 3 `-0.6 0.5 -150` $\rightarrow$ Robot di chuyển đến trước người đứng và sofa, camera YOLO nhận diện `person`.
   * **[2:30 - 2:40]:** Gõ `q` để hoàn tất toàn bộ lộ trình điều hướng.
3. **Upload & Nộp bài:** Tải video lên Google Drive (chọn quyền *"Bất kỳ ai có đường liên kết đều có thể xem"*), dán link vào file Báo cáo PDF.

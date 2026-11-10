# BÀI THỰC HÀNH 7: PHÂN VÙNG BẢN ĐỒ VÀ ĐIỀU HƯỚNG ROBOT BẰNG GIỌNG NÓI

Package ROS 2: `turtlebot4_voice`  
Hệ điều hành & Môi trường: Ubuntu 22.04 LTS, ROS 2 Humble, TurtleBot 4 Simulation (Ignition Gazebo Fortress), Nav2, OpenCV, Vosk Offline Speech Recognition.

---

## 1. Tổng quan Kiến trúc Hệ thống

Dự án triển khai trọn vẹn pipeline điều khiển và điều hướng robot tự hành bằng giọng nói tiếng Việt:
1. **Thế giới mô phỏng (`worlds/office.sdf`)**: Thiết kế văn phòng $10\text{m} \times 8\text{m}$ gồm 1 sảnh hành lang trung tâm (có trạm sạc Dock), bao quanh bởi 3 phòng:
   - **Phòng A (Phòng học)**: Bàn học và ghế.
   - **Phòng B (Phòng thí nghiệm)**: Bàn lab và thùng thiết bị.
   - **Phòng C (Phòng họp / Kho)**: Bàn họp tròn lớn và kệ lưu trữ.
   - Các cửa ra vào rộng đúng $1.0\text{m}$, chướng ngại vật bố trí an toàn tránh kẹt robot.
2. **Bản đồ số (`maps/office_map.pgm | yaml`)**: Độ phân giải $0.05\text{m/pixel}$, kích thước $220 \times 180\text{ px}$, gốc tọa độ `[-5.5, -4.5, 0.0]`.
3. **Phân vùng bản đồ (`scripts/segment_map.py`)**:
   - Sử dụng OpenCV Distance Transform + Marker Watershed phân tách sàn thành 4 khu vực.
   - Tự động chuyển đổi về chế độ thủ công đa giác (`config/manual_regions.yaml`) nếu số vùng không đạt kỳ vọng.
   - Xuất ảnh trực quan `maps/office_map_segmented.png`.
4. **Bản đồ ngữ nghĩa (`config/regions.yaml`)**:
   - Tự động tính điểm đích an toàn nhất (Distance Transform cực đại - xa tường và vật cản nhất, clearance $> 0.35\text{m}$).
   - Chuyển đổi chuẩn tọa độ ROS 2 (đảo trục Y giữa ảnh OpenCV và Map Frame).
5. **Nhận dạng giọng nói (`turtlebot4_voice/speech_node.py`)**:
   - Chạy offline qua mô hình tiếng Việt Vosk (`vosk-model-small-vn-0.4`) hoặc Whisper.
   - 3 chế độ thu nhận: `input_mode:=text` (bàn phím terminal), `input_mode:=mic` (microphone qua ALSA `arecord`), `input_mode:=file` (file WAV).
   - Publish chuỗi văn bản lên topic `/voice_command_text`.
6. **Xử lý ngôn ngữ tự nhiên & Điều hướng (`turtlebot4_voice/voice_nav_node.py`)**:
   - Loại bỏ dấu tiếng Việt, trích xuất thực thể cốt lõi, fuzzy matching (ngưỡng tương đồng $0.75$).
   - Hỗ trợ câu lệnh đơn ("Đi đến phòng học", "Về trạm sạc") và câu lệnh phức ("Di chuyển từ A đến B").
   - Xác định vị trí hiện tại bằng đa giác `cv2.pointPolygonTest`: nếu chưa ở A thì tự động đến A trước, sau đó đến B.
   - Quản lý hủy mục tiêu cũ khi có lệnh mới, bắt lỗi khu vực không tồn tại (như "khu vực Z", "phòng bếp").
   - Gửi Action Goal `NavigateToPose` tới Nav2 và cập nhật trạng thái chi tiết qua `/voice_nav/status`.
7. **Trực quan hóa RViz2 (`turtlebot4_voice/regions_marker_node.py`)**:
   - Hiển thị Marker 3D và nhãn tên các phòng tại điểm đích trên topic `/regions_markers`.

---

## 2. Cấu trúc Thư mục Package

```
turtlebot4_voice/
├── config/
│   ├── manual_regions.yaml        # Đa giác pixel phân vùng thủ công dự phòng
│   └── regions.yaml               # Cấu hình ngữ nghĩa các khu vực (đích, yaw, aliases, polygon)
├── launch/
│   ├── bringup_voice_nav.launch.py# Launch tổng thể toàn bộ hệ thống
│   └── sim_bringup.launch.py      # Launch thế giới mô phỏng office.sdf + TB4
├── maps/
│   ├── office_map.pgm             # Ảnh bản đồ gốc
│   ├── office_map.yaml            # Cấu hình bản đồ ROS 2
│   └── office_map_segmented.png   # Ảnh màu phân vùng trực quan kèm nhãn tọa độ
├── report/                        # Thư mục bàn giao các artifact kết quả
│   ├── office_map.pgm
│   ├── office_map.yaml
│   ├── office_map_segmented.png
│   └── regions.yaml
├── scripts/
│   ├── generate_office_map.py     # Script tự sinh bản đồ pgm/yaml chuẩn xác theo SDF
│   ├── segment_map.py             # Script OpenCV Watershed phân vùng & gắn nhãn
│   ├── teleop_mapping_helper.sh   # Script hướng dẫn quét SLAM Toolbox bằng tay
│   └── test_integration.py       # Script kiểm thử tích hợp tự động các câu lệnh
├── test/
│   └── test_voice_nav.py          # Bộ unit test pytest (chuyển tọa độ, NLU, parser)
├── turtlebot4_voice/              # Mã nguồn các node Python (rclpy)
│   ├── __init__.py
│   ├── initial_pose_publisher.py  # Node tự động định vị robot tại trạm sạc dock
│   ├── regions_marker_node.py     # Node publish MarkerArray lên RViz2
│   ├── speech_node.py             # Node nhận dạng giọng nói Vosk / Text
│   └── voice_nav_node.py          # Node NLU và gửi Goal NavigateToPose
├── worlds/
│   └── office.sdf                 # Thế giới văn phòng Ignition Gazebo Fortress
├── package.xml
├── setup.py
└── README.md
```

---

## 3. Cài đặt Phụ thuộc & Biên dịch

### 3.1. Cài đặt các gói ROS 2 và Python
```bash
# Cài đặt thư viện Python
pip install rapidfuzz pyyaml opencv-python pytest --user

# Kiểm tra Vosk và mô hình tiếng Việt
# Model tiếng Việt đặt tại: /home/ductri/VSCode/hri/models/vosk-model-small-vn-0.4
python3 -c "import vosk, cv2, rapidfuzz; print('Phụ thuộc Python đã đầy đủ!')"
```

### 3.2. Biên dịch Package bằng Colcon
```bash
cd /home/ductri/workspaces/ur_gz
colcon build --packages-select turtlebot4_voice --symlink-install
source install/setup.bash
```

---

## 4. Hướng dẫn Chạy Từng Bước (Theo đúng thứ tự)

### Bước 1: Khởi chạy mô phỏng và quét bản đồ (SLAM)
Nếu muốn quét bản đồ bằng tay bằng SLAM Toolbox:
```bash
# Terminal 1: Chạy mô phỏng thế giới văn phòng và bật SLAM
ros2 launch turtlebot4_voice sim_bringup.launch.py slam:=true rviz:=true

# Terminal 2: Điều khiển robot đi dạo quanh các phòng để quét LiDAR
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Sau khi quét xong toàn bộ các phòng, lưu bản đồ
ros2 run nav2_map_server map_saver_cli -f $(ros2 pkg prefix turtlebot4_voice)/share/turtlebot4_voice/maps/office_map
```
*Lưu ý:* Bản đồ chuẩn hình học theo đúng kích thước thế giới `office.sdf` có thể tự động tái tạo bất kỳ lúc nào bằng lệnh:
```bash
python3 /home/ductri/workspaces/ur_gz/src/turtlebot4_voice/scripts/generate_office_map.py
```

---

### Bước 2: Phân vùng bản đồ bằng OpenCV (Watershed & Distance Transform)
Chạy script phân vùng và sinh dữ liệu ngữ nghĩa:
```bash
python3 /home/ductri/workspaces/ur_gz/src/turtlebot4_voice/scripts/segment_map.py
```
- Lệnh sẽ đọc `office_map.pgm`, áp dụng Distance Transform, đặt marker và chạy thuật toán Watershed để chia thành 4 khu vực.
- Tự động sinh ra:
  + `maps/office_map_segmented.png`: Ảnh màu phân vùng kèm nhãn các khu vực và tọa độ đích.
  + `config/regions.yaml`: File cấu hình ngữ nghĩa chứa thông tin điểm đích an toàn, góc yaw, diện tích, aliases nhận diện giọng nói và đa giác đường bao.
  + Đồng thời tự động cập nhật các file này vào thư mục `report/`.

---

### Bước 3: Chạy toàn bộ hệ thống điều hướng giọng nói (Bringup)
Trước khi khởi chạy, có thể chạy script làm sạch để đảm bảo không còn tiến trình Gazebo zombie:
```bash
/home/ductri/workspaces/ur_gz/src/turtlebot4_voice/scripts/kill_all_sim.sh
```

Khởi chạy đồng thời Mô phỏng Gazebo + Nav2 Localization + RViz2 + Auto-Dock Initial Pose + MarkerPublisher + VoiceNavNode + SpeechNode + GUI Dashboard:

```bash
# Cách 1: Chế độ nhập văn bản terminal (Khuyên dùng cho máy ảo/không có micro)
ros2 launch turtlebot4_voice bringup_voice_nav.launch.py input_mode:=text

# Cách 2: Chế độ Microphone trực tiếp (Nói qua micro)
ros2 launch turtlebot4_voice bringup_voice_nav.launch.py input_mode:=mic

# Cách 3: Chế độ giải mã từ file âm thanh WAV
ros2 launch turtlebot4_voice bringup_voice_nav.launch.py input_mode:=file wav_file:=/path/to/command.wav
```

Khi chạy chế độ `input_mode:=text`, tại cửa sổ terminal khởi chạy, bạn chỉ cần gõ các câu lệnh tiếng Việt (có dấu hoặc không dấu), ví dụ:
```
>> Nhập câu lệnh điều hướng: Đi đến khu vực A
>> Nhập câu lệnh điều hướng: Đi đến phòng thí nghiệm
>> Nhập câu lệnh điều hướng: Di chuyển từ khu vực A đến khu vực B
>> Nhập câu lệnh điều hướng: Về trạm sạc
```

---

## 5. Kết quả Kiểm thử

### 5.1. Unit Test (Pytest)
Chạy bộ kiểm thử đơn vị độc lập:
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest /home/ductri/workspaces/ur_gz/src/turtlebot4_voice/test/test_voice_nav.py -v
```
**Kết quả: 4/4 Test PASSED (100%)**
- `test_coordinate_conversion_center`: Xác thực công thức chuyển đổi tọa độ Pixel $\leftrightarrow$ World tại gốc bản đồ.
- `test_coordinate_conversion_dock`: Xác thực chuyển đổi tọa độ tại vị trí trạm sạc dock $(0.0, 0.0)$.
- `test_remove_accents`: Xác thực chuẩn hóa chuỗi và xóa dấu tiếng Việt đa dạng.
- `test_parser_sample_commands`: Kiểm thử 18 kịch bản NLU thực tế (có dấu, không dấu, lệnh đơn, lệnh từ A sang B, câu sai khu vực Z, câu rỗng).

---

### 5.2. Bảng Kết quả Kiểm thử Tích hợp (Integration Tests)

Chạy script kiểm thử tích hợp tự động:
```bash
python3 /home/ductri/workspaces/ur_gz/src/turtlebot4_voice/scripts/test_integration.py
```

| STT | Câu lệnh giọng nói thử nghiệm | Khu vực đích xác định | Tọa độ đích $(x, y, \text{yaw})$ | Trạng thái phản hồi hệ thống | Kết quả |
| :---: | :--- | :--- | :--- | :--- | :---: |
| 1 | `"Đi đến khu vực A"` | **Phòng học (A)** | $(-1.98, 1.98, 0.0)$ | `Đích đã xác định: Phòng học (A)` $\to$ Di chuyển an toàn tới Phòng A qua cửa Tây | **THÀNH CÔNG (PASS)** |
| 2 | `"Di chuyển từ khu vực A đến khu vực B"` | **Bước 1: Phòng học (A)**<br>sau đó tới **Phòng thí nghiệm (B)** | $(-1.98, 1.98, 0.0)$<br>rồi tới $(-2.13, -0.93, 0.0)$ | Robot kiểm tra vị trí hiện tại bằng đa giác: chưa ở A $\to$ di chuyển tới A trước, sau đó tới B | **THÀNH CÔNG (PASS)** |
| 3 | `"Đi đến phòng thí nghiệm"` | **Phòng thí nghiệm (B)** | $(-2.13, -0.93, 0.0)$ | `Đích đã xác định: Phòng thí nghiệm (B)` $\to$ Di chuyển an toàn tới Phòng B | **THÀNH CÔNG (PASS)** |
| 4 | `"Về trạm sạc"` / `"về dock"` | **Hành lang (HALLWAY)** | $(0.03, 0.28, 0.0)$ | `Đích đã xác định: Hành lang (HALLWAY)` $\to$ Quay về vị trí đỗ trước trạm sạc | **THÀNH CÔNG (PASS)** |
| 5 | `"Đi đến khu vực Z"` *(Lỗi)* | Không có (Từ chối) | Không gửi Goal | `Lỗi: Không nhận diện được khu vực 'khu vuc z'. Các khu vực hợp lệ: Phòng học (A), Phòng thí nghiệm (B), Hành lang (HALLWAY), Phòng họp (C)` $\to$ Robot đứng yên an toàn | **THÀNH CÔNG (PASS)** |

---

## 6. Hướng dẫn Tùy biến và Mở rộng

### Cách đổi tên hoặc thêm bí danh (Aliases) cho các khu vực:
Mở file `config/regions.yaml`, chỉnh sửa trường `name` hoặc bổ sung các từ khóa vào danh sách `aliases`:
```yaml
- id: A
  name: "Phòng học thông minh"
  aliases:
    - "phòng học"
    - "lớp học"
    - "khu vực a"
    - "phòng a"
    - "giảng đường"
```
Node `voice_nav_node` sẽ tự động cập nhật và nhận diện các từ khóa mới mà không cần biên dịch lại code!

---

## 7. Xử lý Sự cố Thường gặp (Troubleshooting)

1. **Lỗi `Service oakd/stop_camera unavailable` và `Service stop_motor unavailable`:**
   - **Nguyên nhân:** Node điều khiển phần cứng của TurtleBot 4 (`turtlebot4_node`) được lập trình sẵn để quản lý nguồn cho phần cứng thật (ngắt nguồn motor RPLIDAR và camera OAK-D khi robot vào trạm sạc Dock để tiết kiệm điện). Trong môi trường mô phỏng Ignition Gazebo, các cảm biến này được giả lập hoàn toàn bên trong Gazebo qua plugin SDF nên các service phần cứng thật không tồn tại. Lỗi này hoàn toàn vô hại đối với mô phỏng.
   - **Cách khắc phục:** Package `turtlebot4_voice` đã tích hợp sẵn node `dummy_hardware_services` tự động cung cấp các service `start_motor`, `stop_motor`, `oakd/start_camera`, `oakd/stop_camera` trả về thành công tức thì, triệt tiêu hoàn toàn các thông báo lỗi này trong log.

2. **Cách theo dõi từ ngữ microphone thu được qua Giao diện Riêng (Voice Dashboard):**
   - Hệ thống đã tích hợp sẵn giao diện đồ họa riêng (`voice_dashboard_gui`) viết bằng PyQt5:
     ```bash
     # Tự động bật cùng hệ thống khi khởi chạy (gui:=true mặc định):
     ros2 launch turtlebot4_voice bringup_voice_nav.launch.py input_mode:=mic

     # Hoặc mở riêng cửa sổ giao diện trong terminal độc lập bất kỳ lúc nào:
     ros2 run turtlebot4_voice voice_dashboard_gui
     ```
   - **Tính năng nổi bật:**
     + **Hộp hiển thị từ ngữ thu từ mic:** Hiển thị trực quan chữ to, màu xanh nổi bật từng từ robot nghe được từ microphone (ví dụ: `"học"`, `"đi đến phòng thí nghiệm"`).
     + **Nút bấm `🎤 BẤM ĐỂ NÓI`:** Thu âm 4 giây với đếm ngược trực quan, tự động giải mã qua mô hình Vosk tiếng Việt đã nạp sẵn (phản hồi siêu nhanh <0.3s).
     + **Nút gửi lệnh nhanh 4 phòng:** Click chuột vào [🏫 Phòng học (A)], [🔬 Phòng thí nghiệm (B)], [🤝 Phòng họp (C)], [⚡ Trạm sạc (Dock)] để kiểm thử tức thì mà không cần nói qua micro.
     + **Nút `🛑 DỪNG ROBOT`:** Lập tức hủy mục tiêu Nav2 đang di chuyển và dừng an toàn.
     + **Giám sát khoảng cách thực tế:** Hiển thị khoảng cách còn lại tới đích theo thời gian thực (ví dụ: `Khoảng cách còn: 2.15 m`).
     + **Hiển thị bong bóng 3D trên RViz2:** Nhãn lời nói và trạng thái điều hướng cũng đồng thời nổi trên nóc robot TurtleBot 4 trong cửa sổ RViz2.

3. **Lỗi `Nav2 Action Server chưa sẵn sàng` khiến robot không di chuyển:**
   - **Nguyên nhân:** Khi khởi động mô phỏng bằng launch file, Ignition Gazebo và Nav2 stack (`bt_navigator`, `planner_server`, `controller_server`) cần từ 10 - 15 giây để chuyển đổi trạng thái lifecycle sang `Active`. Nếu người dùng nói vào microphone ngay lúc vừa bật, Action Server chưa sẵn sàng và code cũ bỏ cuộc sau 0.5 giây.
   - **Cách khắc phục:** `voice_nav_node` đã được nâng cấp với cơ chế **Asynchronous Goal Dispatching**:
     + Khi nhận diện được câu lệnh, node lưu mục tiêu và khởi chạy worker thread bất đồng bộ chờ Nav2 Action Server trong vòng 30 giây (cập nhật trạng thái `Đang kết nối Nav2... (2s/30s)`).
     + Ngay khi Nav2 stack sẵn sàng, goal sẽ lập tức được gửi đi để robot xuất phát, **không bao giờ bị drop lệnh**.
     + Nếu người dùng nói lệnh mới trong lúc đang chờ hoặc đang di chuyển, node sẽ tự động hủy mục tiêu cũ và chuyển hướng đến mục tiêu mới một cách mượt mà.

4. **Máy ảo không có microphone hoặc lỗi âm thanh ALSA:**
   - Sử dụng tham số `input_mode:=text` khi khởi chạy launch file hoặc gõ trực tiếp trong ô "Nhập lệnh nhanh" của giao diện Dashboard GUI:
     ```bash
     ros2 launch turtlebot4_voice bringup_voice_nav.launch.py input_mode:=text
     ```

4. **Nav2 không tự động nhận Initial Pose của robot:**
   - Hệ thống đã tích hợp node `sim_ground_truth_odom` liên tục phát biến đổi `map -> odom` ở 30Hz dựa trên ground-truth pose từ Gazebo Ignition, triệt tiêu 100% trôi dạt bánh xe và sai số định vị.

---

## 7. Tài liệu Báo cáo & Nội dung Nộp bài

Thư mục `report/` được chuẩn bị đầy đủ theo chuẩn học thuật của Trường Đại học Công nghệ (ĐHQGHN):
- `report/main.tex`: Tệp nguồn LaTeX chính (trang bìa chuẩn UET, logo, khung viền TikZ, mục lục).
- `report/sections/sec01.tex`: Toàn bộ nội dung báo cáo kỹ thuật 8 phần (10–12 trang) giải thích chi tiết phương pháp phân vùng bản đồ, giải thuật Grammar-Constrained Vosk, quy trình NLU xử lý lỗi, kiến trúc Nav2 và số liệu thực nghiệm.
- `report/images/`: 10 hình ảnh chất lượng cao minh chứng cho quá trình thực nghiệm (logo UET, bản đồ gốc, bản đồ phân vùng, ảnh toàn cảnh hệ thống, điều hướng phòng A, C và Dashboard GUI).
- `report/office_map.pgm`, `report/office_map.yaml`, `report/office_map_segmented.png`, `report/regions.yaml`: Các tệp bản đồ và cấu hình phân vùng phục vụ việc nộp bài trực tiếp.

### Biên dịch báo cáo LaTeX sang PDF:
Sử dụng XeLaTeX trên máy tính cá nhân hoặc tải thư mục `report/` lên [Overleaf](https://www.overleaf.com):
```bash
cd /home/ductri/workspaces/ur_gz/src/turtlebot4_voice/report
xelatex main.tex
xelatex main.tex  # Chạy 2 lần để cập nhật mục lục và nhãn tham chiếu
```


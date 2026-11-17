# TurtleBot 4 Facial Emotion Controller (`turtlebot4_emotion`)

Package ROS 2 Humble điều khiển robot di động **TurtleBot 4** trong môi trường mô phỏng (Ignition Gazebo / Classic) dựa trên nhận diện biểu cảm khuôn mặt thời gian thực qua Webcam hoặc Video.

---

## 1. Cấu trúc Package

```text
turtlebot4_emotion/
├── CMakeLists.txt / setup.py       # File cấu hình ament_python
├── setup.cfg                       # Đường dẫn cài đặt script ROS 2
├── package.xml                     # Khai báo phụ thuộc ROS 2
├── requirements.txt                # Thư viện Python AI & Computer Vision
├── README.md                       # Hướng dẫn chi tiết
├── resource/
│   └── turtlebot4_emotion          # Marker index package
├── config/
│   └── params.yaml                 # Tham số cấu hình YAML
├── launch/
│   └── emotion_tb4.launch.py       # Launch file khởi chạy hệ thống
├── turtlebot4_emotion/
│   ├── __init__.py
│   └── emotion_tb4_node.py         # Node chính điều khiển và nhận diện
└── test/
    └── test_emotion_mapping.py     # Unit test kiểm tra logic ánh xạ
```

---

## 2. Bảng ánh xạ biểu cảm sang hành vi di chuyển

| Biểu cảm | Hành động | Vận tốc (`linear.x`, `angular.z`) | Ghi chú |
|---|---|---|---|
| **happy** | Quỹ đạo cong sang trái | `x = 0.15 m/s, z = +0.5 rad/s` | Rẽ trái mềm |
| **sad** | Quỹ đạo cong sang phải | `x = 0.15 m/s, z = -0.5 rad/s` | Rẽ phải mềm |
| **neutral** | Dừng tại chỗ | `x = 0.0 m/s, z = 0.0 rad/s` | Giữ nguyên vị trí |
| **angry** | Lùi nhẹ về sau | `x = -0.08 m/s, z = 0.0 rad/s` | Lùi an toàn |
| **surprise** | Xoay 360° tại chỗ | `x = 0.0 m/s, z = 0.8 rad/s` | Chạy đúng `2π/0.8 ≈ 7.85s` rồi dừng, không bị ngắt quãng bởi biểu cảm mới |
| **fear** | Tiến nhẹ về trước | `x = 0.10 m/s, z = 0.0 rad/s` | Tiến thẳng chậm |
| **disgust / mất mặt** | Dừng robot | `x = 0.0 m/s, z = 0.0 rad/s` | Mất mặt quá 1.0s tự động dừng khẩn cấp |

---

## 3. Các đặc điểm kỹ thuật nổi bật

1. **Hiệu năng & FPS mượt mà:**
   - Bộ dò mặt OpenCV Haar Cascade chạy ở luồng chính (main thread ~30 FPS).
   - Module nhận diện biểu cảm DeepFace chạy trong Worker Thread bất đồng bộ, dùng tham số `detector_backend='skip'` với khuôn mặt đã crop sẵn để giảm độ trễ từ 500ms xuống chỉ còn ~30-50ms.
2. **Khử rung giật (Debounce / Smooth):**
   - Áp dụng thuật toán Bỏ phiếu đa số (Majority Voting) trên cửa sổ trượt $N = 5$ khung hình.
3. **Bộ định thời ổn định:**
   - Topic `/cmd_vel` được phát cố định ở tần số **10 Hz** qua ROS 2 Timer.
4. **Máy trạng thái Finite State Machine (FSM):**
   - Hành vi xoay 360° của `surprise` là FSM non-blocking: khi đang xoay, robot bỏ qua mọi biểu cảm mới cho đến khi hoàn thành đúng $360^\circ$ ($2\pi/0.8 \approx 7.85$ giây).
5. **Giao diện OpenCV HUD thời gian thực:**
   - Hiển thị bounding box quanh mặt, tên biểu cảm, độ tin cậy %, hành động hiện tại, vận tốc tịnh tiến/góc đang gửi, FPS và hướng dẫn phím.
6. **Thoát an toàn (Clean Shutdown):**
   - Khi bấm phím `q` hoặc `Ctrl+C`, node phát lệnh `Twist = 0` ba lần, đóng camera, dọn cửa sổ và giải phóng ROS 2 node sạch sẽ.
7. **Tự động Undock:**
   - Tích hợp Action Client `/undock` (`irobot_create_msgs/action/Undock`) tự động đưa robot rời dock sạc khi vừa khởi động.
8. **Chế độ kiểm tra bằng phím số (Manual Override):**
   - Trong cửa sổ OpenCV, bạn có thể bấm trực tiếp các phím:
     - `1`: Happy
     - `2`: Sad
     - `3`: Neutral
     - `4`: Angry
     - `5`: Surprise
     - `6`: Fear
     - `0`: Tắt override, quay về nhận diện tự động từ webcam.

# Bài tập 6: Giao tiếp phi ngôn ngữ và điều khiển TurtleBot 4 bằng cử chỉ tay

Dự án điều khiển TurtleBot 4 trong môi trường mô phỏng Ignition Gazebo bằng nhận diện cử chỉ bàn tay thời gian thực qua Webcam sử dụng OpenCV và MediaPipe Tasks API.

---

## 1. Cấu trúc thư mục

```text
.
├── .gitignore
├── README.md
└── gesture_turtlebot4/
    ├── config/
    │   └── params.yaml                   # Toan bo tham so he thong
    ├── gesture_turtlebot4/
    │   ├── __init__.py
    │   ├── gesture_recognition_node.py   # Node AI MediaPipe + Webcam HUD
    │   └── robot_behavior_node.py        # Node FSM dieu khien robot
    ├── launch/
    │   └── gesture_control.launch.py     # Launch file chay ca 2 node
    ├── models/
    │   └── gesture_recognizer.task       # Model AI Google Gesture Recognizer
    ├── package.xml
    ├── resource/
    ├── scripts/
    │   ├── download_model.sh             # Script tai model tu Google Storage
    │   └── webcam_test.py                # Script kiem tra webcam doc lap
    ├── setup.cfg
    ├── setup.py
    └── worlds/
        └── empty_room.sdf                # Phong trong 12m x 12m khong vat can
```

---

## 2. Bảng ánh xạ cử chỉ & Máy trạng thái (FSM)

| Cử chỉ | Trạng thái Robot | Hành vi thực tế của TurtleBot 4 |
|:---:|:---:|---|
| **Wave** | `EXECUTING_WAVE` | Robot lắc lư xoay trái - phải 3 nhịp để chào lại (10 giây), xong tự về `IDLE`. |
| **Pointing_Up** | `SELECTED_FORWARD_BACK` | **CHỌN** chế độ tiến - lùi (Robot đứng yên chờ `Thumb_Up`). |
| **Victory** | `SELECTED_CIRCLE` | **CHỌN** chế độ chạy vòng tròn (Robot đứng yên chờ `Thumb_Up`). |
| **Thumb_Up** | `EXECUTING` | **XÁC NHẬN** và bắt đầu chạy hành vi đã chọn (`Tiến - Lùi` hoặc `Vòng tròn`). |
| **Open_Palm** | `IDLE` | **DỪNG KHẨN CẤP NGAY LẬP TỨC** (Ưu tiên cao nhất, ngắt mọi hành vi). |
| **Closed_Fist** | `IDLE` | **HỦY** hành vi đang chọn / đang chạy, trở về `IDLE`. |
| **Mất tay > 2s** | `IDLE` | **Watchdog an toàn kích hoạt**, robot tự động dừng lại. |

---

## 3. Cài đặt & Build Package

### Cài đặt thư viện AI (Python 3.10 trên Ubuntu 22.04)
```bash
pip3 install mediapipe opencv-python numpy
```

### Build trong ROS 2 Workspace
```bash
cd ~/workspaces/ur_gz
colcon build --packages-select gesture_turtlebot4
source install/setup.bash
```

---

## 4. Hướng dẫn chạy hệ thống

### Terminal 1: Khởi chạy TurtleBot 4 trong Gazebo
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=depot model:=lite
```

### Terminal 2: Mở khóa bánh xe (Undock)
```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```

### Terminal 3: Chạy hệ thống điều khiển cử chỉ
```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 launch gesture_turtlebot4 gesture_control.launch.py
```

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Package: turtlebot4_emotion
Node: emotion_tb4_node.py
Mục đích:
    - Nhận diện biểu cảm khuôn mặt qua webcam (hoặc file video) thời gian thực bằng DeepFace / OpenCV.
    - Ánh xạ biểu cảm thành quỹ đạo di chuyển của TurtleBot 4 trong mô phỏng Gazebo (Ignition / Classic).
    - Hỗ trợ Undock tự động nếu robot đang đỗ trên trạm sạc.
Tác giả: Robotics Engineer (ROS 2 Humble / Python 3.10)
"""

import collections
import math
import os
import sys
import threading
import time

# Tắt log cảnh báo ồn ào của TensorFlow
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import cv2
import numpy as np

# Nạp TensorFlow trước để DeepFace liên kết đúng backend Keras
try:
    import tensorflow as tf
except Exception:
    pass

# ROS 2 imports
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor

# Create 3 Undock Action
try:
    from irobot_create_msgs.action import Undock
    HAS_UNDOCK_ACTION = True
except ImportError:
    HAS_UNDOCK_ACTION = False

# DeepFace import (xử lý ngoại lệ an toàn nếu chưa cài hoặc lỗi backend)
try:
    from deepface import DeepFace
    HAS_DEEPFACE = True
except Exception as e:
    HAS_DEEPFACE = False


# ==============================================================================
# HẰNG SỐ CẤU HÌNH MẶC ĐỊNH
# ==============================================================================
DEFAULT_ROBOT_NAMESPACE = ''
DEFAULT_VIDEO_SOURCE = '0'
DEFAULT_PUBLISH_RATE = 10.0          # Hz
DEFAULT_CAMERA_FPS = 30.0            # Hz
DEFAULT_WINDOW_SIZE = 5              # Kích thước sliding window bỏ phiếu
DEFAULT_NO_FACE_TIMEOUT = 1.0        # Giây
DEFAULT_MAX_LINEAR_SPEED = 0.2       # m/s (giới hạn an toàn)
DEFAULT_MAX_ANGULAR_SPEED = 1.0      # rad/s (giới hạn an toàn)
DEFAULT_AUTO_UNDOCK = True

# Bảng vận tốc ánh xạ theo đặc tả bài toán (Tiếng Việt không dấu hiển thị chuẩn trên OpenCV)
# | Biểu cảm | Hành động | Vận tốc (linear.x, angular.z, mô tả) |
EMOTION_VELOCITY_MAP = {
    'happy':    (0.15,  0.5,  'Cong trai (Happy)'),
    'sad':      (0.15, -0.5,  'Cong phai (Sad)'),
    'neutral':  (0.00,  0.0,  'Dung tai cho (Neutral)'),
    'angry':    (-0.08, 0.0,  'Lui nhe ve sau (Angry)'),
    'fear':     (0.10,  0.0,  'Tien nhe ve truoc (Fear)'),
    'surprise': (0.00,  0.8,  'Xoay 360 do (Surprise)'),
    'disgust':  (0.00,  0.0,  'Dung (Disgust/Khac)'),
    'unknown':  (0.00,  0.0,  'Dung (Khong xac dinh)'),
}


class EmotionTB4Node(Node):
    """
    ROS 2 Node nhận diện biểu cảm khuôn mặt và điều khiển TurtleBot 4
    """

    def __init__(self):
        super().__init__('emotion_tb4_node')

        # ----------------------------------------------------------------------
        # 1. Khai báo và lấy ROS Parameters
        # ----------------------------------------------------------------------
        dynamic_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('robot_namespace', DEFAULT_ROBOT_NAMESPACE, dynamic_desc)
        self.declare_parameter('video_source', DEFAULT_VIDEO_SOURCE, dynamic_desc)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE, dynamic_desc)
        self.declare_parameter('window_size', DEFAULT_WINDOW_SIZE, dynamic_desc)
        self.declare_parameter('no_face_timeout', DEFAULT_NO_FACE_TIMEOUT, dynamic_desc)
        self.declare_parameter('max_linear_speed', DEFAULT_MAX_LINEAR_SPEED, dynamic_desc)
        self.declare_parameter('max_angular_speed', DEFAULT_MAX_ANGULAR_SPEED, dynamic_desc)
        self.declare_parameter('auto_undock', DEFAULT_AUTO_UNDOCK, dynamic_desc)
        self.declare_parameter('show_gui', True, dynamic_desc)

        self.robot_ns = str(self.get_parameter('robot_namespace').value or '').strip('/')
        self.video_source_param = str(self.get_parameter('video_source').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.no_face_timeout = float(self.get_parameter('no_face_timeout').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.auto_undock = bool(self.get_parameter('auto_undock').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)

        self.get_logger().info('=' * 60)
        self.get_logger().info('   TURTLEBOT 4 EMOTION CONTROLLER NODE KHOI DONG')
        self.get_logger().info(f' * Namespace: "{self.robot_ns}"')
        self.get_logger().info(f' * Nguon video: "{self.video_source_param}"')
        self.get_logger().info(f' * Tan so cmd_vel: {self.publish_rate} Hz')
        self.get_logger().info(f' * Cua so lam muot (N): {self.window_size}')
        self.get_logger().info(f' * Timeout mat khuon mat: {self.no_face_timeout} s')
        self.get_logger().info(f' * Auto Undock: {self.auto_undock}')
        self.get_logger().info('=' * 60)

        # ----------------------------------------------------------------------
        # 2. Thiết lập Publisher cmd_vel
        # ----------------------------------------------------------------------
        cmd_vel_topic = f'/{self.robot_ns}/cmd_vel' if self.robot_ns else '/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.get_logger().info(f'Publishing velocity to topic: {cmd_vel_topic}')

        # ----------------------------------------------------------------------
        # 3. Quản lý trạng thái Robot & Biểu cảm
        # ----------------------------------------------------------------------
        # Trạng thái máy hữu hạn: 'NORMAL' hoặc 'SPINNING_360'
        self.state = 'NORMAL'
        self.spin_start_time = 0.0
        # Xoay 360 độ (2*pi rad) với tốc độ 0.8 rad/s -> Thời gian = 2*pi / 0.8 s
        self.spin_angular_speed = 0.8
        self.spin_duration = (2.0 * math.pi) / self.spin_angular_speed  # ~7.854 s

        # Bộ đệm cửa sổ trượt (Sliding Window) để bỏ phiếu đa số
        self.emotion_history = collections.deque(maxlen=self.window_size)
        self.current_raw_emotion = 'neutral'
        self.current_confidence = 0.0
        self.smoothed_emotion = 'neutral'
        self.current_action_text = 'Dừng tại chỗ'
        self.last_face_timestamp = time.time()
        self.face_detected = False
        self.face_bbox = None  # (x, y, w, h)

        # Vận tốc mục tiêu hiện tại đang gửi
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Phím kiểm tra thủ công (Override bằng bàn phím số 1-6 để test nhanh)
        self.manual_override_emotion = None
        self.manual_override_until = 0.0

        # ----------------------------------------------------------------------
        # 4. Bộ nhận diện khuôn mặt OpenCV Haar Cascade (cực nhanh ~5ms)
        # ----------------------------------------------------------------------
        cascade_path = self._locate_haarcascade()
        if cascade_path and os.path.isfile(cascade_path):
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            self.get_logger().info(f'Đã tải Haar Cascade: {cascade_path}')
        else:
            self.face_cascade = cv2.CascadeClassifier()
            self.get_logger().error('Không tìm thấy tệp haarcascade_frontalface_default.xml!')

        # ----------------------------------------------------------------------
        # 5. Worker Thread nhận diện biểu cảm DeepFace (bất đồng bộ, không nghẽn)
        # ----------------------------------------------------------------------
        if not HAS_DEEPFACE:
            self.get_logger().warn(
                'CHÚ Ý: Chưa tìm thấy thư viện "deepface"!\n'
                '   Hãy chạy: pip install deepface tf-keras\n'
                '   Node sẽ tạm thời hoạt động ở chế độ mô phỏng / phím bấm (1-6) để kiểm tra.'
            )
        self.worker_lock = threading.Lock()
        self.latest_face_crop = None
        self.worker_busy = False
        self.stop_worker = False
        self.worker_thread = threading.Thread(target=self._emotion_worker_loop, daemon=True)
        self.worker_thread.start()

        # ----------------------------------------------------------------------
        # 6. Khởi tạo Camera / Video Capture
        # ----------------------------------------------------------------------
        self.cap = self._init_video_capture(self.video_source_param)
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0.0

        # ----------------------------------------------------------------------
        # 7. Action Client Undock (nếu cần tự động rời dock)
        # ----------------------------------------------------------------------
        self.undock_client = None
        self.undock_done = False
        if HAS_UNDOCK_ACTION and self.auto_undock:
            undock_action_name = f'/{self.robot_ns}/undock' if self.robot_ns else '/undock'
            self.undock_client = ActionClient(self, Undock, undock_action_name)
            # Dùng timer một lần sau 1.5s để gửi goal undock khi các node simulation đã sẵn sàng
            self.undock_timer = self.create_timer(1.5, self._trigger_undock_once)

        # ----------------------------------------------------------------------
        # 8. Timers định thời ROS 2
        # ----------------------------------------------------------------------
        # Timer 1: Chu kỳ đọc camera & hiển thị GUI (~30 Hz)
        self.camera_timer = self.create_timer(1.0 / DEFAULT_CAMERA_FPS, self._camera_callback)
        # Timer 2: Chu kỳ tính toán & gửi vận tốc Twist cố định (10 Hz)
        self.cmd_timer = self.create_timer(1.0 / self.publish_rate, self._cmd_publish_callback)

        self.get_logger().info('He thong da san sang! Nhan "q" tren cua so camera de thoat.')

    # ==========================================================================
    # TÌM TỆP HAAR CASCADE
    # ==========================================================================
    def _locate_haarcascade(self):
        """Tim duong dan tep haarcascade tuong thich ca pip va apt-get opencv"""
        candidates = []
        if hasattr(cv2, 'data') and hasattr(cv2.data, 'haarcascades'):
            candidates.append(os.path.join(cv2.data.haarcascades, 'haarcascade_frontalface_default.xml'))
        candidates.extend([
            '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
            '/usr/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml',
        ])
        for p in candidates:
            if os.path.isfile(p):
                return p
        return None

    # ==========================================================================
    # KHỞI TẠO VIDEO CAPTURE
    # ==========================================================================
    def _init_video_capture(self, source_param):
        """Khoi tao cv2.VideoCapture tu index webcam hoac duong dan file video"""
        if source_param.isdigit():
            dev_idx = int(source_param)
            self.get_logger().info(f'Dang ket noi Webcam phan cung tai /dev/video{dev_idx}...')
            # Dung V4L2 tren Linux de khoi dong camera nhanh va on dinh hon
            cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                # Thu mo lai bang backend mac dinh
                cap = cv2.VideoCapture(dev_idx)
        else:
            self.get_logger().info(f'Dang mo file video nguon: {source_param}')
            cap = cv2.VideoCapture(source_param)

        if not cap.isOpened():
            self.get_logger().error(f'Khong the mo nguon video "{source_param}"!')
        else:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap

    # ==========================================================================
    # ACTION CLIENT: UNDOCK TURTLEBOT 4
    # ==========================================================================
    def _trigger_undock_once(self):
        """Gui yeu cau Undock neu robot dang tren dock"""
        self.undock_timer.cancel()
        if not self.undock_client:
            return

        self.get_logger().info('Kiem tra Action Server /undock...')
        if not self.undock_client.wait_for_server(timeout_sec=4.0):
            self.get_logger().warn('Action server /undock chua san sang (co the robot khong o tren dock). Tiep tuc chay.')
            return

        self.get_logger().info('Gui yeu cau Undock de robot roi dock...')
        goal_msg = Undock.Goal()
        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._undock_response_callback)

    def _undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Yeu cau Undock bi tu choi!')
            return
        self.get_logger().info('Yeu cau Undock da duoc chap nhan, dang cho hoan tat...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._undock_result_callback)

    def _undock_result_callback(self, future):
        result = future.result().result
        self.undock_done = True
        self.get_logger().info(f'Hoan tat Undock! Trang thai is_docked={result.is_docked}. Robot san sang di chuyen.')

    # ==========================================================================
    # WORKER THREAD: NHẬN DIỆN BIỂU CẢM BẤT ĐỒNG BỘ
    # ==========================================================================
    def _emotion_worker_loop(self):
        """
        Luồng nền chuyên chạy model DeepFace mà không làm giật FPS của camera hay trễ 10Hz timer
        """
        while not self.stop_worker:
            face_img = None
            with self.worker_lock:
                if self.latest_face_crop is not None:
                    face_img = self.latest_face_crop.copy()
                    self.latest_face_crop = None
                    self.worker_busy = True
                else:
                    self.worker_busy = False

            if face_img is None:
                time.sleep(0.01)
                continue

            # Thực thi DeepFace
            detected_emotion = 'neutral'
            confidence = 0.0

            if HAS_DEEPFACE:
                try:
                    # Bỏ qua khâu face detector của DeepFace (detector_backend='skip')
                    # vì ta đã dùng Haar Cascade cắt đúng vùng mặt, giúp tốc độ tăng gấp 5-10 lần!
                    results = DeepFace.analyze(
                        img_path=face_img,
                        actions=['emotion'],
                        enforce_detection=False,
                        detector_backend='skip',
                        silent=True
                    )
                    if isinstance(results, list) and len(results) > 0:
                        res = results[0]
                    else:
                        res = results

                    detected_emotion = res.get('dominant_emotion', 'neutral').lower()
                    emotion_scores = res.get('emotion', {})
                    confidence = float(emotion_scores.get(detected_emotion, 0.0))
                except Exception as e:
                    self.get_logger().warn(f'DeepFace analyze warning: {e}')
                    detected_emotion = 'neutral'
                    confidence = 0.0
            else:
                # Giả lập biểu cảm nếu chưa cài DeepFace
                detected_emotion = 'neutral'
                confidence = 88.0

            # Cập nhật kết quả vào Sliding Window
            with self.worker_lock:
                if detected_emotion != self.current_raw_emotion:
                    self.get_logger().info(f'>> Bieu cam moi: {detected_emotion.upper()} ({confidence:.1f}%)')
                self.current_raw_emotion = detected_emotion
                self.current_confidence = confidence
                self.emotion_history.append(detected_emotion)
                self.worker_busy = False

    # ==========================================================================
    # CALLBACK ĐỌC CAMERA VÀ HIỂN THỊ OPENCV GUI (~30 Hz)
    # ==========================================================================
    def _camera_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                # Nếu là video file, tua lại đầu video để lặp vô tận
                if not self.video_source_param.isdigit():
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()

            if not ret or frame is None:
                # Tạo khung hình đen giả lập
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Lật ảnh gương (mirror) khi dùng webcam để người dùng tương tác tự nhiên
        if self.video_source_param.isdigit():
            frame = cv2.flip(frame, 1)

        # ----------------------------------------------------------------------
        # Phát hiện khuôn mặt bằng Haar Cascade
        # ----------------------------------------------------------------------
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(80, 80)
        )

        now = time.time()
        if len(faces) > 0:
            # Chọn khuôn mặt lớn nhất (gần camera nhất)
            largest_face = max(faces, key=lambda b: b[2] * b[3])
            x, y, w, h = largest_face
            self.face_bbox = (x, y, w, h)
            self.face_detected = True
            self.last_face_timestamp = now

            # Mở rộng nhẹ vùng khuôn mặt (padding 10%) để DeepFace nhận diện tốt hơn
            pad_x = int(w * 0.1)
            pad_y = int(h * 0.1)
            y1 = max(0, y - pad_y)
            y2 = min(frame.shape[0], y + h + pad_y)
            x1 = max(0, x - pad_x)
            x2 = min(frame.shape[1], x + w + pad_x)
            face_roi = frame[y1:y2, x1:x2]

            # Gửi vùng mặt sang Worker Thread nếu worker đang rảnh
            with self.worker_lock:
                if not self.worker_busy and face_roi.size > 0:
                    self.latest_face_crop = face_roi
        else:
            self.face_detected = False
            self.face_bbox = None

        # ----------------------------------------------------------------------
        # Tính toán FPS hiển thị
        # ----------------------------------------------------------------------
        self.fps_counter += 1
        elapsed = now - self.fps_start_time
        if elapsed >= 0.5:
            self.current_fps = self.fps_counter / elapsed
            self.fps_counter = 0
            self.fps_start_time = now

        # ----------------------------------------------------------------------
        # Vẽ HUD giao diện người dùng
        # ----------------------------------------------------------------------
        if self.show_gui:
            self._draw_overlay(frame)
            try:
                cv2.imshow('TurtleBot 4 - Facial Emotion Controller', frame)
                key = cv2.waitKey(1) & 0xFF

                # Phím 'q' để thoát an toàn
                if key == ord('q'):
                    self.get_logger().info('Người dùng nhấn "q". Đang dừng robot và thoát...')
                    self._emergency_stop_and_exit()
                    return

                # Phím số 1-6 để ghi đè kiểm tra thủ công nhanh (Manual Override)
                self._handle_manual_key(key)
            except cv2.error as e:
                self.get_logger().warn(f'Giao diện OpenCV HighGUI gặp lỗi ({e}). Tự động chuyển sang chế độ không GUI.')
                self.show_gui = False

    # ==========================================================================
    # VẼ OVERLAY GIAO DIỆN (HUD)
    # ==========================================================================
    def _draw_overlay(self, frame):
        h_img, w_img = frame.shape[:2]

        # 1. Bounding box quanh khuôn mặt
        if self.face_detected and self.face_bbox is not None:
            x, y, w, h = self.face_bbox
            # Màu khung: xanh lá khi bình thường, tím khi đang xoay surprise
            box_color = (180, 0, 220) if self.state == 'SPINNING_360' else (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)

            label = f"{self.smoothed_emotion.upper()} ({self.current_confidence:.0f}%)"
            cv2.putText(frame, label, (x, max(25, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2, cv2.LINE_AA)

        # 2. Bảng điều khiển bán trong suốt ở góc trên bên trái
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (450, 165), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        # Tiêu đề
        cv2.putText(frame, "TURTLEBOT 4 EMOTION CONTROLLER", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 215, 255), 2, cv2.LINE_AA)

        # Trạng thái mặt
        now = time.time()
        time_since_face = now - self.last_face_timestamp
        if self.face_detected:
            status_text = "Mat: Da tim thay"
            status_color = (0, 255, 0)
        elif time_since_face > self.no_face_timeout:
            status_text = f"Mat: Mat ({time_since_face:.1f}s > {self.no_face_timeout}s -> DUNG)"
            status_color = (0, 0, 255)
        else:
            status_text = f"Mat: Tam mat ({time_since_face:.1f}s)"
            status_color = (0, 165, 255)

        cv2.putText(frame, status_text, (20, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, status_color, 1, cv2.LINE_AA)

        # Biểu cảm làm mượt (Majority vote)
        emotion_text = f"Bieu cam (Voting N={self.window_size}): {self.smoothed_emotion.upper()}"
        if self.manual_override_emotion:
            emotion_text += " [MANUAL TEST]"
        cv2.putText(frame, emotion_text, (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)

        # Hành động robot
        cv2.putText(frame, f"Hanh dong: {self.current_action_text}", (20, 104),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)

        # Vận tốc đang gửi
        vel_text = f"cmd_vel: Linear.x={self.current_linear_x:+.2f} m/s | Angular.z={self.current_angular_z:+.2f} rad/s"
        cv2.putText(frame, vel_text, (20, 128),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        # Trạng thái máy & FPS
        sys_info = f"Trang thai: {self.state} | FPS: {self.current_fps:.1f} | Ns: '{self.robot_ns}'"
        cv2.putText(frame, sys_info, (20, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 170, 170), 1, cv2.LINE_AA)

        # 3. Hướng dẫn phím ở chân khung hình
        guide_text = "Nhan 'q' de thoat | Phim 1-6 de test (1:Happy 2:Sad 3:Neutral 4:Angry 5:Surprise 6:Fear 0:Auto)"
        cv2.putText(frame, guide_text, (15, h_img - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

    # ==========================================================================
    # PHÍM TẮT TEST THỦ CÔNG
    # ==========================================================================
    def _handle_manual_key(self, key):
        """Cho phép lập trình viên bấm số 1-6 để kiểm tra các hành động của robot"""
        mapping = {
            ord('1'): 'happy',
            ord('2'): 'sad',
            ord('3'): 'neutral',
            ord('4'): 'angry',
            ord('5'): 'surprise',
            ord('6'): 'fear',
        }
        if key in mapping:
            self.manual_override_emotion = mapping[key]
            self.manual_override_until = time.time() + 4.0  # Giữ biểu cảm test trong 4 giây
            self.get_logger().info(f'[Manual Test] Kích hoạt biểu cảm thử nghiệm: {self.manual_override_emotion.upper()}')
        elif key == ord('0'):
            self.manual_override_emotion = None
            self.get_logger().info('[Manual Test] Đã tắt chế độ ghi đè, trở về camera tự động.')

    # ==========================================================================
    # CALLBACK ĐIỀU KHIỂN ROBOT VÀ PUBLISH CMD_VEL CỐ ĐỊNH 10 HZ
    # ==========================================================================
    def _cmd_publish_callback(self):
        now = time.time()

        # 1. Kiểm tra chế độ manual test override
        if self.manual_override_emotion and now < self.manual_override_until:
            active_emotion = self.manual_override_emotion
        else:
            self.manual_override_emotion = None
            # Bỏ phiếu đa số (Majority Voting) trên cửa sổ trượt N=window_size
            with self.worker_lock:
                if len(self.emotion_history) > 0:
                    counts = collections.Counter(self.emotion_history)
                    active_emotion = counts.most_common(1)[0][0]
                else:
                    active_emotion = 'neutral'

        # 2. Xử lý timeout mất khuôn mặt (> 1.0 giây thì bắt buộc dừng)
        time_since_face = now - self.last_face_timestamp
        if time_since_face > self.no_face_timeout and not self.manual_override_emotion:
            # Khi mất khuôn mặt, đặt biểu cảm thành neutral
            active_emotion = 'neutral'
            face_lost = True
        else:
            face_lost = False

        self.smoothed_emotion = active_emotion

        # ----------------------------------------------------------------------
        # 3. Máy trạng thái hữu hạn (Finite State Machine)
        # ----------------------------------------------------------------------
        target_linear = 0.0
        target_angular = 0.0

        if self.state == 'SPINNING_360':
            # Đang trong hành động xoay 360 độ của biểu cảm 'surprise'
            # Trong lúc này, bỏ qua mọi biểu cảm mới cho đến khi hoàn tất xoay!
            elapsed_spin = now - self.spin_start_time
            if elapsed_spin < self.spin_duration:
                target_linear = 0.0
                target_angular = self.spin_angular_speed
                remain = self.spin_duration - elapsed_spin
                self.current_action_text = f"Xoay 360 do (Surprise: con {remain:.1f}s)"
            else:
                # Đã hoàn thành một vòng xoay 360 độ -> Dừng lại và trở về trạng thái NORMAL
                self.state = 'NORMAL'
                target_linear = 0.0
                target_angular = 0.0
                self.current_action_text = "Hoan tat xoay 360 do -> Dung"
                # Xóa sạch lịch sử để không bị kích hoạt xoay liên tiếp ngay lập tức
                with self.worker_lock:
                    self.emotion_history.clear()
                self.get_logger().info('Hoan tat xoay 360 do an toan!')

        elif self.state == 'NORMAL':
            if face_lost:
                target_linear = 0.0
                target_angular = 0.0
                self.current_action_text = "Mat khuon mat (> 1.0s) -> Dung robot"

            elif active_emotion == 'surprise':
                # Chuyển trạng thái sang SPINNING_360 (Non-blocking)
                self.state = 'SPINNING_360'
                self.spin_start_time = now
                target_linear = 0.0
                target_angular = self.spin_angular_speed
                self.current_action_text = f"Bat dau xoay 360 do ({self.spin_duration:.1f}s)"
                self.get_logger().info(
                    f'Phat hien SURPRISE! Bat dau xoay 360 do tai cho trong {self.spin_duration:.2f} giay.'
                )

            elif active_emotion in EMOTION_VELOCITY_MAP:
                vel = EMOTION_VELOCITY_MAP[active_emotion]
                target_linear = vel[0]
                target_angular = vel[1]
                self.current_action_text = vel[2]

            else:
                # Nhãn khác (disgust, unknown...) -> Coi như neutral
                target_linear = 0.0
                target_angular = 0.0
                self.current_action_text = f"Nhan '{active_emotion}' -> Dung (Neutral)"

        # ----------------------------------------------------------------------
        # 4. Giới hạn vận tốc an toàn (Clamping)
        # ----------------------------------------------------------------------
        target_linear = max(-self.max_linear_speed, min(self.max_linear_speed, target_linear))
        target_angular = max(-self.max_angular_speed, min(self.max_angular_speed, target_angular))

        self.current_linear_x = target_linear
        self.current_angular_z = target_angular

        # ----------------------------------------------------------------------
        # 5. Gửi message Twist lên topic /cmd_vel
        # ----------------------------------------------------------------------
        msg = Twist()
        msg.linear.x = float(target_linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(target_angular)

        self.cmd_vel_pub.publish(msg)

    # ==========================================================================
    # DỪNG KHẨN CẤP VÀ THOÁT SẠCH
    # ==========================================================================
    def _emergency_stop_and_exit(self):
        """Gui lenh van toc 0 va giai phong moi tai nguyen"""
        self.get_logger().info('Gui Twist = 0 de robot dung han...')
        stop_msg = Twist()
        # Gui lien tiep 3 lan de dam bao robot nhan duoc lenh dung
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.02)

        self.stop_worker = True
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=0.5)

        if self.cap.isOpened():
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = EmotionTB4Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Bat tin hieu KeyboardInterrupt (Ctrl+C). Dang tat...')
    finally:
        node._emergency_stop_and_exit()


if __name__ == '__main__':
    main()

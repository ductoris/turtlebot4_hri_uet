#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_recognition_node.py:
Node nhan dien cu chi tay thoi gian thuc qua webcam bang MediaPipe Tasks API.
Publish nhan cu chi len /detected_gesture (std_msgs/msg/String).
Subscribe /robot_state de hien thi HUD dong bo.
Thuat toan nhan dien Wave: Yeu cau ban tay mo Open_Palm kem chuoi 3 nhip dao dong ro ret.
Toan bo text HUD va log tren terminal su dung TIENG VIET KHONG DAU.
"""

import os
import cv2
import time
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# 21 Khop xuong ban tay theo MediaPipe
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # Ngon cai
    (0, 5), (5, 6), (6, 7), (7, 8),        # Ngon tro
    (5, 9), (9, 10), (10, 11), (11, 12),   # Ngon giua
    (9, 13), (13, 14), (14, 15), (15, 16), # Ngon ap ut
    (13, 17), (17, 18), (18, 19), (19, 20),# Ngon ut
    (0, 17)                                # Day long ban tay
]


class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')

        # --- KHAI BAO THAM SO (ROS PARAMETERS) ---
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.68)
        self.declare_parameter('confirmation_frames', 4)
        self.declare_parameter('gesture_cooldown_sec', 1.0)
        self.declare_parameter('wave_min_amplitude', 0.10)
        self.declare_parameter('wave_min_reversals', 3)
        self.declare_parameter('wave_window_sec', 1.4)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('flip_camera', True)
        self.declare_parameter('model_path', '')

        self.camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.confidence_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.confirmation_frames = self.get_parameter('confirmation_frames').get_parameter_value().integer_value
        self.gesture_cooldown = self.get_parameter('gesture_cooldown_sec').get_parameter_value().double_value
        self.wave_min_amp = self.get_parameter('wave_min_amplitude').get_parameter_value().double_value
        self.wave_min_rev = self.get_parameter('wave_min_reversals').get_parameter_value().integer_value
        self.wave_window = self.get_parameter('wave_window_sec').get_parameter_value().double_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value
        self.flip_cam = self.get_parameter('flip_camera').get_parameter_value().bool_value
        custom_model = self.get_parameter('model_path').get_parameter_value().string_value

        # --- NAP MODEL MEDIAPIPE GESTURE RECOGNIZER ---
        self.model_path = self._resolve_model_path(custom_model)
        self.get_logger().info(f"Su dung mo hinh MediaPipe: {self.model_path}")

        try:
            base_options = python.BaseOptions(model_asset_path=self.model_path)
            options = vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.IMAGE,
                num_hands=1,
                min_hand_detection_confidence=0.5,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.recognizer = vision.GestureRecognizer.create_from_options(options)
            self.get_logger().info("Khoi tao MediaPipe Gesture Recognizer thanh cong!")
        except Exception as e:
            self.get_logger().error(f"Loi khoi tao MediaPipe: {str(e)}")
            raise e

        # --- KHOI TAO WEBCAM ---
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            self.get_logger().error(f"Khong the mo webcam /dev/video{self.camera_index}!")
            raise RuntimeError(f"Cannot open camera index {self.camera_index}")

        # --- PUBLISHER & SUBSCRIBER ---
        self.gesture_pub = self.create_publisher(String, '/detected_gesture', 10)
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.robot_state_callback, 10
        )
        self.current_robot_state = "IDLE (Cho lenh)"

        # --- BIEN TRANG THAI XU LY CU CHI ---
        self.hand_pos_history = deque()
        self.last_hand_seen_time = 0.0

        self.candidate_gesture = "None"
        self.candidate_count = 0
        self.confirmed_gesture = "None"
        self.confirmed_score = 0.0

        # Latch nhe cho Wave (0.4s) de tranh flicker ma khong bi dinh cu chi
        self.wave_latch_until = 0.0

        self.last_published_gesture = "None"
        self.fps = 0.0
        self.frame_count = 0
        self.fps_prev_time = time.time()

        # Timer xu ly nhan dien 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("gesture_recognition_node da san sang!")

    def _resolve_model_path(self, custom_path):
        if custom_path and os.path.isfile(custom_path):
            return custom_path
        try:
            pkg_share = get_package_share_directory('gesture_turtlebot4')
            share_model = os.path.join(pkg_share, 'models', 'gesture_recognizer.task')
            if os.path.isfile(share_model):
                return share_model
        except Exception:
            pass

        source_model = os.path.expanduser(
            '~/workspaces/ur_gz/src/gesture_turtlebot4/models/gesture_recognizer.task'
        )
        if os.path.isfile(source_model):
            return source_model

        raise FileNotFoundError("Khong tim thay file model 'gesture_recognizer.task'!")

    def robot_state_callback(self, msg: String):
        self.current_robot_state = msg.data

    def detect_wave(self, hand_x, current_time):
        """
        Phat hien Wave chat che:
        - Phai co bien do dao dong toi thieu wave_min_amplitude (mac dinh 0.10 = 10% man hinh).
        - Phai co chuoi toi thieu 3 lan doi chieu dao dong (reversals >= 3).
        - Nguong loc rung nhe 0.025 de tranh nham voi cu dong di tay thong thuong.
        """
        self.hand_pos_history.append((current_time, hand_x))

        while self.hand_pos_history and (current_time - self.hand_pos_history[0][0]) > self.wave_window:
            self.hand_pos_history.popleft()

        if len(self.hand_pos_history) < 8:
            return False, 0.0, 0

        xs = [pt[1] for pt in self.hand_pos_history]
        amplitude = max(xs) - min(xs)

        if amplitude < self.wave_min_amp:
            return False, amplitude, 0

        reversals = 0
        last_extrema_x = xs[0]
        looking_for_max = None
        threshold_step = 0.025

        for i in range(1, len(xs)):
            diff = xs[i] - last_extrema_x
            if looking_for_max is None:
                if diff > threshold_step:
                    looking_for_max = True
                    last_extrema_x = xs[i]
                elif diff < -threshold_step:
                    looking_for_max = False
                    last_extrema_x = xs[i]
            elif looking_for_max:
                if xs[i] > last_extrema_x:
                    last_extrema_x = xs[i]
                elif diff < -threshold_step:
                    reversals += 1
                    looking_for_max = False
                    last_extrema_x = xs[i]
            else:
                if xs[i] < last_extrema_x:
                    last_extrema_x = xs[i]
                elif diff > threshold_step:
                    reversals += 1
                    looking_for_max = True
                    last_extrema_x = xs[i]

        is_wave = (amplitude >= self.wave_min_amp) and (reversals >= self.wave_min_rev)
        return is_wave, amplitude, reversals

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        current_time = time.time()
        if self.flip_cam:
            frame = cv2.flip(frame, 1)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        detection_result = self.recognizer.recognize(mp_image)

        raw_gesture = "None"
        raw_score = 0.0
        hand_landmarks = None
        is_wave = False
        wave_amp = 0.0
        wave_rev = 0

        if detection_result.gestures and len(detection_result.gestures) > 0:
            top_gesture = detection_result.gestures[0][0]
            hand_landmarks = detection_result.hand_landmarks[0] if detection_result.hand_landmarks else None
            self.last_hand_seen_time = current_time

            if hand_landmarks:
                hand_x = (hand_landmarks[0].x + hand_landmarks[9].x) / 2.0
                is_wave, wave_amp, wave_rev = self.detect_wave(hand_x, current_time)

            if top_gesture.score >= self.confidence_thresh:
                candidate = top_gesture.category_name
                raw_score = top_gesture.score

                # BAT BUOC: Chi khi ban tay dang mo Open_Palm VA co chuoi 3 nhip dao dong moi tinh la Wave
                if candidate == "Open_Palm":
                    if is_wave:
                        raw_gesture = "Wave"
                    else:
                        raw_gesture = "Open_Palm"
                else:
                    # Neu la cu chi khac (Closed_Fist, Pointing_Up, Victory, Thumb_Up), giu nguyen cu chi do!
                    raw_gesture = candidate
            else:
                raw_gesture = "None"
        else:
            raw_gesture = "None"
            if (current_time - self.last_hand_seen_time) > 0.4:
                self.hand_pos_history.clear()

        # Giu nhan Wave ngan (0.4s) sau khi dat du 3 nhip dao dong
        if is_wave and raw_gesture == "Wave":
            self.wave_latch_until = current_time + 0.4

        if current_time < self.wave_latch_until:
            raw_gesture = "Wave"
            raw_score = 0.95

        # --- BO LOC KHUNG HINH LIEN TIEP (DEBOUNCE) ---
        if raw_gesture == "Wave":
            self.confirmed_gesture = "Wave"
            self.confirmed_score = raw_score
            self.candidate_gesture = "Wave"
            self.candidate_count = self.confirmation_frames
        else:
            if raw_gesture == self.candidate_gesture:
                self.candidate_count += 1
            else:
                self.candidate_gesture = raw_gesture
                self.candidate_count = 1

            if self.candidate_count >= self.confirmation_frames:
                self.confirmed_gesture = self.candidate_gesture
                self.confirmed_score = raw_score
            elif raw_gesture == "None" and self.candidate_count >= 3:
                self.confirmed_gesture = "None"
                self.confirmed_score = 0.0

        # Publish dinh ky len /detected_gesture
        msg = String()
        msg.data = self.confirmed_gesture
        self.gesture_pub.publish(msg)

        if self.confirmed_gesture != self.last_published_gesture:
            if self.confirmed_gesture != "None":
                self.get_logger().info(
                    f"[GESTURE] -> {self.confirmed_gesture} ({self.confirmed_score*100:.1f}%)"
                )
            else:
                self.get_logger().info("[GESTURE] Mat ban tay (None)")
            self.last_published_gesture = self.confirmed_gesture

        # Tinh FPS
        self.frame_count += 1
        elapsed = current_time - self.fps_prev_time
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_prev_time = current_time

        # Ve giao dien (HUD)
        if self.show_gui:
            self.draw_hud(frame, hand_landmarks, raw_gesture, is_wave, wave_amp, wave_rev)
            cv2.imshow("TurtleBot 4 Gesture Recognition", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info("Nhan 'q' - Dung node gesture_recognition_node.")
                rclpy.shutdown()

    def draw_hud(self, frame, hand_landmarks, raw_gesture, is_wave, wave_amp, wave_rev):
        h, w, _ = frame.shape

        if hand_landmarks:
            coords = []
            for lm in hand_landmarks:
                cx, cy = int(lm.x * w), int(lm.y * h)
                coords.append((cx, cy))
                cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)

            for p1, p2 in HAND_CONNECTIONS:
                if p1 < len(coords) and p2 < len(coords):
                    cv2.line(frame, coords[p1], coords[p2], (0, 200, 0), 2)

            center_x = int((hand_landmarks[0].x + hand_landmarks[9].x) / 2.0 * w)
            center_y = int((hand_landmarks[0].y + hand_landmarks[9].y) / 2.0 * h)
            cv2.circle(frame, (center_x, center_y), 8, (0, 0, 255), -1)

        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 88), (20, 20, 20), -1)
        cv2.rectangle(overlay, (0, h - 32), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        if self.confirmed_gesture == "Wave":
            gesture_color = (0, 255, 0)
        elif self.confirmed_gesture in ["Open_Palm", "Closed_Fist"]:
            gesture_color = (0, 0, 255)
        elif self.confirmed_gesture in ["Pointing_Up", "Victory"]:
            gesture_color = (255, 200, 0)
        elif self.confirmed_gesture == "Thumb_Up":
            gesture_color = (0, 255, 0)
        else:
            gesture_color = (180, 180, 180)

        status_text = f"CU CHI: {self.confirmed_gesture}"
        if self.confirmed_gesture != "None":
            status_text += f" ({self.confirmed_score*100:.1f}%)"
        if self.confirmed_gesture == "Wave":
            status_text += " [VAY CHAO!]"
        cv2.putText(frame, status_text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.85, gesture_color, 2)

        robot_text = f"ROBOT: {self.current_robot_state}"
        cv2.putText(frame, robot_text, (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        wave_debug_color = (0, 255, 0) if is_wave else (0, 255, 255)
        wave_text = f"Wave Debug -> Bien do: {wave_amp:.2f}/{self.wave_min_amp:.2f} | Dao dong: {wave_rev}/{self.wave_min_rev} nhip"
        cv2.putText(frame, wave_text, (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, wave_debug_color, 1)

        cv2.putText(frame, f"FPS: {self.fps:.1f}", (15, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, "Nhan 'q' de thoat", (w - 170, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if hasattr(self, 'recognizer'):
            try:
                self.recognizer.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

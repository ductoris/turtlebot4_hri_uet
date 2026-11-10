#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_dashboard_gui.py:
Giao diện đồ họa giám sát cao cấp (PyQt5) hiển thị trực quan các từ ngữ microphone thu được,
kèm trạng thái phân tích ngữ nghĩa NLU, tiến độ điều hướng thực tế của TurtleBot 4,
và các nút điều khiển thu âm / gửi lệnh nhanh.
"""

import sys
import os
import time
import json
import wave
import subprocess
import threading

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QTextEdit, QFrame, QGridLayout, QProgressBar
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosSignalBridge(QObject):
    voice_received = pyqtSignal(str)
    status_received = pyqtSignal(str)
    model_loaded = pyqtSignal(bool)
    mic_status_signal = pyqtSignal(str, str) # text, color

class RosListenerNode(Node):
    def __init__(self, bridge: RosSignalBridge):
        super().__init__('voice_dashboard_gui_node')
        self.bridge = bridge

        self.sub_voice = self.create_subscription(
            String,
            '/voice_command_text',
            self.on_voice,
            10
        )

        self.sub_status = self.create_subscription(
            String,
            '/voice_nav/status',
            self.on_status,
            10
        )

        self.pub_voice = self.create_publisher(String, '/voice_command_text', 10)

    def on_voice(self, msg: String):
        text = msg.data.strip()
        self.bridge.voice_received.emit(text)

    def on_status(self, msg: String):
        status = msg.data.strip()
        self.bridge.status_received.emit(status)

    def send_command(self, text: str):
        if text.strip():
            msg = String()
            msg.data = text.strip()
            self.pub_voice.publish(msg)

class VoiceDashboardWindow(QMainWindow):
    def __init__(self, ros_node: RosListenerNode, bridge: RosSignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.bridge = bridge
        self.is_recording = False

        self.vosk_model = None
        self.vosk_model_path = '/home/ductri/VSCode/hri/models/vosk-model-small-vn-0.4'

        self.init_ui()

        # Kết nối tín hiệu PyQt
        self.bridge.voice_received.connect(self.display_voice_text)
        self.bridge.status_received.connect(self.display_status_text)
        self.bridge.model_loaded.connect(self.on_model_loaded)
        self.bridge.mic_status_signal.connect(self.update_mic_status_ui)

        # Preload Vosk Model trong background thread
        threading.Thread(target=self.preload_vosk, daemon=True).start()

    def preload_vosk(self):
        """Nạp sẵn mô hình Vosk vào bộ nhớ để khi bấm micro nhận diện tức thì."""
        try:
            if os.path.exists(self.vosk_model_path):
                import vosk
                self.vosk_model = vosk.Model(self.vosk_model_path)
                self.bridge.model_loaded.emit(True)
            else:
                self.bridge.model_loaded.emit(False)
        except Exception:
            self.bridge.model_loaded.emit(False)

    def on_model_loaded(self, success):
        if success:
            self.model_badge.setText("🧠 VOSK TIẾNG VIỆT: SẴN SÀNG")
            self.model_badge.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; font-size: 11px; border-radius: 12px; padding: 4px 10px; border: 1px solid #059669;")
            self.log("Đã tải xong mô hình nhận dạng giọng nói tiếng Việt Vosk (Offline).")
        else:
            self.model_badge.setText("⚠️ VOSK: CHƯA NẠP ĐƯỢC")
            self.model_badge.setStyleSheet("background-color: #7F1D1D; color: #FCA5A5; font-weight: bold; font-size: 11px; border-radius: 12px; padding: 4px 10px; border: 1px solid #DC2626;")

    def init_ui(self):
        self.setWindowTitle("🐢 TurtleBot 4 - Bảng Giám Sát Lời Nói & Điều Hướng")
        self.resize(920, 750)
        self.setMinimumSize(820, 640)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0B1120;
            }
            QLabel {
                color: #E2E8F0;
                font-family: 'Segoe UI', Arial, sans-serif;
            }
            QFrame.card {
                background-color: #1E293B;
                border-radius: 12px;
                border: 1px solid #334155;
            }
            QLineEdit {
                background-color: #0F172A;
                border: 1px solid #475569;
                border-radius: 6px;
                padding: 10px 14px;
                color: #F8FAFC;
                font-size: 13px;
            }
            QLineEdit:focus {
                border: 1px solid #38BDF8;
            }
            QPushButton.btn-primary {
                background-color: #0284C7;
                color: #FFFFFF;
                font-weight: bold;
                border-radius: 6px;
                padding: 10px 18px;
                font-size: 13px;
            }
            QPushButton.btn-primary:hover {
                background-color: #0369A1;
            }
            QPushButton.btn-mic {
                background-color: #2563EB;
                color: #FFFFFF;
                font-weight: bold;
                border-radius: 8px;
                padding: 14px 24px;
                font-size: 14px;
            }
            QPushButton.btn-mic:hover {
                background-color: #1D4ED8;
            }
            QPushButton.btn-quick {
                background-color: #334155;
                color: #F1F5F9;
                font-weight: 600;
                border-radius: 6px;
                padding: 8px 12px;
                font-size: 12px;
                border: 1px solid #475569;
            }
            QPushButton.btn-quick:hover {
                background-color: #475569;
                border-color: #38BDF8;
            }
            QPushButton.btn-stop {
                background-color: #991B1B;
                color: #FFFFFF;
                font-weight: bold;
                border-radius: 6px;
                padding: 8px 14px;
                font-size: 12px;
            }
            QPushButton.btn-stop:hover {
                background-color: #7F1D1D;
            }
            QTextEdit {
                background-color: #030712;
                border: 1px solid #1F2937;
                border-radius: 6px;
                color: #94A3B8;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 12px;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(18, 14, 18, 14)
        main_layout.setSpacing(12)

        # 1. HEADER BAR
        header_layout = QHBoxLayout()
        title_label = QLabel("🐢 TURTLEBOT 4 - HỆ THỐNG GIÁM SÁT LỜI NÓI & ĐIỀU HƯỚNG")
        title_label.setFont(QFont("Arial", 13, QFont.Bold))
        title_label.setStyleSheet("color: #38BDF8;")
        header_layout.addWidget(title_label)

        header_layout.addStretch()

        self.model_badge = QLabel("⏳ ĐANG TẢI VOSK...")
        self.model_badge.setStyleSheet("background-color: #1E293B; color: #94A3B8; font-weight: bold; font-size: 11px; border-radius: 12px; padding: 4px 10px; border: 1px solid #475569;")
        header_layout.addWidget(self.model_badge)

        self.conn_badge = QLabel("🟢 ROS 2 KẾT NỐI")
        self.conn_badge.setStyleSheet("background-color: #064E3B; color: #34D399; font-weight: bold; font-size: 11px; border-radius: 12px; padding: 4px 10px; border: 1px solid #059669;")
        header_layout.addWidget(self.conn_badge)
        main_layout.addLayout(header_layout)

        # 2. KHUNG HIỂN THỊ CHÍNH: LỜI NÓI NHẬN ĐƯỢC TỪ MICROPHONE
        voice_card = QFrame()
        voice_card.setProperty("class", "card")
        voice_card.setStyleSheet("""
            background-color: #1E293B;
            border-radius: 12px;
            border: 2px solid #38BDF8;
            padding: 14px;
        """)
        voice_vbox = QVBoxLayout(voice_card)
        voice_vbox.setSpacing(8)

        card_top = QHBoxLayout()
        card_title = QLabel("🎙️ TỪ NGỮ THU ĐƯỢC TỪ MICROPHONE (RECOGNIZED SPEECH):")
        card_title.setFont(QFont("Arial", 11, QFont.Bold))
        card_title.setStyleSheet("color: #7DD3FC; font-size: 12px;")
        card_top.addWidget(card_title)

        card_top.addStretch()

        self.lbl_mic_state = QLabel("🟢 Sẵn sàng")
        self.lbl_mic_state.setFont(QFont("Arial", 10, QFont.Bold))
        self.lbl_mic_state.setStyleSheet("color: #34D399; font-weight: bold;")
        card_top.addWidget(self.lbl_mic_state)
        voice_vbox.addLayout(card_top)

        # Hộp hiển thị câu nói nhận dạng được
        self.lbl_voice_text = QLabel("(Chưa có âm thanh. Bấm nút '🎤 BẤM ĐỂ NÓI' hoặc nói vào micro...)")
        self.lbl_voice_text.setFont(QFont("Segoe UI", 18, QFont.Bold))
        self.lbl_voice_text.setAlignment(Qt.AlignCenter)
        self.lbl_voice_text.setWordWrap(True)
        self.lbl_voice_text.setStyleSheet("""
            background-color: #0F172A;
            color: #F8FAFC;
            border-radius: 8px;
            border: 1px solid #334155;
            padding: 18px 14px;
            min-height: 60px;
        """)
        voice_vbox.addWidget(self.lbl_voice_text)

        # Chi tiết phân tích NLU
        self.lbl_nlu_detail = QLabel("🎯 Phân tích NLU: Chờ lệnh giọng nói...")
        self.lbl_nlu_detail.setFont(QFont("Segoe UI", 11))
        self.lbl_nlu_detail.setStyleSheet("color: #94A3B8; padding-left: 4px;")
        voice_vbox.addWidget(self.lbl_nlu_detail)

        # Thanh điều khiển micro
        mic_bar = QHBoxLayout()
        self.btn_mic = QPushButton("🎤 BẤM ĐỂ NÓI (THU ÂM 4 GIÂY)")
        self.btn_mic.setProperty("class", "btn-mic")
        self.btn_mic.setCursor(Qt.PointingHandCursor)
        self.btn_mic.clicked.connect(self.record_mic_action)
        mic_bar.addWidget(self.btn_mic)

        self.btn_stop = QPushButton("🛑 DỪNG ROBOT")
        self.btn_stop.setProperty("class", "btn-stop")
        self.btn_stop.setCursor(Qt.PointingHandCursor)
        self.btn_stop.clicked.connect(lambda: self.ros_node.send_command("dừng lại"))
        mic_bar.addWidget(self.btn_stop)

        mic_bar.addStretch()
        voice_vbox.addLayout(mic_bar)
        main_layout.addWidget(voice_card)

        # 3. KHUNG TRẠNG THÁI NAV2 & ROBOT
        status_card = QFrame()
        status_card.setStyleSheet("""
            background-color: #1E293B;
            border-radius: 10px;
            border: 1px solid #334155;
            padding: 10px 14px;
        """)
        status_vbox = QVBoxLayout(status_card)
        status_vbox.setSpacing(6)

        status_title = QLabel("🤖 TRẠNG THÁI TIẾN ĐỘ ĐIỀU HƯỚNG ROBOT (NAV2 PROGRESS):")
        status_title.setFont(QFont("Arial", 10, QFont.Bold))
        status_title.setStyleSheet("color: #5EEAD4; font-size: 11px;")
        status_vbox.addWidget(status_title)

        self.lbl_nav_status = QLabel("🟢 Sẵn sàng nhận lệnh điều hướng mới...")
        self.lbl_nav_status.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self.lbl_nav_status.setStyleSheet("""
            background-color: #134E4A;
            color: #2DD4BF;
            border-radius: 6px;
            padding: 10px 14px;
            border: 1px solid #14B8A6;
        """)
        status_vbox.addWidget(self.lbl_nav_status)
        main_layout.addWidget(status_card)

        # 4. THANH NÚT NHẤN GỬI LỆNH NHANH (QUICK ACCESS BUTTONS)
        quick_card = QFrame()
        quick_card.setStyleSheet("""
            background-color: #1E293B;
            border-radius: 8px;
            border: 1px solid #334155;
            padding: 8px 12px;
        """)
        quick_vbox = QVBoxLayout(quick_card)
        quick_vbox.setSpacing(6)

        quick_title = QLabel("⚡ GỬI LỆNH NHANH (TEST KHÔNG CẦN MICROPHONE):")
        quick_title.setFont(QFont("Arial", 10, QFont.Bold))
        quick_title.setStyleSheet("color: #FBBF24; font-size: 10px;")
        quick_vbox.addWidget(quick_title)

        btn_grid = QHBoxLayout()
        btn_a = QPushButton("🏫 Phòng học (A)")
        btn_a.setProperty("class", "btn-quick")
        btn_a.clicked.connect(lambda: self.ros_node.send_command("Đi đến phòng học"))
        btn_grid.addWidget(btn_a)

        btn_b = QPushButton("🔬 Phòng thí nghiệm (B)")
        btn_b.setProperty("class", "btn-quick")
        btn_b.clicked.connect(lambda: self.ros_node.send_command("Đi đến phòng thí nghiệm"))
        btn_grid.addWidget(btn_b)

        btn_c = QPushButton("🤝 Phòng họp (C)")
        btn_c.setProperty("class", "btn-quick")
        btn_c.clicked.connect(lambda: self.ros_node.send_command("Đi đến phòng họp"))
        btn_grid.addWidget(btn_c)

        btn_dock = QPushButton("⚡ Trạm sạc (Dock)")
        btn_dock.setProperty("class", "btn-quick")
        btn_dock.clicked.connect(lambda: self.ros_node.send_command("Về trạm sạc"))
        btn_grid.addWidget(btn_dock)

        quick_vbox.addLayout(btn_grid)
        main_layout.addWidget(quick_card)

        # 5. THANH NHẬP LỆNH BÀN PHÍM
        input_card = QFrame()
        input_card.setStyleSheet("""
            background-color: #1E293B;
            border-radius: 8px;
            border: 1px solid #334155;
            padding: 6px 12px;
        """)
        input_hbox = QHBoxLayout(input_card)
        input_hbox.setSpacing(10)

        input_lbl = QLabel("Nhập lệnh:")
        input_lbl.setStyleSheet("font-weight: bold; color: #CBD5E1;")
        input_hbox.addWidget(input_lbl)

        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("Ví dụ: học, đi đến phòng học, di chuyển từ khu vực A đến B...")
        self.cmd_input.returnPressed.connect(self.send_text_action)
        input_hbox.addWidget(self.cmd_input)

        btn_send = QPushButton("Gửi Lệnh")
        btn_send.setProperty("class", "btn-primary")
        btn_send.setCursor(Qt.PointingHandCursor)
        btn_send.clicked.connect(self.send_text_action)
        input_hbox.addWidget(btn_send)

        main_layout.addWidget(input_card)

        # 6. LỊCH SỬ CÂU LỆNH (HISTORY LOG)
        log_vbox = QVBoxLayout()
        log_vbox.setSpacing(4)
        log_title = QLabel("📜 NHẬT KÝ THU ÂM VÀ TIẾN ĐỘ THỰC THI (LOG):")
        log_title.setStyleSheet("font-weight: bold; font-size: 11px; color: #94A3B8;")
        log_vbox.addWidget(log_title)

        self.log_edit = QTextEdit()
        self.log_edit.setReadOnly(True)
        log_vbox.addWidget(self.log_edit)

        main_layout.addLayout(log_vbox)

        self.log("Khởi động Voice Dashboard GUI thành công.")
        self.log("Đang lắng nghe: /voice_command_text và /voice_nav/status")

    def log(self, text: str):
        t_str = time.strftime("[%H:%M:%S] ")
        self.log_edit.append(f"{t_str}{text}")

    def update_mic_status_ui(self, text: str, color: str):
        self.lbl_mic_state.setText(text)
        self.lbl_mic_state.setStyleSheet(f"color: {color}; font-weight: bold;")

    def display_voice_text(self, text: str):
        """Cập nhật hiển thị từ ngữ vừa nhận diện được từ micro."""
        self.lbl_voice_text.setText(f'"{text}"')
        self.lbl_voice_text.setStyleSheet("""
            background-color: #082F49;
            color: #38BDF8;
            border-radius: 8px;
            border: 2px solid #0284C7;
            padding: 18px 14px;
            min-height: 60px;
        """)
        t_now = time.strftime("%H:%M:%S")
        self.lbl_nlu_detail.setText(f"🎯 Lời nói nhận lúc {t_now}: \"{text}\" -> Đang điều hướng qua Nav2...")
        self.log(f"🎙️ NHẬN ĐƯỢC TỪ MICRO: \"{text}\"")

    def display_status_text(self, status: str):
        """Cập nhật trạng thái điều hướng của robot."""
        self.lbl_nav_status.setText(status)
        if "Lỗi" in status or "Thất bại" in status or "từ chối" in status:
            self.lbl_nav_status.setStyleSheet("""
                background-color: #7F1D1D;
                color: #F87171;
                border-radius: 6px;
                padding: 10px 14px;
                border: 1px solid #DC2626;
            """)
        elif "Đã đến" in status or "thành công" in status:
            self.lbl_nav_status.setStyleSheet("""
                background-color: #064E3B;
                color: #34D399;
                border-radius: 6px;
                padding: 10px 14px;
                border: 1px solid #10B981;
            """)
        elif "Đang di chuyển" in status or "chấp nhận" in status:
            self.lbl_nav_status.setStyleSheet("""
                background-color: #78350F;
                color: #FBBF24;
                border-radius: 6px;
                padding: 10px 14px;
                border: 1px solid #D97706;
            """)
        elif "chờ Nav2" in status or "kết nối" in status:
            self.lbl_nav_status.setStyleSheet("""
                background-color: #1E1B4B;
                color: #A5B4FC;
                border-radius: 6px;
                padding: 10px 14px;
                border: 1px solid #6366F1;
            """)
        self.log(f"🤖 TRẠNG THÁI: {status}")

    def send_text_action(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.ros_node.send_command(cmd)
            self.cmd_input.clear()

    def record_mic_action(self):
        """Bấm nút micro trên GUI để thu âm 4 giây với đếm ngược hiển thị trực quan."""
        if self.is_recording:
            return

        def task():
            self.is_recording = True
            rec_file = f"/tmp/tb4_gui_record_{time.time_ns()}.wav"

            # Đảm bảo xóa file tạm nếu đã có từ trước
            if os.path.exists(rec_file):
                try:
                    os.remove(rec_file)
                except Exception:
                    pass

            self.bridge.mic_status_signal.emit("🔴 ĐANG THU ÂM (4s)...", "#EF4444")
            self.btn_mic.setText("🔴 ĐANG THU ÂM... (HÃY NÓI VÀO MIC)")
            self.btn_mic.setStyleSheet("background-color: #DC2626; color: white;")
            self.log("Bắt đầu ghi âm 4 giây từ microphone ALSA...")

            # Ghi âm qua ALSA arecord
            cmd = ["arecord", "-D", "default", "-f", "S16_LE", "-r", "16000", "-c", "1", "-d", "4", rec_file]
            try:
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)

                self.bridge.mic_status_signal.emit("⏳ Đang giải mã văn bản...", "#FBBF24")
                self.btn_mic.setText("⏳ ĐANG NHẬN DẠNG VOSK TIẾNG VIỆT...")
                self.btn_mic.setStyleSheet("background-color: #D97706; color: white;")
                self.log("Ghi âm hoàn tất, đang nhận dạng qua Vosk Tiếng Việt...")

                # Tập từ vựng ngữ pháp chuẩn (Grammar Constrained) giúp Vosk chỉ nhận các lệnh hợp lệ, loại bỏ tạp âm mic
                grammar_list = [
                    "đi đến phòng học", "phòng học", "khu vực a", "phòng a", "lớp học", "học",
                    "đi đến phòng thí nghiệm", "phòng thí nghiệm", "khu vực b", "phòng b", "phòng lab", "thí nghiệm", "lab",
                    "đi đến phòng họp", "phòng họp", "khu vực c", "phòng c", "nhà kho", "kho", "họp",
                    "về trạm sạc", "trạm sạc", "trạm sạc dock", "dock", "về sạc", "về dock", "hành lang", "khu trung tâm",
                    "dừng lại", "dừng", "dừng robot",
                    "[unk]"
                ]
                grammar_json = json.dumps(grammar_list, ensure_ascii=False)

                # Nhận dạng giọng nói (ưu tiên Whisper nếu cài đặt, hoặc Vosk với Grammar)
                text = ""
                try:
                    import whisper
                    self.log("Đang nhận dạng bằng OpenAI Whisper...")
                    if not hasattr(self, '_whisper_model'):
                        self._whisper_model = whisper.load_model("base")
                    result = self._whisper_model.transcribe(rec_file, language="vi")
                    text = result.get('text', '').strip()
                except ImportError:
                    import vosk
                    if self.vosk_model is None and os.path.exists(self.vosk_model_path):
                        self.vosk_model = vosk.Model(self.vosk_model_path)

                    if self.vosk_model and os.path.exists(rec_file):
                        wf = wave.open(rec_file, "rb")
                        rec = vosk.KaldiRecognizer(self.vosk_model, 16000, grammar_json)
                        rec.Reset()
                        while True:
                            data = wf.readframes(4000)
                            if len(data) == 0:
                                break
                            rec.AcceptWaveform(data)
                        res = json.loads(rec.FinalResult())
                        text = res.get('text', '').replace('[unk]', '').strip()
                        rec.Reset()

                if text:
                    self.ros_node.send_command(text)
                    self.log(f"Đã giải mã thành công: '{text}'")
                else:
                    self.log("Cảnh báo: Không phát hiện lời nói trong bản thu âm.")
                    self.bridge.status_received.emit("⚠️ Không nhận diện được tiếng nói trong bản thu. Hãy thử nói to hơn.")
            except Exception as e:
                self.log(f"Lỗi khi thu âm/giải mã: {e}")
            finally:
                # Xóa sạch file âm thanh vừa ghi để chống lưu đệm cho các lần sau
                try:
                    if os.path.exists(rec_file):
                        os.remove(rec_file)
                except Exception:
                    pass
                self.is_recording = False
                self.btn_mic.setText("🎤 BẤM ĐỂ NÓI (THU ÂM 4 GIÂY)")
                self.btn_mic.setStyleSheet("")
                self.bridge.mic_status_signal.emit("🟢 Sẵn sàng", "#34D399")

        threading.Thread(target=task, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    bridge = RosSignalBridge()
    node = RosListenerNode(bridge)

    window = VoiceDashboardWindow(node, bridge)
    window.show()

    # Vòng lặp spin ROS 2 bằng QTimer không chặn giao diện Qt
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(30)

    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

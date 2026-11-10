#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
speech_node.py:
Node nhận dạng giọng nói cho điều khiển robot TurtleBot 4.
Hỗ trợ:
- Engine: Vosk (offline tiếng Việt) hoặc Whisper
- Input modes:
  + 'mic': Thu âm trực tiếp từ microphone (dùng ALSA arecord)
    * Khi chạy tương tác (TTY): bấm Enter để nói
    * Khi chạy nền / launch file: lắng nghe định kỳ liên tục
  + 'file': Đọc file âm thanh WAV đầu vào
  + 'text': Nhập văn bản trực tiếp từ bàn phím terminal (dự phòng tối ưu cho máy ảo)
- Publish chuỗi nhận dạng được lên topic: /voice_command_text (std_msgs/String)
"""

import os
import sys
import time
import wave
import json
import threading
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')

        # Khai báo các tham số ROS 2
        default_vosk_path = '/home/ductri/VSCode/hri/models/vosk-model-small-vn-0.4'
        self.declare_parameter('engine', 'vosk')                 # 'vosk' hoặc 'whisper'
        self.declare_parameter('model_path', default_vosk_path)
        self.declare_parameter('input_mode', 'text')             # 'text', 'file', 'mic'
        self.declare_parameter('wav_file', '')
        self.declare_parameter('topic_name', '/voice_command_text')

        self.engine = self.get_parameter('engine').get_parameter_value().string_value.lower()
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.input_mode = self.get_parameter('input_mode').get_parameter_value().string_value.lower()
        self.wav_file = self.get_parameter('wav_file').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(String, self.topic_name, 10)

        self.get_logger().info("==========================================")
        self.get_logger().info(f"Khởi động SpeechNode [Engine: {self.engine.upper()} | Mode: {self.input_mode.upper()}]")
        self.get_logger().info(f"Publish text lên topic: {self.topic_name}")
        self.get_logger().info("==========================================")

        self.vosk_model = None
        self.vosk_rec = None
        self.last_published_text = ""
        self.last_published_time = 0.0

        # Topic trigger để thu âm theo yêu cầu (Push-To-Talk)
        self.trigger_sub = self.create_subscription(
            String,
            '/voice_record_trigger',
            self.trigger_callback,
            10
        )
        self.trigger_record_requested = False
        if self.input_mode in ['file', 'mic']:
            self.init_speech_engine()

        # Khởi chạy luồng xử lý đầu vào
        self.is_running = True
        self.worker_thread = threading.Thread(target=self.run_input_loop, daemon=True)
        self.worker_thread.start()

    def trigger_callback(self, msg: String):
        """Kích hoạt thu âm một lần khi nhận tín hiệu từ GUI hoặc topic ngoài."""
        self.get_logger().info("[TRIGGER] Nhận yêu cầu thu âm giọng nói (Push-To-Talk).")
        self.trigger_record_requested = True

    def get_navigation_grammar(self):
        """Tập từ vựng ngữ pháp chuẩn cho điều khiển TurtleBot 4."""
        return [
            "đi đến phòng học", "phòng học", "khu vực a", "phòng a", "lớp học", "học",
            "đi đến phòng thí nghiệm", "phòng thí nghiệm", "khu vực b", "phòng b", "phòng lab", "thí nghiệm", "lab",
            "đi đến phòng họp", "phòng họp", "khu vực c", "phòng c", "nhà kho", "kho", "họp",
            "về trạm sạc", "trạm sạc", "trạm sạc dock", "dock", "về sạc", "về dock", "hành lang", "khu trung tâm",
            "dừng lại", "dừng", "dừng robot",
            "[unk]"
        ]

    def init_speech_engine(self):
        """Khởi tạo engine nhận dạng giọng nói offline Vosk hoặc Whisper."""
        if self.engine == 'vosk':
            try:
                import vosk
                if not os.path.exists(self.model_path):
                    self.get_logger().error(f"Không tìm thấy thư mục model Vosk: {self.model_path}")
                    return
                self.get_logger().info(f"Đang nạp mô hình Vosk từ: {self.model_path}...")
                self.vosk_model = vosk.Model(self.model_path)
                grammar_json = json.dumps(self.get_navigation_grammar(), ensure_ascii=False)
                self.vosk_rec = vosk.KaldiRecognizer(self.vosk_model, 16000, grammar_json)
                self.get_logger().info("Vosk Engine đã sẵn sàng với tập từ vựng chuẩn (Grammar Constrained)!")
            except Exception as e:
                self.get_logger().error(f"Lỗi khởi tạo Vosk: {e}")
        elif self.engine == 'whisper':
            try:
                import whisper
                self.whisper_model = whisper.load_model("base")
                self.get_logger().info("Whisper Engine đã sẵn sàng!")
            except Exception as e:
                self.get_logger().error(f"Lỗi khởi tạo Whisper: {e}")

    def transcribe_wav(self, wav_filepath):
        """Chuyển đổi file âm thanh WAV thành văn bản qua Vosk / Whisper và tự động dọn dẹp file tạm."""
        if not os.path.exists(wav_filepath):
            return ""

        text = ""
        if self.engine == 'vosk' and self.vosk_rec:
            try:
                wf = wave.open(wav_filepath, "rb")
                rec = self.vosk_rec
                rec.Reset()
                while True:
                    data = wf.readframes(4000)
                    if len(data) == 0:
                        break
                    rec.AcceptWaveform(data)

                final_res = json.loads(rec.FinalResult())
                text = final_res.get('text', '').strip()
                # Loại bỏ [unk] hoặc tạp âm không rõ từ
                text = text.replace('[unk]', '').strip()
                # Reset decoder một lần nữa để xóa sạch buffer âm thanh cũ
                rec.Reset()
            except Exception as e:
                self.get_logger().error(f"Lỗi khi giải mã Vosk: {e}")
        elif self.engine == 'whisper' and hasattr(self, 'whisper_model'):
            try:
                result = self.whisper_model.transcribe(wav_filepath, language="vi")
                text = result.get('text', '').strip()
            except Exception as e:
                self.get_logger().error(f"Lỗi khi giải mã Whisper: {e}")

        # Xóa file WAV ngay sau khi xử lý để chống tích lũy đệm cũ
        try:
            if os.path.exists(wav_filepath):
                os.remove(wav_filepath)
        except Exception:
            pass

        return text

    def record_mic_alsa(self, duration_sec=4, out_file=None):
        """Ghi âm từ microphone thông qua ALSA arecord với tên file duy nhất theo timestamp."""
        dur_int = max(1, int(round(float(duration_sec))))
        if out_file is None:
            out_file = f"/tmp/tb4_voice_{time.time_ns()}.wav"

        # Đảm bảo xóa file cũ nếu đã tồn tại
        if os.path.exists(out_file):
            try:
                os.remove(out_file)
            except Exception:
                pass

        cmd = [
            "arecord",
            "-D", "default",
            "-f", "S16_LE",
            "-r", "16000",
            "-c", "1",
            "-d", str(dur_int),
            out_file
        ]
        try:
            res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            if res.returncode != 0:
                err_msg = res.stderr.strip() if res.stderr else f"Mã lỗi {res.returncode}"
                if not getattr(self, '_last_arecord_err_logged', False):
                    self.get_logger().warn(f"Cảnh báo ALSA arecord: {err_msg} (sẽ thử lại sau)")
                    self._last_arecord_err_logged = True
                return None
            self._last_arecord_err_logged = False
            return out_file
        except Exception as e:
            if not getattr(self, '_last_arecord_err_logged', False):
                self.get_logger().warn(f"Lỗi khi thu âm bằng arecord: {e}")
                self._last_arecord_err_logged = True
            return None

    def publish_command(self, text):
        """Phát chuỗi lệnh nhận diện được lên ROS 2 topic với bộ lọc trùng lặp (Debounce)."""
        text = text.strip()
        if not text:
            return

        now = time.time()
        # Nếu cùng một câu lệnh đến trong vòng 6.0 giây, bỏ qua để chống lặp vô hạn
        if text == self.last_published_text and (now - self.last_published_time) < 6.0:
            self.get_logger().info(
                f"[LỌC TRÙNG LẶP] Bỏ qua câu lệnh lặp lại \"{text}\" (cách lần trước {(now - self.last_published_time):.1f}s)."
            )
            return

        self.last_published_text = text
        self.last_published_time = now

        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"[ĐÃ PHÁT LỆNH GIỌNG NÓI] -> /voice_command_text: \"{text}\"")

    def run_input_loop(self):
        """Vòng lặp thu thập câu lệnh theo chế độ được cấu hình."""
        if self.input_mode == 'text':
            is_tty = sys.stdin.isatty()
            if is_tty:
                print("\n" + "="*50)
                print("  CHẾ ĐỘ NHẬP LỆNH VĂN BẢN (TEXT MODE)")
                print("  Gợi ý các câu lệnh:")
                print("    - Đi đến khu vực A / Đi đến phòng học / học")
                print("    - Đi đến phòng thí nghiệm / khu vực B")
                print("    - Đi đến phòng họp / khu vực C")
                print("    - Về trạm sạc / quay về hành lang")
                print("    - Di chuyển từ khu vực A đến khu vực B")
                print("  Nhập 'exit' để thoát.")
                print("="*50 + "\n")

            while self.is_running and rclpy.ok():
                try:
                    if is_tty:
                        cmd_input = input(">> Nhập câu lệnh điều hướng: ").strip()
                        if cmd_input.lower() in ['exit', 'quit']:
                            break
                        if cmd_input:
                            self.publish_command(cmd_input)
                    else:
                        time.sleep(1.0)
                except EOFError:
                    is_tty = False
                    time.sleep(1.0)
                except Exception as e:
                    self.get_logger().warn(f"Lỗi nhập dữ liệu: {e}")
                    time.sleep(1.0)

        elif self.input_mode == 'file':
            if self.wav_file and os.path.exists(self.wav_file):
                self.get_logger().info(f"Đang xử lý file âm thanh: {self.wav_file}")
                text = self.transcribe_wav(self.wav_file)
                self.publish_command(text)
            else:
                self.get_logger().warn(f"Chưa cung cấp wav_file hoặc file không tồn tại: {self.wav_file}")

        elif self.input_mode == 'mic':
            is_tty = sys.stdin.isatty()
            if is_tty:
                print("\n[CHẾ ĐỘ MICROPHONE TƯƠNG TÁC]")
                print("  Nhấn [Enter] để bắt đầu thu âm một câu lệnh (4 giây)...")
                print("  Gợi ý câu lệnh: 'đi đến phòng học', 'phòng thí nghiệm', 'phòng họp', 'về trạm sạc', 'dừng lại'")
                print("="*50 + "\n")
            else:
                self.get_logger().info(
                    "SpeechNode [MIC Push-To-Talk]: Sẵn sàng. Thu âm được kích hoạt chủ động qua nút bấm GUI hoặc topic /voice_record_trigger. (Không tự động thu âm liên tục để chống nhiễu)."
                )

            while self.is_running and rclpy.ok():
                try:
                    should_record = False
                    if is_tty:
                        input("🎤 Nhấn [Enter] để nói...")
                        should_record = True
                    else:
                        if self.trigger_record_requested:
                            self.trigger_record_requested = False
                            should_record = True
                        else:
                            time.sleep(0.1)
                            continue

                    if should_record:
                        recorded_path = self.record_mic_alsa(duration_sec=4)
                        if recorded_path and os.path.exists(recorded_path):
                            text = self.transcribe_wav(recorded_path)
                            if text:
                                self.publish_command(text)
                            elif is_tty:
                                self.get_logger().warn("Không nhận diện được giọng nói trong bản ghi.")
                except EOFError:
                    is_tty = False
                    time.sleep(1.0)
                except Exception as e:
                    self.get_logger().error(f"Lỗi microphone: {e}")
                    time.sleep(1.0)

    def destroy_node(self):
        self.is_running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
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

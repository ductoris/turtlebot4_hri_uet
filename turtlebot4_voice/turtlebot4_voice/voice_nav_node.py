#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
voice_nav_node.py:
Node xử lý ngôn ngữ tự nhiên (NLU), phân tích ngữ nghĩa câu lệnh giọng nói,
và điều khiển robot di chuyển tới các khu vực phòng thông qua Nav2 NavigateToPose.

Hỗ trợ:
- Phân tích câu lệnh đa dạng:
  + "Đi đến [đích]" / "Tới [đích]" / "Về [đích]"
  + "Di chuyển từ [A] đến [B]"
  + Từ khóa rút gọn: "học", "thí nghiệm", "họp", "sạc", "hành lang"
  + Câu lệnh dừng: "dừng lại", "stop", "hủy"
- Khử dấu tiếng Việt, fuzzy matching (rapidfuzz/difflib), loại bỏ stopwords/tiền tố.
- Action Client NavigateToPose với hàng đợi worker thread bất đồng bộ:
  + Chờ Nav2 Action Server sẵn sàng tới 30 giây (không bao giờ drop lệnh khi Nav2 đang boot).
  + Tự động cập nhật khoảng cách còn lại (real-time distance monitoring).
  + Hủy bỏ mục tiêu cũ an toàn khi có câu lệnh mới.
"""

import os
import sys
import re
import math
import yaml
import string
import unicodedata
import threading
from difflib import SequenceMatcher

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros
from ament_index_python.packages import get_package_share_directory

try:
    from rapidfuzz import fuzz
    USE_RAPIDFUZZ = True
except ImportError:
    USE_RAPIDFUZZ = False

def remove_accents(input_str: str) -> str:
    """Loại bỏ dấu tiếng Việt và chuyển sang chữ thường không dấu."""
    s = input_str.lower().strip()
    s = s.replace('đ', 'd').replace('Đ', 'd')
    nfkd = unicodedata.normalize('NFKD', s)
    no_accent = ''.join([c for c in nfkd if not unicodedata.combining(c)])
    return no_accent

def clean_text(text: str) -> str:
    """Làm sạch chuỗi văn bản: bỏ dấu câu, chuẩn hóa khoảng trắng."""
    t = text.lower()
    for p in string.punctuation:
        t = t.replace(p, ' ')
    t = re.sub(r'\s+', ' ', t).strip()
    return t

def get_core_entity(s: str) -> str:
    """Tách phần thực thể cốt lõi, loại bỏ các tiền tố chung như 'khu vực', 'phòng', 'nhà', 'lớp'."""
    return re.sub(r'^(?:khu vuc|khu vực|phong|phòng|vung|vùng|nha|nhà|lop|lớp)\s+', '', s).strip()

def string_similarity(s1: str, s2: str) -> float:
    """Tính độ tương đồng giữa hai chuỗi (0.0 đến 1.0)."""
    if USE_RAPIDFUZZ:
        return fuzz.ratio(s1, s2) / 100.0
    else:
        return SequenceMatcher(None, s1, s2).ratio()

class VoiceNavNode(Node):
    def __init__(self):
        super().__init__('voice_nav_node')

        self.cb_group = ReentrantCallbackGroup()

        # Tham số cấu hình
        default_regions_file = os.path.join(
            get_package_share_directory('turtlebot4_voice'),
            'config', 'regions.yaml'
        )
        self.declare_parameter('regions_file', default_regions_file)
        self.declare_parameter('similarity_threshold', 0.70)
        self.regions_file = self.get_parameter('regions_file').get_parameter_value().string_value
        self.sim_thresh = self.get_parameter('similarity_threshold').get_parameter_value().double_value

        # Nạp dữ liệu các vùng ngữ nghĩa
        self.regions = self.load_regions(self.regions_file)

        # Action Client NavigateToPose của Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        self.current_goal_handle = None
        self.is_navigating = False
        self.active_region = None
        self.current_goal_id = 0
        self.goal_lock = threading.Lock()

        # TF Buffer & Listener để theo dõi tọa độ robot trong map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber nhận văn bản giọng nói
        self.cmd_sub = self.create_subscription(
            String,
            '/voice_command_text',
            self.command_callback,
            10,
            callback_group=self.cb_group
        )

        # Publisher báo cáo trạng thái hệ thống điều hướng
        self.status_pub = self.create_publisher(String, '/voice_nav/status', 10)

        # Timer kiểm tra và cập nhật khoảng cách còn lại tới đích (1 Hz)
        self.dist_timer = self.create_timer(1.0, self.distance_tracking_callback, callback_group=self.cb_group)

        self.publish_status("VoiceNavNode đã sẵn sàng nhận lệnh.")
        self.get_logger().info(f"VoiceNavNode đã khởi động. Đã nạp {len(self.regions)} khu vực từ {self.regions_file}")

    def load_regions(self, filepath: str):
        """Nạp dữ liệu cấu hình vùng từ file regions.yaml."""
        if not os.path.exists(filepath):
            self.get_logger().error(f"Không tìm thấy file: {filepath}")
            return []
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data.get('regions', [])
        except Exception as e:
            self.get_logger().error(f"Lỗi khi đọc file regions.yaml: {e}")
            return []

    def publish_status(self, text: str):
        """Publish thông báo trạng thái lên topic /voice_nav/status."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def get_robot_pose(self):
        """Lấy tọa độ hiện tại (x, y) của robot từ TF map -> base_link."""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return float(x), float(y)
        except Exception:
            return None

    def distance_tracking_callback(self):
        """Định kỳ tính toán và cập nhật khoảng cách thực tế còn lại tới đích."""
        if not self.is_navigating or not self.active_region:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        rx, ry = pose
        gx, gy = self.active_region.get('world_goal', [0.0, 0.0])
        dist = math.hypot(gx - rx, gy - ry)

        reg_name = self.active_region.get('name', 'đích')
        reg_id = self.active_region.get('id', '')
        if dist > 0.3:
            self.publish_status(f"Đang di chuyển tới {reg_name} ({reg_id})... Khoảng cách còn: {dist:.2f} m")

    def get_current_region_id(self):
        """Xác định ID vùng mà robot hiện tại đang đứng dựa trên tọa độ và đa giác polygon."""
        pose = self.get_robot_pose()
        if pose is None:
            return None
        rx, ry = pose

        for reg in self.regions:
            poly_pts = reg.get('polygon_world', [])
            if len(poly_pts) >= 3:
                poly_arr = np.array(poly_pts, dtype=np.float32)
                dist = cv2.pointPolygonTest(poly_arr, (rx, ry), False)
                if dist >= 0:
                    return reg['id']
            else:
                bbox = reg.get('bbox_world', [])
                if len(bbox) == 2:
                    min_x, min_y = bbox[0]
                    max_x, max_y = bbox[1]
                    if min_x <= rx <= max_x and min_y <= ry <= max_y:
                        return reg['id']
        return None

    def match_region(self, phrase: str):
        """
        Tìm khu vực khớp nhất với cụm từ phrase thông qua danh sách aliases.
        Hỗ trợ so khớp chính xác, so khớp không dấu và fuzzy matching.
        """
        phrase_clean = clean_text(phrase)
        phrase_no_accent = remove_accents(phrase_clean)
        phrase_core = get_core_entity(phrase_no_accent)

        if not phrase_clean:
            return None

        # 1. So khớp chính xác ID hoặc Alias hoặc Core Entity
        for reg in self.regions:
            reg_id = reg.get('id', '').lower()
            if phrase_core == reg_id or phrase_no_accent == reg_id:
                return reg

            name = reg.get('name', '')
            aliases = reg.get('aliases', []) + [name, f"khu vực {reg['id']}", f"phòng {reg['id']}", reg['id']]

            for alias in aliases:
                alias_clean = clean_text(alias)
                alias_no_accent = remove_accents(alias_clean)
                alias_core = get_core_entity(alias_no_accent)

                if phrase_clean == alias_clean or phrase_no_accent == alias_no_accent or phrase_core == alias_core:
                    return reg

        # 2. Nếu thực thể cốt lõi chỉ là 1 ký tự (như 'z', 'x') mà không khớp ID nào -> không hợp lệ
        if len(phrase_core) == 1:
            return None

        # 3. Tìm kiếm toàn bộ từ (whole word) trong câu
        for reg in self.regions:
            aliases = reg.get('aliases', []) + [reg.get('name', '')]
            for alias in aliases:
                alias_core = get_core_entity(remove_accents(clean_text(alias)))
                if len(alias_core) > 2 and re.search(r'\b' + re.escape(alias_core) + r'\b', phrase_no_accent):
                    return reg

        # 4. Fuzzy matching trên thực thể cốt lõi (Core Entity)
        best_match = None
        highest_score = 0.0

        for reg in self.regions:
            aliases = reg.get('aliases', []) + [reg.get('name', '')]
            for alias in aliases:
                alias_core = get_core_entity(remove_accents(clean_text(alias)))
                if len(alias_core) > 2:
                    sim = string_similarity(phrase_core, alias_core)
                    if sim > highest_score:
                        highest_score = sim
                        best_match = reg

        if highest_score >= self.sim_thresh:
            return best_match
        return None

    def parse_command(self, raw_cmd: str):
        """
        Phân tích cú pháp câu lệnh tự nhiên (NLU):
        Hỗ trợ:
          - Câu lệnh dừng: "dừng lại", "stop", "hủy" -> trả về ('STOP', None, None)
          - "Di chuyển từ [A] đến [B]" -> trả về (orig_reg, dest_reg, None)
          - "Đi đến [đích]" / "tới [đích]" / "về [đích]" -> trả về (None, dest_reg, None)
          - So khớp từ khóa trực tiếp: "học", "thí nghiệm", "họp", "trạm sạc" -> trả về (None, dest_reg, None)
        Trả về: (origin_region, destination_region, error_msg)
        """
        text = clean_text(raw_cmd)
        text_no_acc = remove_accents(text)

        if not text:
            return None, None, "Câu lệnh rỗng, xin vui lòng nói lại."

        # Kiểm tra lệnh dừng khẩn cấp
        if text_no_acc in ['dung', 'dung lai', 'dung robot', 'stop', 'huy', 'cancel', 'quay xe']:
            return 'STOP', None, None

        # Mẫu 1: Kiểm tra dạng "từ <origin> đến/tới/sang/về <destination>"
        match_ab = re.search(r'(?:tu|từ)\s+(.+?)\s+(?:den|đến|toi|tới|sang|ve|về)\s+(.+)', text_no_acc)
        if match_ab:
            orig_phrase = match_ab.group(1).strip()
            dest_phrase = match_ab.group(2).strip()

            orig_reg = self.match_region(orig_phrase)
            dest_reg = self.match_region(dest_phrase)

            if not dest_reg:
                valid_names = ", ".join([f"{r['name']} ({r['id']})" for r in self.regions])
                return None, None, f"Không tìm thấy đích đến '{dest_phrase}'. Các khu vực hợp lệ: {valid_names}"

            return orig_reg, dest_reg, None

        # Mẫu 2: Kiểm tra dạng "đi đến / tới / về / quay về <destination>"
        match_single = re.search(r'(?:di den|đi đến|den|đến|toi|tới|di ve|đi về|ve|về|quay ve|quay về|chay den|chạy đến)\s+(.+)', text_no_acc)
        if match_single:
            target_phrase = match_single.group(1).strip()
            dest_reg = self.match_region(target_phrase)
            if dest_reg:
                return None, dest_reg, None
            else:
                valid_names = ", ".join([f"{r['name']} ({r['id']})" for r in self.regions])
                return None, None, f"Không nhận diện được khu vực '{target_phrase}'. Các khu vực hợp lệ: {valid_names}"

        # Mẫu 3: So khớp trực tiếp toàn bộ câu với các vùng
        dest_reg = self.match_region(text)
        if dest_reg:
            return None, dest_reg, None

        valid_names = ", ".join([f"{r['name']} ({r['id']})" for r in self.regions])
        return None, None, f"Không hiểu câu lệnh: '{raw_cmd}'. Vui lòng nói theo mẫu 'Đi đến [khu vực]'. Hợp lệ: {valid_names}"

    def cancel_current_navigation(self):
        """Hủy bỏ mục tiêu điều hướng đang chạy nếu có."""
        with self.goal_lock:
            if self.is_navigating and self.current_goal_handle:
                self.get_logger().info("Đang hủy mục tiêu điều hướng hiện tại để thực hiện lệnh mới...")
                try:
                    self.current_goal_handle.cancel_goal_async()
                except Exception as e:
                    self.get_logger().warn(f"Lỗi khi hủy goal: {e}")
                self.is_navigating = False
                self.active_region = None

    def send_nav_goal_async(self, region):
        """Khởi chạy luồng gửi goal bất đồng bộ để đợi Nav2 sẵn sàng mà không bao giờ drop goal."""
        with self.goal_lock:
            self.current_goal_id += 1
            this_goal_id = self.current_goal_id
            self.active_region = region

        # Chạy trong worker thread để không block rclpy executor
        threading.Thread(
            target=self._dispatch_goal_worker,
            args=(region, this_goal_id),
            daemon=True
        ).start()

    def _dispatch_goal_worker(self, region, goal_id):
        reg_name = region.get('name', '')
        reg_id = region.get('id', '')
        gx, gy = region.get('world_goal', [0.0, 0.0])
        yaw = float(region.get('yaw', 0.0))

        # Kiểm tra Action Server
        if not self.nav_client.server_is_ready():
            self.publish_status(f"Đang chờ Nav2 Action Server sẵn sàng để đi tới {reg_name} ({reg_id})...")
            self.get_logger().info(f"Nav2 Action Server chưa sẵn sàng. Đang chờ kết nối (tối đa 60s)...")

            ready = False
            for attempt in range(1, 31):
                # Kiểm tra nếu đã có goal mới thay thế trong lúc chờ
                with self.goal_lock:
                    if self.current_goal_id != goal_id:
                        self.get_logger().info(f"Goal {goal_id} bị thay thế bởi goal mới hơn. Dừng chờ.")
                        return

                if self.nav_client.wait_for_server(timeout_sec=2.0):
                    ready = True
                    break

                self.publish_status(f"Đang kết nối Nav2 Action Server... ({attempt*2}/60s)")
                self.get_logger().info(f"Đang chờ Nav2 server... ({attempt*2}/60s)")

            if not ready:
                self.publish_status(f"Lỗi: Nav2 Action Server không phản hồi sau 60s. Vui lòng kiểm tra mô phỏng.")
                self.get_logger().error("Nav2 Action Server không phản hồi sau 60 giây!")
                return

        # Kiểm tra lại xem goal này có còn là goal mới nhất không
        with self.goal_lock:
            if self.current_goal_id != goal_id:
                return

        self.get_logger().info(f"Nav2 Action Server đã sẵn sàng! Gửi goal tới {reg_name} ({reg_id}) tại ({gx:.2f}, {gy:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.publish_status(f"Đang di chuyển tới {reg_name} ({reg_id}) tại ({gx:.2f}, {gy:.2f})...")
        self.is_navigating = True
        self._last_fb_nanos = 0

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.feedback_callback(fb, reg_name, reg_id)
        )
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, region, goal_id))

    def feedback_callback(self, feedback_msg, reg_name, reg_id):
        dist = feedback_msg.feedback.distance_remaining
        curr_nanos = self.get_clock().now().nanoseconds
        if curr_nanos - self._last_fb_nanos > 2_000_000_000:
            self._last_fb_nanos = curr_nanos
            self.publish_status(f"Đang đến {reg_name} ({reg_id}) | Khoảng cách còn lại: {dist:.2f} m")

    def goal_response_callback(self, future, region, goal_id):
        with self.goal_lock:
            if self.current_goal_id != goal_id:
                return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status(f"Thất bại: Nav2 từ chối mục tiêu tới {region['name']}.")
            self.is_navigating = False
            self.active_region = None
            return

        with self.goal_lock:
            self.current_goal_handle = goal_handle

        self.publish_status(f"Nav2 đã chấp nhận mục tiêu! Bắt đầu dẫn đường tới {region['name']} ({region['id']}).")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda res: self.goal_result_callback(res, region, goal_id))

    def goal_result_callback(self, future, region, goal_id):
        with self.goal_lock:
            if self.current_goal_id != goal_id:
                return
            self.is_navigating = False
            self.active_region = None

        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                curr_pose = self.get_robot_pose()
                gx, gy = region.get('world_goal', [0.0, 0.0])
                if curr_pose is not None:
                    dist_to_goal = math.hypot(curr_pose[0] - gx, curr_pose[1] - gy)
                    if dist_to_goal <= 0.8:
                        self.publish_status(f"🎉 Đã đến {region['name']} ({region['id']}) an toàn!")
                    else:
                        self.publish_status(f"Robot đã tiếp cận {region['name']} ({region['id']}) tại ({curr_pose[0]:.2f}, {curr_pose[1]:.2f}) (cách đích {dist_to_goal:.2f}m).")
                else:
                    self.publish_status(f"🎉 Đã đến {region['name']} ({region['id']}) an toàn!")
            elif status == GoalStatus.STATUS_CANCELED:
                self.publish_status(f"Mục tiêu tới {region['name']} đã được hủy.")
            elif status == GoalStatus.STATUS_ABORTED:
                self.publish_status(f"Nav2 đã hủy bỏ mục tiêu tới {region['name']} (đường đi bị chặn hoặc vượt quá thời gian).")
            else:
                self.publish_status(f"Mục tiêu tới {region['name']} kết thúc với mã trạng thái: {status}.")
        except Exception as e:
            self.publish_status(f"Lỗi khi nhận kết quả Nav2: {e}")

    def command_callback(self, msg: String):
        """Xử lý câu lệnh giọng nói từ topic /voice_command_text."""
        raw_cmd = msg.data.strip()
        self.get_logger().info(f"\n[NHẬN LỆNH MỚI] \"{raw_cmd}\"")

        orig_reg, dest_reg, err = self.parse_command(raw_cmd)

        if orig_reg == 'STOP':
            self.cancel_current_navigation()
            self.publish_status("🛑 Đã dừng robot theo yêu cầu!")
            return

        if err:
            self.get_logger().warn(f"[LỖI CÂU LỆNH] {err}")
            self.publish_status(f"Lỗi: {err}")
            return

        # Kiểm tra lệnh trùng lặp: nếu robot đang di chuyển đến cùng một đích thì không hủy và không phát lại
        if self.is_navigating and self.active_region:
            target_id = dest_reg['id'] if (dest_reg and not orig_reg) else (orig_reg['id'] if orig_reg else None)
            if target_id and self.active_region.get('id') == target_id:
                self.get_logger().info(
                    f"Robot đang di chuyển đến {self.active_region['name']} ({target_id}), tiếp tục hành trình (bỏ qua lệnh lặp)."
                )
                return

        # Hủy mục tiêu đang chạy nếu có lệnh di chuyển mới sang điểm khác
        if self.is_navigating:
            self.cancel_current_navigation()

        # Xử lý logic câu lệnh "Từ A đến B"
        if orig_reg and dest_reg:
            current_reg_id = self.get_current_region_id()
            self.get_logger().info(f"Lệnh yêu cầu di chuyển từ [{orig_reg['id']}] tới [{dest_reg['id']}]. Robot hiện ở: [{current_reg_id}]")

            if current_reg_id == orig_reg['id']:
                self.get_logger().info(f"Robot đã ở đúng vị trí xuất phát {orig_reg['name']}. Đi thẳng tới {dest_reg['name']}.")
                self.publish_status(f"Đích đã xác định: {dest_reg['name']} ({dest_reg['id']})")
                self.send_nav_goal_async(dest_reg)
            else:
                self.get_logger().info(f"Robot chưa ở {orig_reg['name']}. Sẽ điều hướng tới {orig_reg['name']} trước.")
                self.publish_status(f"Đích bước 1: {orig_reg['name']} ({orig_reg['id']})")
                self.send_nav_goal_async(orig_reg)
        else:
            # Lệnh đi đến một điểm đích duy nhất
            self.publish_status(f"Đích đã xác định: {dest_reg['name']} ({dest_reg['id']})")
            self.send_nav_goal_async(dest_reg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

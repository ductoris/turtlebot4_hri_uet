#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
regions_marker_node.py:
Publish MarkerArray lên topic /regions_markers để hiển thị trực quan các khu vực,
tên phòng, và điểm đích trên bản đồ RViz2.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String
import tf2_ros

class RegionsMarkerNode(Node):
    def __init__(self):
        super().__init__('regions_marker_node')

        # Khai báo tham số
        default_config = os.path.join(
            get_package_share_directory('turtlebot4_voice'),
            'config', 'regions.yaml'
        )
        self.declare_parameter('regions_file', default_config)
        self.regions_file = self.get_parameter('regions_file').get_parameter_value().string_value

        self.marker_pub = self.create_publisher(MarkerArray, '/regions_markers', 10)

        # Lắng nghe câu lệnh giọng nói và trạng thái để hiển thị trực tiếp lên robot trong RViz
        self.last_voice_cmd = "(Chờ lệnh từ microphone...)"
        self.last_status = "Sẵn sàng"

        self.sub_voice = self.create_subscription(String, '/voice_command_text', self.cb_voice, 10)
        self.sub_status = self.create_subscription(String, '/voice_nav/status', self.cb_status, 10)

        # TF để định vị robot trong RViz
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nạp dữ liệu regions
        self.regions_data = self.load_regions(self.regions_file)

        # Timer định kỳ publish marker lên RViz (mỗi 0.5 giây)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.get_logger().info(f"RegionsMarkerNode đã khởi động, nạp cấu hình từ: {self.regions_file}")

    def cb_voice(self, msg: String):
        self.last_voice_cmd = msg.data.strip()

    def cb_status(self, msg: String):
        self.last_status = msg.data.strip()

    def load_regions(self, filepath):
        if not os.path.exists(filepath):
            self.get_logger().error(f"Không tìm thấy file: {filepath}")
            return []
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            return data.get('regions', [])
        except Exception as e:
            self.get_logger().error(f"Lỗi khi nạp file regions: {e}")
            return []

    def publish_markers(self):
        if not self.regions_data:
            return

        marker_array = MarkerArray()
        idx = 0

        # Bảng màu tương ứng cho các vùng
        colors = {
            'A': (0.2, 0.6, 1.0),        # Xanh dương
            'B': (0.2, 0.8, 0.2),        # Xanh lá
            'HALLWAY': (0.9, 0.7, 0.1),  # Vàng cam
            'C': (0.9, 0.3, 0.3)         # Đỏ nhạt
        }

        for reg in self.regions_data:
            reg_id = reg.get('id', '')
            name = reg.get('name', '')
            gx, gy = reg.get('world_goal', [0.0, 0.0])
            r, g, b = colors.get(reg_id, (0.5, 0.5, 0.5))

            # 1. Marker hình trụ (Cylinder) đánh dấu tâm điểm đích
            point_marker = Marker()
            point_marker.header.frame_id = 'map'
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = 'region_goals'
            point_marker.id = idx
            point_marker.type = Marker.CYLINDER
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = float(gx)
            point_marker.pose.position.y = float(gy)
            point_marker.pose.position.z = 0.1
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.35
            point_marker.scale.y = 0.35
            point_marker.scale.z = 0.2
            point_marker.color.r = r
            point_marker.color.g = g
            point_marker.color.b = b
            point_marker.color.a = 0.85
            marker_array.markers.append(point_marker)
            idx += 1

            # 2. Marker chữ (Text) hiển thị tên phòng và mã ID
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'region_labels'
            text_marker.id = idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(gx)
            text_marker.pose.position.y = float(gy)
            text_marker.pose.position.z = 0.5  # Nâng cao độ chữ lên trên điểm đích
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.35  # Kích thước chữ
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            text_marker.text = f"{name} ({reg_id})\n[{gx:.2f}, {gy:.2f}]"
            marker_array.markers.append(text_marker)
            idx += 1

        # 3. Bong bóng hiển thị Lời nói Microphone ngay trên đầu Robot trong RViz
        try:
            now = rclpy.time.Time()
            tf_trans = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.1))
            rx = tf_trans.transform.translation.x
            ry = tf_trans.transform.translation.y

            robot_bubble = Marker()
            robot_bubble.header.frame_id = 'map'
            robot_bubble.header.stamp = self.get_clock().now().to_msg()
            robot_bubble.ns = 'robot_speech_bubble'
            robot_bubble.id = 999
            robot_bubble.type = Marker.TEXT_VIEW_FACING
            robot_bubble.action = Marker.ADD
            robot_bubble.pose.position.x = rx
            robot_bubble.pose.position.y = ry
            robot_bubble.pose.position.z = 0.8  # Nổi trên nóc robot
            robot_bubble.pose.orientation.w = 1.0
            robot_bubble.scale.z = 0.30
            robot_bubble.color.r = 0.1
            robot_bubble.color.g = 0.9
            robot_bubble.color.b = 1.0
            robot_bubble.color.a = 1.0
            robot_bubble.text = f"🎙️ \"{self.last_voice_cmd}\"\n📍 {self.last_status}"
            marker_array.markers.append(robot_bubble)
        except Exception:
            pass

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = RegionsMarkerNode()
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

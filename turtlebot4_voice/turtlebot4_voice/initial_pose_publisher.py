#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
initial_pose_publisher.py (Enhanced with Ground-Truth Simulation Pose Sync):
1. Tự động thiết lập initial pose tới topic /initialpose của AMCL khi khởi chạy hệ thống (0.0, 0.0, yaw=0.0).
2. Lắng nghe ground-truth pose thực tế từ mô phỏng Gazebo (_internal/sim_ground_truth_pose).
3. Giám sát độ sai lệch giữa AMCL ước lượng trong RViz và tọa độ vật lý thực trong Gazebo:
   Nếu phát hiện sai lệch > 0.35m (do trượt bánh xe hoặc AMCL phân tán hạt), node sẽ tự động
   phát lệnh tái định vị /initialpose chuẩn xác theo Gazebo, triệt tiêu hoàn toàn hiện tượng
   "RViz không khớp Gazebo" hay "robot trong rviz ở một nơi còn gazebo ở một nẻo".
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_msgs.msg import TFMessage

def quaternion_to_yaw(x, y, z, w):
    """Chuyển đổi quaternion sang góc yaw (radian)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay_sec', 3.0)
        self.declare_parameter('enable_sync', True)
        self.declare_parameter('max_deviation_m', 0.35)

        self.default_x = self.get_parameter('x').get_parameter_value().double_value
        self.default_y = self.get_parameter('y').get_parameter_value().double_value
        self.default_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.delay_sec = self.get_parameter('delay_sec').get_parameter_value().double_value
        self.enable_sync = self.get_parameter('enable_sync').get_parameter_value().bool_value
        self.max_deviation_m = self.get_parameter('max_deviation_m').get_parameter_value().double_value

        # Tọa độ Ground Truth từ Gazebo
        self.gt_pose = None
        self.gt_x = self.default_x
        self.gt_y = self.default_y
        self.gt_yaw = self.default_yaw
        self.gt_orientation = (0.0, 0.0, 0.0, 1.0)
        self.has_gt = False

        # Tọa độ AMCL từ RViz/Nav2
        self.amcl_pose = None
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.has_amcl = False

        self.publish_count = 0
        self.last_resync_time = 0.0

        # Publishers & Subscribers
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.gt_pub = self.create_publisher(PoseStamped, '/ground_truth_pose', 10)

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.gt_sub = self.create_subscription(
            TFMessage,
            '_internal/sim_ground_truth_pose',
            self.gt_pose_callback,
            10
        )

        self.get_logger().info(f"Khởi động InitialPosePublisher & SimPoseSync. Chờ {self.delay_sec}s để Nav2 sẵn sàng...")
        self.timer = self.create_timer(1.0, self.timer_callback)

    def gt_pose_callback(self, msg: TFMessage):
        """Nhận vị trí thực tế của robot từ Gazebo qua ROS-Ignition bridge."""
        for t in msg.transforms:
            child = t.child_frame_id.lower()
            if 'turtlebot4' in child or 'base_link' in child:
                self.gt_x = t.transform.translation.x
                self.gt_y = t.transform.translation.y
                self.gt_orientation = (
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                )
                self.gt_yaw = quaternion_to_yaw(*self.gt_orientation)
                self.has_gt = True

                # Publish PoseStamped cho RViz nếu cần soi chiếu
                gt_msg = PoseStamped()
                gt_msg.header.frame_id = 'map'
                gt_msg.header.stamp = self.get_clock().now().to_msg()
                gt_msg.pose.position.x = self.gt_x
                gt_msg.pose.position.y = self.gt_y
                gt_msg.pose.position.z = 0.0
                gt_msg.pose.orientation.x = self.gt_orientation[0]
                gt_msg.pose.orientation.y = self.gt_orientation[1]
                gt_msg.pose.orientation.z = self.gt_orientation[2]
                gt_msg.pose.orientation.w = self.gt_orientation[3]
                self.gt_pub.publish(gt_msg)
                break

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Nhận vị trí ước lượng của AMCL trong RViz."""
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.amcl_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.has_amcl = True

    def publish_initial_pose(self, x, y, qz, qw, reason="Khởi tạo"):
        """Gửi thông điệp /initialpose tới AMCL để gom cụm hạt về đúng vị trí."""
        now_time = self.get_clock().now()
        if now_time.nanoseconds == 0:
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = now_time.to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Ma trận hiệp phương sai nhỏ (độ tin cậy cực cao để AMCL hội tụ tức thì)
        covariance = [0.0] * 36
        covariance[0] = 0.02   # x var
        covariance[7] = 0.02   # y var
        covariance[35] = 0.03  # yaw var
        msg.pose.covariance = covariance

        self.pub.publish(msg)
        self.get_logger().info(f"[{reason}] Đã gửi /initialpose: ({x:.2f}, {y:.2f})")

    def timer_callback(self):
        now_time = self.get_clock().now()
        if now_time.nanoseconds == 0:
            return

        now_sec = time.time()

        # 1. Giai đoạn khởi động (Boot phase): Phát initial pose liên tục vài lần đầu
        if self.publish_count < 5:
            target_x = self.gt_x if self.has_gt else self.default_x
            target_y = self.gt_y if self.has_gt else self.default_y
            if self.has_gt:
                qz = self.gt_orientation[2]
                qw = self.gt_orientation[3]
            else:
                qz = math.sin(self.default_yaw / 2.0)
                qw = math.cos(self.default_yaw / 2.0)

            self.publish_initial_pose(target_x, target_y, qz, qw, reason=f"Khởi tạo lần {self.publish_count + 1}")
            self.publish_count += 1
            return

        # 2. Giai đoạn hoạt động (Runtime Watchdog): Tự động phát hiện lệch RViz - Gazebo
        if self.enable_sync and self.has_gt and self.has_amcl:
            # Tính khoảng cách sai lệch giữa Gazebo thật và RViz AMCL
            dx = self.gt_x - self.amcl_x
            dy = self.gt_y - self.amcl_y
            deviation = math.hypot(dx, dy)

            # Nếu khoảng cách lệch vượt ngưỡng (e.g. > 0.35m do trượt bánh hoặc phân tán hạt)
            # và cách lần resync gần nhất ít nhất 2.5s:
            if deviation > self.max_deviation_m and (now_sec - self.last_resync_time) > 2.5:
                self.get_logger().warn(
                    f"⚠️ [PHÁT HIỆN LỆCH VỊ TRÍ] RViz AMCL ({self.amcl_x:.2f}, {self.amcl_y:.2f}) "
                    f"lệch so với Gazebo thật ({self.gt_x:.2f}, {self.gt_y:.2f}) là {deviation:.2f}m! "
                    f"Đang tự động đồng bộ lại /initialpose..."
                )
                qz = self.gt_orientation[2]
                qw = self.gt_orientation[3]
                self.publish_initial_pose(self.gt_x, self.gt_y, qz, qw, reason="ĐỒNG BỘ LỆCH VỊ TRÍ")
                self.last_resync_time = now_sec

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
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

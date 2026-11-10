#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_ground_truth_odom.py:
Node chỉnh sửa TF map->odom dựa trên ground-truth pose thật từ Gazebo Ignition.

Nguyên lý hoạt động:
  1. Đọc tọa độ thực tế (ground truth) của robot từ Gazebo qua topic
     '_internal/sim_ground_truth_pose' (tf2_msgs/TFMessage, do parameter_bridge phát
     từ topic native '/world/warehouse/pose/info' của Ignition Gazebo).
  2. Đọc biến đổi 'odom -> base_link' từ diffdrive_controller.
  3. Tính: map_T_odom = map_T_base * inverse(odom_T_base)
  4. Phát biến đổi 'map -> odom' lên /tf ở 30Hz với stamp bù trễ (+0.05s).

Kết quả: RViz2 và Nav2 luôn hiển thị robot đúng 100% vị trí thực trong Gazebo,
triệt tiêu toàn bộ sai số tích lũy do trượt bánh xe (wheel slip) khi va quẹt.
AMCL vẫn chạy cho mục đích hiển thị particle swarm và topic /amcl_pose,
nhưng KHÔNG phát TF (tf_broadcast: false trong localization.yaml).
"""

import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np


def quat_to_matrix(x, y, z, w):
    """Quaternion -> 4x4 homogeneous transformation matrix (rotation only)."""
    m = np.eye(4)
    m[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    m[0, 1] = 2.0 * (x * y - z * w)
    m[0, 2] = 2.0 * (x * z + y * w)
    m[1, 0] = 2.0 * (x * y + z * w)
    m[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    m[1, 2] = 2.0 * (y * z - x * w)
    m[2, 0] = 2.0 * (x * z - y * w)
    m[2, 1] = 2.0 * (y * z + x * w)
    m[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return m


def matrix_to_quat(m):
    """4x4 homogeneous matrix -> quaternion (x, y, z, w)."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def pose_to_matrix(tx, ty, tz, qx, qy, qz, qw):
    """Translation + quaternion -> 4x4 homogeneous transformation matrix."""
    m = quat_to_matrix(qx, qy, qz, qw)
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m


class SimGroundTruthOdom(Node):
    def __init__(self):
        super().__init__('sim_ground_truth_odom')

        # Ground truth pose từ Gazebo (map_T_base)
        self.gt_matrix = None  # 4x4 numpy array
        self._first_sync_logged = False

        # TF broadcaster cho map -> odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # TF buffer & listener để đọc odom -> base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe ground truth pose từ Gazebo bridge
        self.gt_sub = self.create_subscription(
            TFMessage,
            '_internal/sim_ground_truth_pose',
            self.gt_callback,
            10
        )

        # Timer phát map -> odom ở 30Hz (đáp ứng realtime cho Nav2 và RViz)
        self.create_timer(1.0 / 30.0, self.publish_map_odom)

        self.get_logger().info(
            "SimGroundTruthOdom: Khởi động. Sẵn sàng đồng bộ vị trí thực từ Gazebo sang RViz2."
        )

    def gt_callback(self, msg: TFMessage):
        """Nhận ground-truth pose của robot từ Gazebo."""
        for t in msg.transforms:
            child = t.child_frame_id.strip()
            # Khớp chính xác tên model 'turtlebot4' (hoặc link gốc)
            if child == 'turtlebot4' or child.endswith('/turtlebot4') or child.endswith('::turtlebot4'):
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                self.gt_matrix = pose_to_matrix(tx, ty, tz, qx, qy, qz, qw)

                if not self._first_sync_logged:
                    self.get_logger().info(
                        f"🎯 ĐÃ KẾT NỐI GAZEBO GROUND TRUTH: Robot đang tại (x={tx:.2f}, y={ty:.2f}, z={tz:.2f}). "
                        "RViz và Gazebo được đồng bộ tuyệt đối 100%!"
                    )
                    self._first_sync_logged = True
                break

    def publish_map_odom(self):
        """Tính và phát biến đổi map -> odom từ ground truth."""
        if self.gt_matrix is None:
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        # Đọc odom -> base_link từ diffdrive_controller
        try:
            odom_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception:
            return

        t = odom_base.transform
        odom_T_base = pose_to_matrix(
            t.translation.x, t.translation.y, t.translation.z,
            t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
        )

        # map_T_odom = map_T_base * inverse(odom_T_base)
        try:
            odom_T_base_inv = np.linalg.inv(odom_T_base)
        except np.linalg.LinAlgError:
            return

        map_T_odom = self.gt_matrix @ odom_T_base_inv

        # Trích xuất translation & quaternion
        tx = map_T_odom[0, 3]
        ty = map_T_odom[1, 3]
        tz = map_T_odom[2, 3]
        qx, qy, qz, qw = matrix_to_quat(map_T_odom)

        # Phát TransformStamped: map -> odom với stamp bù 0.05s để tránh trễ TF
        ts = TransformStamped()
        ts.header.stamp = (now + rclpy.duration.Duration(seconds=0.05)).to_msg()
        ts.header.frame_id = 'map'
        ts.child_frame_id = 'odom'
        ts.transform.translation.x = float(tx)
        ts.transform.translation.y = float(ty)
        ts.transform.translation.z = float(tz)
        ts.transform.rotation.x = float(qx)
        ts.transform.rotation.y = float(qy)
        ts.transform.rotation.z = float(qz)
        ts.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(ts)


def main(args=None):
    rclpy.init(args=args)
    node = SimGroundTruthOdom()
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

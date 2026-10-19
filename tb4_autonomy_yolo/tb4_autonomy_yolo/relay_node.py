#!/usr/bin/env python3
"""relay_node.py - Chuyển tiếp /cmd_vel sang /diffdrive_controller/cmd_vel_unstamped.

Đảm bảo lệnh điều khiển từ Nav2 luôn được truyền thẳng tới diffdrive_controller
trong Ignition Gazebo ngay cả khi motion_control của Create 3 rơi vào trạng thái
docked hoặc reflex.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diffdrive_controller/cmd_vel_unstamped')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(Twist, in_topic, self.cb, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.get_logger().info(f'Bắt đầu Relay: {in_topic} -> {out_topic}')

    def cb(self, msg: Twist):
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

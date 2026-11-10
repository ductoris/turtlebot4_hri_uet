#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dummy_hardware_services.py:
Cung cấp các service giả lập cho phần cứng thật (RPLIDAR motor, OAK-D camera power).
Trong môi trường mô phỏng Gazebo, turtlebot4_node thường xuyên tìm kiếm các service vật lý này:
- /start_motor, /stop_motor (std_srvs/Empty)
- /oakd/start_camera, /oakd/stop_camera (std_srvs/Trigger)
- /robot_power (irobot_create_msgs/RobotPower nếu có)
- /e_stop (irobot_create_msgs/EStop nếu có)

Node này phản hồi thành công lập tức để triệt tiêu toàn bộ các thông báo lỗi:
"[ERROR] [turtlebot4_node]: Service oakd/stop_camera unavailable."
"[ERROR] [turtlebot4_node]: Service stop_motor unavailable."
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

try:
    from irobot_create_msgs.srv import RobotPower, EStop
    HAS_CREATE_MSGS = True
except ImportError:
    HAS_CREATE_MSGS = False

class DummyHardwareServices(Node):
    def __init__(self):
        super().__init__('dummy_hardware_services')

        # Dịch vụ điều khiển motor RPLIDAR vật lý
        self.srv_start_motor = self.create_service(Empty, 'start_motor', self.cb_empty)
        self.srv_stop_motor = self.create_service(Empty, 'stop_motor', self.cb_empty)
        self.srv_start_motor_abs = self.create_service(Empty, '/start_motor', self.cb_empty)
        self.srv_stop_motor_abs = self.create_service(Empty, '/stop_motor', self.cb_empty)

        # Dịch vụ điều khiển camera OAK-D vật lý
        self.srv_start_cam = self.create_service(Trigger, 'oakd/start_camera', self.cb_trigger)
        self.srv_stop_cam = self.create_service(Trigger, 'oakd/stop_camera', self.cb_trigger)
        self.srv_start_cam_abs = self.create_service(Trigger, '/oakd/start_camera', self.cb_trigger)
        self.srv_stop_cam_abs = self.create_service(Trigger, '/oakd/stop_camera', self.cb_trigger)

        # Giả lập thêm Power và EStop nếu có irobot_create_msgs
        if HAS_CREATE_MSGS:
            try:
                self.srv_power = self.create_service(RobotPower, 'robot_power', self.cb_power)
                self.srv_estop = self.create_service(EStop, 'e_stop', self.cb_estop)
            except Exception:
                pass

        self.get_logger().info("DummyHardwareServices đã sẵn sàng (đã chặn triệt để lỗi Service oakd/stop_camera và stop_motor unavailable).")

    def cb_empty(self, request, response):
        return response

    def cb_trigger(self, request, response):
        response.success = True
        response.message = "OK (Simulation Dummy Hardware Ready)"
        return response

    def cb_power(self, request, response):
        response.success = True
        response.message = "OK"
        return response

    def cb_estop(self, request, response):
        response.success = True
        response.message = "OK"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyHardwareServices()
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

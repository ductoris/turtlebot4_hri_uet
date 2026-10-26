#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_behavior_node.py:
Node dieu khien hanh vi cua TurtleBot 4 dua tren cu chi tay.
- Subscribe: /detected_gesture (std_msgs/msg/String), /odom (nav_msgs/msg/Odometry)
- Publish: /cmd_vel (geometry_msgs/msg/Twist), /robot_state (std_msgs/msg/String)
- Mock services: /start_motor, /stop_motor, /oakd/* de triet tieu loi thieu service trong Gazebo.
- May trang thai huu han (FSM) non-blocking voi timer 20 Hz.
- Toan bo thong bao va log he thong su dung TIENG VIET KHONG DAU.
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from irobot_create_msgs.msg import DockStatus
    from irobot_create_msgs.action import Undock
    CREATE3_MSGS_AVAILABLE = True
except ImportError:
    CREATE3_MSGS_AVAILABLE = False


class RobotState(Enum):
    IDLE = "IDLE"
    SELECTED_FORWARD_BACK = "SELECTED_FORWARD_BACK"
    SELECTED_CIRCLE = "SELECTED_CIRCLE"
    EXECUTING_WAVE = "EXECUTING_WAVE"
    EXECUTING_FORWARD_BACK = "EXECUTING_FORWARD_BACK"
    EXECUTING_CIRCLE = "EXECUTING_CIRCLE"


class RobotBehaviorNode(Node):
    def __init__(self):
        super().__init__('robot_behavior_node')

        # --- KHAI BAO TOAN BO THAM SO (ROS PARAMETERS) ---
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/robot_state')
        self.declare_parameter('gesture_topic', '/detected_gesture')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('use_odom', False)
        self.declare_parameter('watchdog_timeout', 2.0)
        self.declare_parameter('auto_undock', True)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.00)

        # Tham so hanh vi Tien - Lui
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('forward_distance', 0.50)
        self.declare_parameter('forward_back_cycles', 4)
        self.declare_parameter('pause_between_steps', 0.20)

        # Tham so hanh vi Vong tron
        self.declare_parameter('circle_linear_speed', 0.20)
        self.declare_parameter('circle_angular_speed', 0.40)
        self.declare_parameter('circle_duration', 15.71)

        # Tham so hanh vi Chao (Wave)
        self.declare_parameter('wave_angular_speed', 0.60)
        self.declare_parameter('wave_step_duration', 0.80)

        # Lay gia tri tham so
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        self.gesture_topic = self.get_parameter('gesture_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.use_odom = self.get_parameter('use_odom').get_parameter_value().bool_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').get_parameter_value().double_value
        self.auto_undock = self.get_parameter('auto_undock').get_parameter_value().bool_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.forward_dist = self.get_parameter('forward_distance').get_parameter_value().double_value
        self.forward_cycles = self.get_parameter('forward_back_cycles').get_parameter_value().integer_value
        self.pause_time = self.get_parameter('pause_between_steps').get_parameter_value().double_value

        self.circle_v = self.get_parameter('circle_linear_speed').get_parameter_value().double_value
        self.circle_w = self.get_parameter('circle_angular_speed').get_parameter_value().double_value
        self.circle_duration = self.get_parameter('circle_duration').get_parameter_value().double_value

        self.wave_w = self.get_parameter('wave_angular_speed').get_parameter_value().double_value
        self.wave_step_dur = self.get_parameter('wave_step_duration').get_parameter_value().double_value

        # --- MOCK SERVICES DE TRIET TIEU LOI Service start_motor / oakd unavailable ---
        # Khi chay trong simulation khong co phan cung lidar/camera that, cac service nay se tra ve thanh cong ngay
        self.mock_start_motor = self.create_service(Empty, '/start_motor', self._dummy_empty_cb)
        self.mock_stop_motor = self.create_service(Empty, '/stop_motor', self._dummy_empty_cb)
        self.mock_start_cam = self.create_service(Trigger, '/oakd/start_camera', self._dummy_trigger_cb)
        self.mock_stop_cam = self.create_service(Trigger, '/oakd/stop_camera', self._dummy_trigger_cb)

        # --- PUBLISHER & SUBSCRIBER ---
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.gesture_sub = self.create_subscription(
            String, self.gesture_topic, self.gesture_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )

        self.is_docked = False
        if CREATE3_MSGS_AVAILABLE:
            self.dock_sub = self.create_subscription(
                DockStatus, '/dock_status', self.dock_status_callback, qos_profile_sensor_data
            )
            self.undock_client = ActionClient(self, Undock, '/undock')
        else:
            self.dock_sub = None
            self.undock_client = None

        # --- BIEN TRANG THAI HE THONG ---
        self.current_state = RobotState.IDLE
        self.last_gesture = "None"
        self.last_watchdog_time = time.time()

        self.odom_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.action_steps = []
        self.current_step_index = 0
        self.step_start_time = 0.0
        self.step_start_x = 0.0
        self.step_start_y = 0.0
        self.action_description = ""

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.state_timer = self.create_timer(0.1, self.publish_robot_state)

        self.get_logger().info("robot_behavior_node da khoi tao thanh cong!")
        self.get_logger().info(f"Publish cmd_vel toi: {self.cmd_vel_topic}")
        self.get_logger().info("Trang thai ban dau: IDLE (San sang nhan cu chi)")

    def _dummy_empty_cb(self, request, response):
        return response

    def _dummy_trigger_cb(self, request, response):
        response.success = True
        response.message = "Mock Simulation OK"
        return response

    def dock_status_callback(self, msg):
        was_docked = self.is_docked
        self.is_docked = msg.is_docked
        if self.is_docked and not was_docked:
            self.get_logger().warn(
                "[CANH BAO DOCK] Robot dang o tren Dock sac Create 3. "
                "Banh xe bi khoa de bao ve. Chay lenh sau de giai phong:\n"
                "ros2 action send_goal /undock irobot_create_msgs/action/Undock {}"
            )
            if self.auto_undock:
                self.trigger_undock()

    def trigger_undock(self):
        if self.undock_client and self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("[UNDOCK] Dang gui lenh Undock de robot roi khoi dock...")
            goal_msg = Undock.Goal()
            self.undock_client.send_goal_async(goal_msg)

    def odom_callback(self, msg: Odometry):
        self.odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def gesture_callback(self, msg: String):
        gesture = msg.data.strip()
        now = time.time()
        self.last_watchdog_time = now

        if gesture == self.last_gesture:
            return

        prev_gesture = self.last_gesture
        self.last_gesture = gesture

        if gesture == "None":
            return

        self.get_logger().info(f"[NHAN CU CHI] '{prev_gesture}' -> '{gesture}'")

        # 1. Open_Palm: Dung khan cap
        if gesture == "Open_Palm":
            self.get_logger().warn("[KHAN CAP] Nhan cu chi Open_Palm -> DUNG NGAY LAP TUC!")
            self.emergency_stop()
            self.current_state = RobotState.IDLE
            return

        # 2. Closed_Fist: Huy ve IDLE
        if gesture == "Closed_Fist":
            self.get_logger().info("[HUY] Nhan cu chi Closed_Fist -> Huy toan bo hanh vi, ve IDLE.")
            self.stop_robot()
            self.action_steps.clear()
            self.current_state = RobotState.IDLE
            return

        # 3. May trang thai theo tung che do
        if self.current_state == RobotState.IDLE:
            if gesture == "Wave":
                self.get_logger().info("[KICH HOAT] Nhan Wave -> Chao lai ngay!")
                self.start_wave_action()
            elif gesture == "Pointing_Up":
                self.get_logger().info("[CHON] Pointing_Up -> Chon hanh vi TIEN-LUI (Cho Thumb_Up de bat dau).")
                self.current_state = RobotState.SELECTED_FORWARD_BACK
            elif gesture == "Victory":
                self.get_logger().info("[CHON] Victory -> Chon hanh vi VONG TRON (Cho Thumb_Up de bat dau).")
                self.current_state = RobotState.SELECTED_CIRCLE
            elif gesture == "Thumb_Up":
                self.get_logger().warn("[CANH BAO] Thumb_Up bi bo qua: Chua chon hanh vi! Hay giơ Pointing_Up hoac Victory truoc.")

        elif self.current_state == RobotState.SELECTED_FORWARD_BACK:
            if gesture == "Thumb_Up":
                self.get_logger().info("[XAC NHAN] Thumb_Up -> Bat dau chay hanh vi TIEN-LUI 4 chu ky!")
                self.start_forward_back_action()
            elif gesture == "Victory":
                self.get_logger().info("[DOI LUA CHON] Chuyen tu Tien-Lui sang VONG TRON.")
                self.current_state = RobotState.SELECTED_CIRCLE
            elif gesture == "Wave":
                self.get_logger().info("[HUY CHON] Chuyen sang cu chi Wave chao lai.")
                self.start_wave_action()

        elif self.current_state == RobotState.SELECTED_CIRCLE:
            if gesture == "Thumb_Up":
                self.get_logger().info("[XAC NHAN] Thumb_Up -> Bat dau chay hanh vi VONG TRON (1 vong)!")
                self.start_circle_action()
            elif gesture == "Pointing_Up":
                self.get_logger().info("[DOI LUA CHON] Chuyen tu Vong tron sang TIEN-LUI.")
                self.current_state = RobotState.SELECTED_FORWARD_BACK
            elif gesture == "Wave":
                self.get_logger().info("[HUY CHON] Chuyen sang cu chi Wave chao lai.")
                self.start_wave_action()

        elif self.is_executing():
            self.get_logger().warn(
                f"[BAN] Robot dang thuc hien '{self.action_description}'. "
                f"Bo qua cu chi '{gesture}'. (Dung Open_Palm de dung hoac Closed_Fist de huy)."
            )

    def is_executing(self):
        return self.current_state in [
            RobotState.EXECUTING_WAVE,
            RobotState.EXECUTING_FORWARD_BACK,
            RobotState.EXECUTING_CIRCLE
        ]

    def check_dock_before_action(self):
        if self.is_docked:
            self.get_logger().warn("[CANH BAO] Robot dang o tren Dock! Gui lenh Undock ngay...")
            self.trigger_undock()

    def start_wave_action(self):
        self.check_dock_before_action()
        step_d = self.wave_step_dur
        w = self.wave_w
        self.action_steps = [
            {'type': 'time', 'v': 0.0, 'w': w, 'duration': step_d, 'name': 'Vay chao 1: Xoay trai'},
            {'type': 'time', 'v': 0.0, 'w': -w, 'duration': step_d * 2.0, 'name': 'Vay chao 1: Xoay phai'},
            {'type': 'time', 'v': 0.0, 'w': w, 'duration': step_d * 2.0, 'name': 'Vay chao 2: Xoay trai'},
            {'type': 'time', 'v': 0.0, 'w': -w, 'duration': step_d * 2.0, 'name': 'Vay chao 2: Xoay phai'},
            {'type': 'time', 'v': 0.0, 'w': w, 'duration': step_d * 2.0, 'name': 'Vay chao 3: Xoay trai'},
            {'type': 'time', 'v': 0.0, 'w': -w, 'duration': step_d, 'name': 'Vay chao 3: Ve thang huong'}
        ]
        self.action_description = "Wave (Vay chao 3 nhip)"
        self.current_state = RobotState.EXECUTING_WAVE
        self.init_first_step()

    def start_forward_back_action(self):
        self.check_dock_before_action()
        steps = []
        step_duration = self.forward_dist / self.forward_speed

        for i in range(1, self.forward_cycles + 1):
            steps.append({
                'type': 'distance_or_time',
                'v': self.forward_speed,
                'w': 0.0,
                'target_dist': self.forward_dist,
                'duration': step_duration,
                'name': f"Chu ky {i}/{self.forward_cycles}: Tien {self.forward_dist:.2f}m"
            })
            steps.append({
                'type': 'time',
                'v': 0.0,
                'w': 0.0,
                'duration': self.pause_time,
                'name': f"Chu ky {i}/{self.forward_cycles}: Dung nghi"
            })
            steps.append({
                'type': 'distance_or_time',
                'v': -self.forward_speed,
                'w': 0.0,
                'target_dist': self.forward_dist,
                'duration': step_duration,
                'name': f"Chu ky {i}/{self.forward_cycles}: Lui {self.forward_dist:.2f}m"
            })
            steps.append({
                'type': 'time',
                'v': 0.0,
                'w': 0.0,
                'duration': self.pause_time,
                'name': f"Chu ky {i}/{self.forward_cycles}: Dung nghi"
            })

        self.action_steps = steps
        self.action_description = f"Tien - Lui ({self.forward_cycles} chu ky)"
        self.current_state = RobotState.EXECUTING_FORWARD_BACK
        self.init_first_step()

    def start_circle_action(self):
        self.check_dock_before_action()
        self.action_steps = [
            {
                'type': 'time',
                'v': self.circle_v,
                'w': self.circle_w,
                'duration': self.circle_duration,
                'name': f"Chay vong tron R={(self.circle_v/self.circle_w):.2f}m"
            },
            {
                'type': 'time',
                'v': 0.0,
                'w': 0.0,
                'duration': 0.5,
                'name': "Hoan tat vong tron"
            }
        ]
        self.action_description = "Vong tron (1 vong)"
        self.current_state = RobotState.EXECUTING_CIRCLE
        self.init_first_step()

    def init_first_step(self):
        self.current_step_index = 0
        self.step_start_time = time.time()
        self.step_start_x = self.current_x
        self.step_start_y = self.current_y
        if self.action_steps:
            first = self.action_steps[0]
            self.get_logger().info(f"[BUOC 1/{len(self.action_steps)}] {first['name']}")

    def control_loop(self):
        now = time.time()

        if self.is_executing() and (now - self.last_watchdog_time > self.watchdog_timeout):
            self.get_logger().error(
                f"[WATCHDOG] Qua {self.watchdog_timeout:.1f}s khong co tin hieu cu chi! Dung robot an toan."
            )
            self.emergency_stop()
            self.current_state = RobotState.IDLE
            return

        if not self.is_executing():
            return

        if self.current_step_index >= len(self.action_steps):
            self.get_logger().info(f"[HOAN THANH] Xong toan bo {self.action_description} -> Ve IDLE.")
            self.stop_robot()
            self.current_state = RobotState.IDLE
            self.action_steps.clear()
            return

        current_step = self.action_steps[self.current_step_index]
        step_done = False
        elapsed = now - self.step_start_time

        if current_step['type'] == 'distance_or_time':
            if self.use_odom and self.odom_received:
                dx = self.current_x - self.step_start_x
                dy = self.current_y - self.step_start_y
                dist = math.hypot(dx, dy)
                if dist >= current_step['target_dist']:
                    step_done = True
            if elapsed >= current_step['duration']:
                step_done = True

        elif current_step['type'] == 'time':
            if elapsed >= current_step['duration']:
                step_done = True

        if step_done:
            self.current_step_index += 1
            self.step_start_time = now
            self.step_start_x = self.current_x
            self.step_start_y = self.current_y

            if self.current_step_index < len(self.action_steps):
                nxt = self.action_steps[self.current_step_index]
                self.get_logger().info(
                    f"[BUOC {self.current_step_index + 1}/{len(self.action_steps)}] {nxt['name']}"
                )
            return

        cmd_v = self.clamp(current_step['v'], -self.max_linear_speed, self.max_linear_speed)
        cmd_w = self.clamp(current_step['w'], -self.max_angular_speed, self.max_angular_speed)
        self.publish_cmd_vel(cmd_v, cmd_w)

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)

    def emergency_stop(self):
        self.action_steps.clear()
        for _ in range(3):
            self.publish_cmd_vel(0.0, 0.0)

    def publish_robot_state(self):
        msg = String()
        if self.current_state == RobotState.IDLE:
            state_str = "IDLE (San sang nhan lenh)"
        elif self.current_state == RobotState.SELECTED_FORWARD_BACK:
            state_str = "DA CHON: TIEN-LUI (Cho Thumb_Up)"
        elif self.current_state == RobotState.SELECTED_CIRCLE:
            state_str = "DA CHON: VONG TRON (Cho Thumb_Up)"
        elif self.is_executing():
            total = len(self.action_steps)
            curr = min(self.current_step_index + 1, total)
            step_name = self.action_steps[self.current_step_index]['name'] if self.action_steps else ""
            state_str = f"DANG CHAY: {self.action_description} [{curr}/{total} - {step_name}]"
        else:
            state_str = self.current_state.value

        msg.data = state_str
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Tat robot_behavior_node... Gui lenh dung robot.")
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
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

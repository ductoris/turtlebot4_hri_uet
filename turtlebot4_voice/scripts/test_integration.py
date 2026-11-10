#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_integration.py:
Kịch bản kiểm thử tích hợp tự động cho Bài thực hành 7:
1. "Đi đến khu vực A" -> Đích xác định là Phòng học (A).
2. "Di chuyển từ khu vực A đến khu vực B" -> Đích bước 1 là Phòng học (A), sau đó Phòng thí nghiệm (B).
3. "Đi đến phòng thí nghiệm" -> Đích xác định là Phòng thí nghiệm (B).
4. "Đi đến khu vực Z" (không tồn tại) -> Báo lỗi vùng không hợp lệ, không gửi goal, robot đứng yên.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class IntegrationTester(Node):
    def __init__(self):
        super().__init__('voice_nav_integration_tester')

        self.cmd_pub = self.create_publisher(String, '/voice_command_text', 10)
        self.status_sub = self.create_subscription(String, '/voice_nav/status', self.status_cb, 10)

        self.received_statuses = []

    def status_cb(self, msg: String):
        self.received_statuses.append(msg.data)
        print(f"  [STATUS NHẬN ĐƯỢC]: {msg.data}")

def run_tests():
    rclpy.init()
    tester = IntegrationTester()

    # Chờ các kết nối topic hoàn tất
    time.sleep(1.0)
    for _ in range(5):
        rclpy.spin_once(tester, timeout_sec=0.1)

    test_commands = [
        ("Đi đến khu vực A", "Phòng học (A)"),
        ("Di chuyển từ khu vực A đến khu vực B", "Phòng học (A)"),
        ("Đi đến phòng thí nghiệm", "Phòng thí nghiệm (B)"),
        ("Đi đến khu vực Z", "Lỗi: Không nhận diện được khu vực"),
    ]

    results = []

    print("\n" + "="*60)
    print(" BẮT ĐẦU KIỂM THỬ TÍCH HỢP HỆ THỐNG ĐIỀU HƯỚNG GIỌNG NÓI")
    print("="*60)

    for cmd, expected_keyword in test_commands:
        print(f"\n--- TEST CASE: Phát lệnh: \"{cmd}\" ---")
        tester.received_statuses.clear()

        # Phát lệnh
        msg = String()
        msg.data = cmd
        tester.cmd_pub.publish(msg)

        # Lắng nghe phản hồi
        start_t = time.time()
        matched = False
        while time.time() - start_t < 3.0:
            rclpy.spin_once(tester, timeout_sec=0.1)
            for status in tester.received_statuses:
                if expected_keyword in status:
                    matched = True
                    break
            if matched:
                break

        if matched:
            print(f"  ==> KẾT QUẢ: ĐẠT (PASS) [Khớp từ khóa: '{expected_keyword}']")
            results.append((cmd, expected_keyword, "THÀNH CÔNG (PASS)"))
        else:
            print(f"  ==> KẾT QUẢ: KHÔNG ĐẠT (FAIL) [Chưa thấy '{expected_keyword}']")
            results.append((cmd, expected_keyword, "THẤT BÀI (FAIL)"))

        time.sleep(1.0)

    print("\n" + "="*60)
    print(" BẢNG TỔNG HỢP KẾT QUẢ KIỂM THỬ TÍCH HỢP")
    print("="*60)
    print(f"{'Câu lệnh giọng nói':<40} | {'Từ khóa kỳ vọng':<30} | {'Kết quả'}")
    print("-" * 85)
    for c, exp, res in results:
        print(f"{c:<40} | {exp:<30} | {res}")
    print("="*60 + "\n")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_tests()

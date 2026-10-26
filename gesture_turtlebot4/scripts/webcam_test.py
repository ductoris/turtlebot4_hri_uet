#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
webcam_test.py: Script kiem tra webcam laptop bang OpenCV.
Toan bo text hien thi tren terminal va HUD su dung TIENG VIET KHONG DAU.
Nhan 'q' tren cua so de thoat.
"""

import cv2
import sys
import time


def main():
    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print(f"[ERROR] Chi so camera khong hop le: {sys.argv[1]}. Su dung mac dinh 0.")

    print(f"[INFO] Dang thu mo camera index: {camera_index}...")
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"[ERROR] Khong the mo webcam tai /dev/video{camera_index}!")
        print("--- GOI Y XU LY LOI CAMERA ---")
        print("1. Kiem tra danh sach camera: ls -l /dev/video*")
        print("2. Quyen truy cap: sudo usermod -a -G video $USER (dang nhap lai sau khi go)")
        print("3. Kiem tra xem co app khac (Zoom, Meet, Cheese) dang chiem camera khong.")
        return 1

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_prop = cap.get(cv2.CAP_PROP_FPS)

    print(f"[SUCCESS] Da mo thanh cong camera!")
    print(f" - Do phan giai: {width} x {height}")
    print(f" - FPS cam bien: {fps_prop:.1f}")
    print(" - Nhan 'q' tren cua so video de thoat.\n")

    prev_time = time.time()
    frame_count = 0
    fps = 0.0
    window_name = "Webcam Test - Nhan 'q' de thoat"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        frame = cv2.flip(frame, 1)
        frame_count += 1
        curr_time = time.time()
        time_diff = curr_time - prev_time
        if time_diff >= 1.0:
            fps = frame_count / time_diff
            frame_count = 0
            prev_time = curr_time

        cv2.putText(
            frame, f"Webcam OK | FPS: {fps:.1f} | Res: {width}x{height}",
            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            frame, "Nhan 'q' de thoat",
            (15, height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print("[INFO] Nguoi dung nhan thoat.")
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())

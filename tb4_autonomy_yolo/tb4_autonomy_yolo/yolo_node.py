#!/usr/bin/env python3
"""yolo_node.py - Nhận diện vật thể YOLOv8n từ camera OAK-D của TurtleBot 4.

Subscribe : <image_topic>  (sensor_msgs/Image)   mặc định /oakd/rgb/preview/image_raw
Publish   : /yolo/image_annotated (sensor_msgs/Image, bgr8) - ảnh đã vẽ bbox

Tham số ROS (đổi bằng --ros-args -p ten:=gia_tri):
  image_topic      (str)   /oakd/rgb/preview/image_raw
  output_topic     (str)   /yolo/image_annotated
  model_path       (str)   yolov8n.pt   (tự tải ở lần chạy đầu)
  conf_threshold   (float) 0.40
  imgsz            (int)   640
  show_window      (bool)  True   (tắt nếu chạy không có màn hình)
  print_period_sec (float) 0.5    (giới hạn tần suất in terminal)

Chạy:
  ros2 run tb4_autonomy_yolo yolo_node --ros-args -p use_sim_time:=true
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import torch
    torch.set_num_threads(2)
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'Chưa cài ultralytics hoặc torch. Chạy: pip3 install ultralytics torch') from exc


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ---- Tham số ------------------------------------------------------
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('output_topic', '/yolo/image_annotated')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.40)
        self.declare_parameter('imgsz', 480)
        self.declare_parameter('show_window', True)
        self.declare_parameter('print_period_sec', 0.5)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_path = self.get_parameter('model_path').value
        self.conf = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.print_period = float(self.get_parameter('print_period_sec').value)

        # ---- Model ----------------------------------------------------------
        self.get_logger().info(f'Đang nạp mô hình {model_path} (PyTorch threads=2, imgsz={self.imgsz}) ...')
        self.model = YOLO(model_path)
        self.names = self.model.names
        self.bridge = CvBridge()
        self.last_print = 0.0
        self.last_infer = 0.0
        self.busy = False  # bỏ frame đến khi đang suy luận (tránh trễ tích luỹ)

        # ---- ROS I/O --------------------------------------------------------
        # Camera là dữ liệu cảm biến: BEST_EFFORT + queue nhỏ, tương thích với
        # cả publisher RELIABLE lẫn BEST_EFFORT.
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(f'Subscribe: {image_topic} | Publish: {output_topic}')

    # ------------------------------------------------------------------------
    def image_callback(self, msg: Image):
        now = time.time()
        if self.busy or (now - self.last_infer < 0.12):
            return
        self.busy = True
        self.last_infer = now
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge lỗi: {e}')
            self.busy = False
            return

        results = self.model.predict(
            frame, conf=self.conf, imgsz=self.imgsz, verbose=False)[0]

        detections = []
        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            label = f'{self.names[cls_id]} {conf:.2f}'
            detections.append((self.names[cls_id], conf))

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            top = max(y1 - th - base, 0)
            cv2.rectangle(frame, (x1, top), (x1 + tw, top + th + base), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, top + th), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)

        # In terminal (giới hạn tần suất)
        now = time.time()
        if detections and now - self.last_print >= self.print_period:
            self.last_print = now
            text = ', '.join(f'{n} ({c * 100:.1f}%)' for n, c in detections)
            self.get_logger().info(f'Phát hiện {len(detections)} vật thể: {text}')

        # Publish ảnh đã vẽ (giữ nguyên header để đồng bộ thời gian/frame)
        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.pub.publish(out)

        if self.show_window:
            cv2.imshow('YOLOv8n - TurtleBot 4', frame)
            cv2.waitKey(1)
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

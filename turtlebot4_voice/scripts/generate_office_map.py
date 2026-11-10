#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_office_map.py:
Tạo bản đồ pgm và yaml chuẩn khớp chính xác 100% với hình học của worlds/office.sdf.
Bản đồ dùng cho Nav2 (map_server, costmaps, AMCL) và làm đầu vào cho bài toán phân vùng.
"""

import os
import cv2
import numpy as np
import yaml

def world_to_pixel(x, y, origin_x, origin_y, resolution, height_px):
    """
    Chuyển tọa độ world (x, y) sang pixel ảnh OpenCV (col_u, row_v).
    Lưu ý: row v=0 nằm ở mép trên cùng của ảnh (y max),
    trong khi tọa độ map ROS có y=0 ở mép dưới.
    """
    u = int(round((x - origin_x) / resolution))
    v = height_px - 1 - int(round((y - origin_y) / resolution))
    return u, v

def generate_map():
    resolution = 0.05  # 5 cm/pixel
    origin_x = -5.5
    origin_y = -4.5
    width_m = 11.0     # [-5.5, 5.5]
    height_m = 9.0     # [-4.5, 4.5]

    w_px = int(round(width_m / resolution))   # 220 px
    h_px = int(round(height_m / resolution))  # 180 px

    # Khởi tạo toàn bộ bản đồ là vùng chưa biết (unknown: 205)
    img = np.full((h_px, w_px), 205, dtype=np.uint8)

    # Tô vùng trống bên trong 4 bức tường ngoài (free: 254)
    # Tường ngoài từ x = [-4.925, 4.925], y = [-3.925, 3.925]
    u1, v1 = world_to_pixel(-4.925, 3.925, origin_x, origin_y, resolution, h_px)
    u2, v2 = world_to_pixel(4.925, -3.925, origin_x, origin_y, resolution, h_px)
    cv2.rectangle(img, (u1, v1), (u2, v2), 254, -1)

    # Hàm vẽ hộp hình chữ nhật (occupied: 0)
    def draw_box(cx, cy, sx, sy):
        x_min, x_max = cx - sx / 2.0, cx + sx / 2.0
        y_min, y_max = cy - sy / 2.0, cy + sy / 2.0
        c1, r1 = world_to_pixel(x_min, y_max, origin_x, origin_y, resolution, h_px)
        c2, r2 = world_to_pixel(x_max, y_min, origin_x, origin_y, resolution, h_px)
        cv2.rectangle(img, (c1, r1), (c2, r2), 0, -1)

    # Hàm vẽ hình trụ (occupied: 0)
    def draw_cylinder(cx, cy, radius):
        c, r = world_to_pixel(cx, cy, origin_x, origin_y, resolution, h_px)
        rad_px = int(round(radius / resolution))
        cv2.circle(img, (c, r), rad_px, 0, -1)

    # 1. Bức tường bao ngoài (occupied: 0)
    draw_box(0.0, 4.0, 10.0, 0.15)    # North
    draw_box(0.0, -4.0, 10.0, 0.15)   # South
    draw_box(-5.0, 0.0, 0.15, 8.0)    # West
    draw_box(5.0, 0.0, 0.15, 8.0)     # East

    # 2. Tường ngăn giữa Phòng A và B
    draw_box(-3.1, 0.0, 3.8, 0.15)

    # 3. Tường ngăn phía Tây (phòng A, B với Hành lang), chừa 2 cửa rộng 1.30m (y: 1.35->2.65 và y: -2.65->-1.35)
    draw_box(-1.2, 3.325, 0.15, 1.35)   # Trên cửa A (y: 2.65 -> 4.0)
    draw_box(-1.2, 0.0, 0.15, 2.70)     # Giữa cửa A và B (y: -1.35 -> 1.35)
    draw_box(-1.2, -3.325, 0.15, 1.35)  # Dưới cửa B (y: -4.0 -> -2.65)

    # 4. Tường ngăn phía Đông (phòng C với Hành lang), chừa 1 cửa rộng 1.30m (y: -0.65 -> 0.65)
    draw_box(1.2, 2.325, 0.15, 3.35)    # Trên cửa C (y: 0.65 -> 4.0)
    draw_box(1.2, -2.325, 0.15, 3.35)   # Dưới cửa C (y: -4.0 -> -0.65)

    # 5. Đồ vật Phòng A (Phòng học)
    draw_box(-3.3, 2.8, 1.2, 0.6)       # Bàn 1
    draw_cylinder(-3.3, 3.4, 0.2)       # Ghế 1
    draw_box(-3.3, 1.0, 1.2, 0.6)       # Bàn 2

    # 6. Đồ vật Phòng B (Phòng thí nghiệm)
    draw_box(-3.3, -2.8, 1.2, 0.6)      # Bàn lab
    draw_box(-4.0, -1.0, 0.5, 0.5)      # Thùng hộp

    # 7. Đồ vật Phòng C (Phòng họp / Kho)
    draw_cylinder(3.2, 1.5, 0.6)        # Bàn họp tròn
    draw_box(3.5, -2.5, 0.8, 0.8)       # Kệ chứa đồ

    # Thư mục đích
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    maps_dir = os.path.join(pkg_dir, 'maps')
    report_dir = os.path.join(pkg_dir, 'report')
    os.makedirs(maps_dir, exist_ok=True)
    os.makedirs(report_dir, exist_ok=True)

    pgm_path = os.path.join(maps_dir, 'office_map.pgm')
    yaml_path = os.path.join(maps_dir, 'office_map.yaml')

    # Lưu ảnh PGM
    cv2.imwrite(pgm_path, img)

    # Lưu cấu hình YAML
    map_meta = {
        'image': 'office_map.pgm',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(map_meta, f, sort_keys=False)

    # Sao chép vào report/ theo yêu cầu bàn giao
    cv2.imwrite(os.path.join(report_dir, 'office_map.pgm'), img)
    with open(os.path.join(report_dir, 'office_map.yaml'), 'w', encoding='utf-8') as f:
        yaml.dump(map_meta, f, sort_keys=False)

    print(f"Map successfully generated:")
    print(f"  PGM:  {pgm_path} ({w_px}x{h_px})")
    print(f"  YAML: {yaml_path}")

if __name__ == '__main__':
    generate_map()

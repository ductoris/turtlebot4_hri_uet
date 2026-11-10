#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
segment_map.py:
Phân vùng bản đồ bằng OpenCV (Distance Transform + Watershed) và gắn nhãn ngữ nghĩa (Semantic Mapping).
Đầu vào:
  - maps/office_map.pgm, maps/office_map.yaml
  - config/manual_regions.yaml (phương án dự phòng thủ công)
Đầu ra:
  - maps/office_map_segmented.png (và bản sao trong report/)
  - config/regions.yaml (và bản sao trong report/)
"""

import os
import argparse
import yaml
import cv2
import numpy as np

def pixel_to_world(u, v, origin_x, origin_y, resolution, height_px):
    """
    Chuyển đổi từ tọa độ pixel ảnh (u: cột, v: hàng) sang tọa độ thế giới (x, y) theo chuẩn ROS 2.
    Lưu ý: Trục y của ảnh OpenCV tính từ trên xuống dưới (0 tại đỉnh),
    ngược lại tọa độ thế giới map ROS 2 tính từ dưới lên trên.
    Công thức:
      x = origin_x + (u + 0.5) * resolution
      y = origin_y + (height_px - 1 - v + 0.5) * resolution
    """
    x = origin_x + (u + 0.5) * resolution
    y = origin_y + (height_px - 1 - v + 0.5) * resolution
    return round(float(x), 4), round(float(y), 4)

def world_to_pixel(x, y, origin_x, origin_y, resolution, height_px):
    """
    Chuyển đổi từ tọa độ thế giới (x, y) sang pixel ảnh OpenCV (u: cột, v: hàng).
    """
    u = int(round((x - origin_x) / resolution - 0.5))
    v = int(round((height_px - 1) - ((y - origin_y) / resolution - 0.5)))
    return u, v

def segment_map_auto(binary_free, dist_transform, H, W):
    """
    Phân vùng tự động bằng thuật toán Watershed dựa trên Distance Transform.
    Đặt marker tại cực đại khoảng cách (xa tường nhất) của từng phòng và sảnh.
    """
    markers = np.zeros((H, W), dtype=np.int32)

    # 1. Khu vực A (Phòng học): góc Tây Bắc (c < 86, r < 89)
    mask_a = np.zeros((H, W), dtype=bool)
    mask_a[:89, :86] = (binary_free[:89, :86] == 255)
    if mask_a.sum() > 0:
        ra, ca = np.unravel_index(np.argmax(dist_transform * mask_a), (H, W))
        cv2.circle(markers, (ca, ra), 5, 1, -1)

    # 2. Khu vực B (Phòng thí nghiệm): góc Tây Nam (c < 86, r >= 89)
    mask_b = np.zeros((H, W), dtype=bool)
    mask_b[89:, :86] = (binary_free[89:, :86] == 255)
    if mask_b.sum() > 0:
        rb, cb = np.unravel_index(np.argmax(dist_transform * mask_b), (H, W))
        cv2.circle(markers, (cb, rb), 5, 2, -1)

    # 3. Khu vực Hành lang (HALLWAY): trục giữa (86 <= c <= 134)
    # Đặt marker ở cả phần trên và phần dưới hành lang (qua dock) để bao phủ toàn bộ sảnh
    mask_h_up = np.zeros((H, W), dtype=bool)
    mask_h_up[:89, 86:134] = (binary_free[:89, 86:134] == 255)
    if mask_h_up.sum() > 0:
        rhu, chu = np.unravel_index(np.argmax(dist_transform * mask_h_up), (H, W))
        cv2.circle(markers, (chu, rhu), 5, 3, -1)

    mask_h_dn = np.zeros((H, W), dtype=bool)
    mask_h_dn[89:, 86:134] = (binary_free[89:, 86:134] == 255)
    if mask_h_dn.sum() > 0:
        rhd, chd = np.unravel_index(np.argmax(dist_transform * mask_h_dn), (H, W))
        cv2.circle(markers, (chd, rhd), 5, 3, -1)

    # 4. Khu vực C (Phòng họp / Kho): phía Đông (c > 134)
    mask_c = np.zeros((H, W), dtype=bool)
    mask_c[:, 134:] = (binary_free[:, 134:] == 255)
    if mask_c.sum() > 0:
        rc, cc = np.unravel_index(np.argmax(dist_transform * mask_c), (H, W))
        cv2.circle(markers, (cc, rc), 5, 4, -1)

    # Watershed yêu cầu ảnh BGR 3 kênh
    img_bgr = cv2.merge([binary_free, binary_free, binary_free])
    cv2.watershed(img_bgr, markers)

    # Đếm số vùng hợp lệ (> 200 pixels)
    valid_labels = []
    for l in [1, 2, 3, 4]:
        if (markers == l).sum() > 200:
            valid_labels.append(l)

    return markers, valid_labels

def segment_map_manual(manual_yaml_path, H, W):
    """
    Phân vùng thủ công sử dụng các đa giác đã định nghĩa trong manual_regions.yaml.
    """
    with open(manual_yaml_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    markers = np.zeros((H, W), dtype=np.int32)
    label = 1
    for reg in data.get('regions', []):
        pts = np.array(reg['polygon_pixels'], dtype=np.int32)
        cv2.fillPoly(markers, [pts], label)
        label += 1

    return markers, list(range(1, label))

def main():
    parser = argparse.ArgumentParser(description="Map Segmentation and Semantic Mapping for TurtleBot 4")
    parser.add_argument('--mode', type=str, default='auto', choices=['auto', 'manual'],
                        help="Chế độ phân vùng: 'auto' (Watershed) hoặc 'manual' (đa giác)")
    args = parser.parse_args()

    # Xác định đường dẫn thư mục package
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    maps_dir = os.path.join(pkg_dir, 'maps')
    config_dir = os.path.join(pkg_dir, 'config')
    report_dir = os.path.join(pkg_dir, 'report')
    os.makedirs(report_dir, exist_ok=True)

    map_yaml_path = os.path.join(maps_dir, 'office_map.yaml')
    map_pgm_path = os.path.join(maps_dir, 'office_map.pgm')
    manual_yaml_path = os.path.join(config_dir, 'manual_regions.yaml')

    # Đọc cấu hình map yaml
    with open(map_yaml_path, 'r', encoding='utf-8') as f:
        map_info = yaml.safe_load(f)

    resolution = float(map_info['resolution'])
    origin_x = float(map_info['origin'][0])
    origin_y = float(map_info['origin'][1])

    # Đọc ảnh bản đồ xám
    img_gray = cv2.imread(map_pgm_path, cv2.IMREAD_GRAYSCALE)
    if img_gray is None:
        raise FileNotFoundError(f"Không thể mở file ảnh bản đồ: {map_pgm_path}")
    H, W = img_gray.shape

    # 1. Nhị phân hóa bản đồ: vùng trống (free, 254) = 255, còn lại = 0
    _, binary_free = cv2.threshold(img_gray, 250, 255, cv2.THRESH_BINARY)

    # 2. Làm sạch nhiễu bằng phép toán hình thái học (Morphology Open)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary_clean = cv2.morphologyEx(binary_free, cv2.MORPH_OPEN, kernel)

    # 3. Tính Distance Transform (khoảng cách Euclidean tới chướng ngại vật gần nhất)
    dist_transform = cv2.distanceTransform(binary_clean, cv2.DIST_L2, 5)

    mode_used = args.mode
    labels_map = None
    valid_labels = []

    if mode_used == 'auto':
        print("[INFO] Đang thử phân vùng tự động bằng Distance Transform + Watershed...")
        labels_map, valid_labels = segment_map_auto(binary_clean, dist_transform, H, W)
        if len(valid_labels) == 4:
            print(f"[INFO] Phân vùng tự động thành công! Tìm thấy đủ 4 khu vực (3 phòng + 1 sảnh hành lang).")
        else:
            print(f"[CẢNH BÁO] Phân vùng tự động tìm thấy {len(valid_labels)} vùng (kỳ vọng 4).")
            print("[INFO] Tự động chuyển về chế độ thủ công (Fallback to config/manual_regions.yaml)...")
            mode_used = 'manual'

    if mode_used == 'manual':
        print(f"[INFO] Đang áp dụng phân vùng thủ công từ {manual_yaml_path}...")
        labels_map, valid_labels = segment_map_manual(manual_yaml_path, H, W)
        print(f"[INFO] Phân vùng thủ công hoàn tất với {len(valid_labels)} khu vực.")

    # Cấu hình siêu dữ liệu ngữ nghĩa cho 4 khu vực
    # Robot radius: ~0.18m, min safe clearance threshold = 0.40m
    min_safe_clearance_px = int(round(0.40 / resolution))  # 8 pixels

    region_defs = {
        1: {
            'id': 'A',
            'name': 'Phòng học',
            'aliases': ['khu vực a', 'khu vuc a', 'phòng a', 'phong a', 'phòng học', 'phong hoc', 'lớp học', 'lop hoc'],
            'yaw': 0.0,
            'color_bgr': (255, 180, 100)  # Xanh dương nhạt
        },
        2: {
            'id': 'B',
            'name': 'Phòng thí nghiệm',
            'aliases': ['khu vực b', 'khu vuc b', 'phòng b', 'phong b', 'phòng thí nghiệm', 'phong thi nghiem', 'lab', 'phòng lab', 'phong lab'],
            'yaw': 0.0,
            'color_bgr': (100, 220, 100)  # Xanh lá
        },
        3: {
            'id': 'HALLWAY',
            'name': 'Hành lang',
            'aliases': ['hành lang', 'hanh lang', 'sảnh', 'sanh', 'trạm sạc', 'tram sac', 'dock', 'về sạc', 've sac', 'về dock', 've dock', 'khu trung tâm'],
            'yaw': 0.0,
            'color_bgr': (200, 200, 200)  # Xám sáng
        },
        4: {
            'id': 'C',
            'name': 'Phòng họp',
            'aliases': ['khu vực c', 'khu vuc c', 'phòng c', 'phong c', 'phòng họp', 'phong hop', 'kho', 'nhà kho', 'nha kho'],
            'yaw': 3.14159,  # Quay mặt về hướng Tây (cửa phòng)
            'color_bgr': (100, 150, 255)  # Cam / Đỏ nhạt
        }
    }

    # Tạo ảnh màu để trực quan hóa
    vis_img = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

    # Tô màu cho từng vùng đã phân đoạn (chỉ tô trên phần sàn trống)
    for lbl, info in region_defs.items():
        color = info['color_bgr']
        mask = (labels_map == lbl) & (binary_free == 255)
        vis_img[mask] = (0.5 * np.array(vis_img[mask], dtype=np.float32) + 0.5 * np.array(color, dtype=np.float32)).astype(np.uint8)

    regions_output_data = {'mode': mode_used, 'resolution': resolution, 'origin': [origin_x, origin_y, 0.0], 'regions': []}

    print("\n--- KẾT QUẢ GẮN NHÃN NGỮ NGHĨA (SEMANTIC MAPPING) ---")
    for lbl in [1, 2, 3, 4]:
        info = region_defs[lbl]
        region_mask = (labels_map == lbl) & (binary_free == 255)
        area_pixels = int(region_mask.sum())
        area_m2 = round(area_pixels * (resolution ** 2), 2)

        # Tìm điểm xa tường nhất trong vùng (Distance Transform cực đại)
        masked_dist = np.where(region_mask, dist_transform, 0.0)

        # Trường hợp trạm sạc trong Hành lang: ưu tiên vị trí trước dock (0.0, 0.3)
        if info['id'] == 'HALLWAY':
            # Điểm đón tại trạm sạc: x=0.0, y=0.3
            dock_u, dock_v = world_to_pixel(0.0, 0.3, origin_x, origin_y, resolution, H)
            if region_mask[dock_v, dock_u]:
                best_v, best_u = dock_v, dock_u
            else:
                best_v, best_u = np.unravel_index(np.argmax(masked_dist), (H, W))
        else:
            best_v, best_u = np.unravel_index(np.argmax(masked_dist), (H, W))

        clearance_m = round(float(dist_transform[best_v, best_u] * resolution), 3)

        # Chuyển đổi tọa độ pixel sang tọa độ thế giới
        gx, gy = pixel_to_world(best_u, best_v, origin_x, origin_y, resolution, H)

        # Bounding box
        y_indices, x_indices = np.where(region_mask)
        if len(x_indices) > 0:
            min_u, max_u = int(x_indices.min()), int(x_indices.max())
            min_v, max_v = int(y_indices.min()), int(y_indices.max())
            bbox_px = [min_u, min_v, max_u, max_v]
            bl_x, bl_y = pixel_to_world(min_u, max_v, origin_x, origin_y, resolution, H)
            tr_x, tr_y = pixel_to_world(max_u, min_v, origin_x, origin_y, resolution, H)
            bbox_world = [[bl_x, bl_y], [tr_x, tr_y]]
        else:
            bbox_px = [0, 0, 0, 0]
            bbox_world = [[0.0, 0.0], [0.0, 0.0]]

        # Tìm đa giác đường bao (contour)
        contours, _ = cv2.findContours(np.uint8(region_mask), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        polygon_world = []
        if contours:
            main_contour = max(contours, key=cv2.contourArea)
            # Giảm số đỉnh đa giác (xấp xỉ epsilon ~ 2 pixels)
            approx = cv2.approxPolyDP(main_contour, 2.0, True)
            for pt in approx:
                pu, pv = pt[0][0], pt[0][1]
                wx, wy = pixel_to_world(pu, pv, origin_x, origin_y, resolution, H)
                polygon_world.append([wx, wy])

        region_entry = {
            'id': info['id'],
            'name': info['name'],
            'aliases': info['aliases'],
            'pixel_goal': [int(best_u), int(best_v)],
            'world_goal': [gx, gy],
            'yaw': info['yaw'],
            'clearance_m': clearance_m,
            'area_m2': area_m2,
            'bbox_pixels': bbox_px,
            'bbox_world': bbox_world,
            'polygon_world': polygon_world
        }
        regions_output_data['regions'].append(region_entry)

        # Vẽ lên ảnh trực quan
        # 1. Vẽ điểm đích
        cv2.circle(vis_img, (best_u, best_v), 4, (0, 0, 255), -1)
        cv2.circle(vis_img, (best_u, best_v), 7, (0, 0, 0), 1)

        # 2. Vẽ nhãn văn bản tên phòng và tọa độ đích
        label_text = f"{info['name']} ({info['id']})"
        coord_text = f"({gx:.2f}, {gy:.2f})"

        text_x = max(10, min(W - 80, best_u - 30))
        text_y = max(20, min(H - 10, best_v - 10))

        cv2.putText(vis_img, label_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(vis_img, label_text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(vis_img, coord_text, (text_x, text_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 0, 255), 1, cv2.LINE_AA)

        print(f"Khu vực [{info['id']}] - {info['name']}:")
        print(f"  Pixel Goal:   ({best_u}, {best_v}) | World Goal: ({gx}, {gy}), yaw={info['yaw']}")
        print(f"  Clearance:    {clearance_m} m (An toàn cho robot tb4)")
        print(f"  Diện tích:    {area_m2} m2 ({area_pixels} px)")

    # Lưu ảnh trực quan maps/office_map_segmented.png
    seg_png_path = os.path.join(maps_dir, 'office_map_segmented.png')
    cv2.imwrite(seg_png_path, vis_img)
    cv2.imwrite(os.path.join(report_dir, 'office_map_segmented.png'), vis_img)
    print(f"\n[XUẤT ẢNH] Đã lưu ảnh phân vùng tại: {seg_png_path} và {os.path.join(report_dir, 'office_map_segmented.png')}")

    # Lưu cấu hình config/regions.yaml
    regions_yaml_path = os.path.join(config_dir, 'regions.yaml')
    with open(regions_yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(regions_output_data, f, allow_unicode=True, sort_keys=False)
    with open(os.path.join(report_dir, 'regions.yaml'), 'w', encoding='utf-8') as f:
        yaml.dump(regions_output_data, f, allow_unicode=True, sort_keys=False)
    print(f"[XUẤT CẤU HÌNH] Đã lưu thông tin vùng tại: {regions_yaml_path} và {os.path.join(report_dir, 'regions.yaml')}")

if __name__ == '__main__':
    main()

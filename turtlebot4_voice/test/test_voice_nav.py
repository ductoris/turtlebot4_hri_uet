#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_voice_nav.py:
Unit test kiểm thử:
1. Độ chính xác của hàm chuyển đổi tọa độ pixel <-> world (chuẩn ROS 2, đảo trục Y).
2. Hàm phân tích câu lệnh tiếng Việt (NLU): dấu, không dấu, fuzzy match, trích xuất điểm đầu/cuối, bắt lỗi khu vực không tồn tại.
"""

import sys
import os
import pytest
import math

pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)
scripts_path = os.path.join(pkg_root, 'scripts')
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)
module_path = os.path.join(pkg_root, 'turtlebot4_voice')
if module_path not in sys.path:
    sys.path.insert(0, module_path)

from segment_map import pixel_to_world, world_to_pixel
from voice_nav_node import remove_accents, clean_text, VoiceNavNode

# 1. UNIT TEST CHUYỂN ĐỔI TỌA ĐỘ PIXEL <-> WORLD
def test_coordinate_conversion_center():
    """Kiểm tra điểm gốc và tâm bản đồ."""
    ox, oy = -5.5, -4.5
    res = 0.05
    H = 180

    # Điểm gốc dưới cùng bên trái: pixel (0, H-1) -> world (ox + 0.5*res, oy + 0.5*res)
    wx, wy = pixel_to_world(0, H - 1, ox, oy, res, H)
    assert math.isclose(wx, -5.5 + 0.025, abs_tol=1e-3)
    assert math.isclose(wy, -4.5 + 0.025, abs_tol=1e-3)

    # Roundtrip: world -> pixel -> world
    pu, pv = world_to_pixel(wx, wy, ox, oy, res, H)
    assert pu == 0
    assert pv == H - 1

def test_coordinate_conversion_dock():
    """Kiểm tra vị trí Dock tại sảnh (0.0, 0.0)."""
    ox, oy = -5.5, -4.5
    res = 0.05
    H = 180

    pu, pv = world_to_pixel(0.0, 0.0, ox, oy, res, H)
    assert pu == 110  # (-5.5)/0.05 + 110 = 0.0
    assert pv == 90   # (179) - (4.5/0.05 - 0.5)

    wx, wy = pixel_to_world(pu, pv, ox, oy, res, H)
    assert abs(wx - 0.0) <= 0.03
    assert abs(wy - 0.0) <= 0.03

# 2. UNIT TEST CHUẨN HÓA VĂN BẢN VÀ XÓA DẤU TIẾNG VIỆT
def test_remove_accents():
    assert remove_accents("Phòng học") == "phong hoc"
    assert remove_accents("Phòng thí nghiệm") == "phong thi nghiem"
    assert remove_accents("Về trạm sạc") == "ve tram sac"
    assert remove_accents("Đi đến khu vực C") == "di den khu vuc c"
    assert remove_accents("Điện thoại") == "dien thoai"

# 3. UNIT TEST PARSER CÂU LỆNH NLU (Ít nhất 10 câu mẫu)
class DummyVoiceNav:
    """Mock node để test parser logic."""
    def __init__(self):
        self.regions = [
            {
                'id': 'A',
                'name': 'Phòng học',
                'aliases': ['khu vực a', 'khu vuc a', 'phòng a', 'phong a', 'phòng học', 'phong hoc', 'lớp học', 'lop hoc'],
                'world_goal': [-2.5, 2.0]
            },
            {
                'id': 'B',
                'name': 'Phòng thí nghiệm',
                'aliases': ['khu vực b', 'khu vuc b', 'phòng b', 'phong b', 'phòng thí nghiệm', 'phong thi nghiem', 'lab', 'phòng lab'],
                'world_goal': [-2.3, -1.0]
            },
            {
                'id': 'HALLWAY',
                'name': 'Hành lang',
                'aliases': ['hành lang', 'hanh lang', 'sảnh', 'sanh', 'trạm sạc', 'tram sac', 'dock', 'về sạc', 'về dock'],
                'world_goal': [0.0, 0.0]
            },
            {
                'id': 'C',
                'name': 'Phòng họp',
                'aliases': ['khu vực c', 'khu vuc c', 'phòng c', 'phong c', 'phòng họp', 'phong hop', 'kho', 'nhà kho'],
                'world_goal': [2.5, 0.0]
            }
        ]
        self.sim_thresh = 0.65

    match_region = VoiceNavNode.match_region
    parse_command = VoiceNavNode.parse_command

@pytest.fixture
def parser():
    return DummyVoiceNav()

def test_parser_sample_commands(parser):
    test_cases = [
        # (câu lệnh vào, orig_expected, dest_expected, should_succeed)
        ("Đi đến khu vực A", None, "A", True),
        ("di den khu vuc a", None, "A", True),
        ("Đi đến phòng học", None, "A", True),
        ("phong hoc", None, "A", True),
        ("Đi đến phòng thí nghiệm", None, "B", True),
        ("toi phong thi nghiem", None, "B", True),
        ("Đến phòng lab", None, "B", True),
        ("Về trạm sạc", None, "HALLWAY", True),
        ("quay về hành lang", None, "HALLWAY", True),
        ("ve dock", None, "HALLWAY", True),
        ("Đi đến phòng họp", None, "C", True),
        ("tới khu vực C", None, "C", True),
        ("Di chuyển từ khu vực A đến khu vực B", "A", "B", True),
        ("di chuyen tu phong hoc sang phong thi nghiem", "A", "B", True),
        ("tu phong b ve tram sac", "B", "HALLWAY", True),
        ("học", None, "A", True),
        ("thí nghiệm", None, "B", True),
        ("họp", None, "C", True),
        ("sạc", None, "HALLWAY", True),
        # Lệnh dừng khẩn cấp
        ("dừng lại", "STOP", None, True),
        ("stop", "STOP", None, True),
        # Các câu lệnh lỗi / không tồn tại
        ("Đi đến khu vực Z", None, None, False),
        ("Đi đến phòng bếp", None, None, False),
        ("", None, None, False),
    ]

    for utterance, exp_orig, exp_dest, should_succeed in test_cases:
        orig_reg, dest_reg, err = parser.parse_command(utterance)
        if should_succeed:
            assert err is None, f"Lỗi không mong muốn với câu: '{utterance}': {err}"
            if exp_orig == "STOP":
                assert orig_reg == "STOP"
            else:
                assert dest_reg is not None, f"Không tìm thấy đích cho: '{utterance}'"
                assert dest_reg['id'] == exp_dest, f"Sai đích cho '{utterance}': mong đợi {exp_dest}, nhận {dest_reg['id']}"
                if exp_orig:
                    assert orig_reg is not None and orig_reg['id'] == exp_orig, f"Sai gốc cho '{utterance}'"
        else:
            assert err is not None, f"Câu lệnh lỗi '{utterance}' đáng lẽ phải báo lỗi nhưng lại pass!"

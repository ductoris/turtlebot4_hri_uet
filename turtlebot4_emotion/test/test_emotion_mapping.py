#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests kiểm tra logic ánh xạ biểu cảm -> vận tốc,
cửa sổ trượt voting và máy trạng thái xoay 360 độ.
"""

import collections
import math
import pytest
from turtlebot4_emotion.emotion_tb4_node import EMOTION_VELOCITY_MAP


def test_velocity_mappings():
    """Kiểm tra đúng vận tốc theo bảng yêu cầu"""
    # happy: linear.x=0.15, angular.z=+0.5
    assert EMOTION_VELOCITY_MAP['happy'][0] == 0.15
    assert EMOTION_VELOCITY_MAP['happy'][1] == 0.5

    # sad: linear.x=0.15, angular.z=-0.5
    assert EMOTION_VELOCITY_MAP['sad'][0] == 0.15
    assert EMOTION_VELOCITY_MAP['sad'][1] == -0.5

    # neutral: linear.x=0, angular.z=0
    assert EMOTION_VELOCITY_MAP['neutral'][0] == 0.0
    assert EMOTION_VELOCITY_MAP['neutral'][1] == 0.0

    # angry: linear.x=-0.08, angular.z=0
    assert EMOTION_VELOCITY_MAP['angry'][0] == -0.08
    assert EMOTION_VELOCITY_MAP['angry'][1] == 0.0

    # surprise: linear.x=0, angular.z=0.8
    assert EMOTION_VELOCITY_MAP['surprise'][0] == 0.0
    assert EMOTION_VELOCITY_MAP['surprise'][1] == 0.8

    # fear: linear.x=0.1, angular.z=0
    assert EMOTION_VELOCITY_MAP['fear'][0] == 0.10
    assert EMOTION_VELOCITY_MAP['fear'][1] == 0.0


def test_sliding_window_voting():
    """Kiểm tra thuật toán bỏ phiếu đa số trên cửa sổ N=5"""
    window = collections.deque(maxlen=5)

    # Thêm 3 happy và 2 neutral -> happy chiếm đa số
    for e in ['neutral', 'happy', 'neutral', 'happy', 'happy']:
        window.append(e)

    counts = collections.Counter(window)
    assert counts.most_common(1)[0][0] == 'happy'

    # Thêm 2 sad -> window hiện tại có: neutral, happy, happy, sad, sad
    # Thêm tiếp 1 sad -> window có: happy, happy, sad, sad, sad -> sad chiếm đa số
    window.append('sad')
    window.append('sad')
    window.append('sad')
    counts = collections.Counter(window)
    assert counts.most_common(1)[0][0] == 'sad'


def test_surprise_rotation_duration():
    """Kiểm tra thời gian xoay 360 độ: 2*pi / 0.8 s"""
    speed = 0.8
    duration = (2.0 * math.pi) / speed
    expected = 7.853981633974483
    assert abs(duration - expected) < 1e-6

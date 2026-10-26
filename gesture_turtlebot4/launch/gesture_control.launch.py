#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_control.launch.py:
Khởi chạy đồng thời cả hai node:
1. gesture_recognition_node: Đọc webcam, nhận diện cử chỉ, phát /detected_gesture
2. robot_behavior_node: Nhận cử chỉ, điều khiển TurtleBot 4 qua /cmd_vel
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('gesture_turtlebot4')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Đường dẫn tới file cấu hình yaml chứa tham số cho hai node'
    )

    gesture_node = Node(
        package='gesture_turtlebot4',
        executable='gesture_recognition_node',
        name='gesture_recognition_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    behavior_node = Node(
        package='gesture_turtlebot4',
        executable='robot_behavior_node',
        name='robot_behavior_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_arg,
        gesture_node,
        behavior_node
    ])


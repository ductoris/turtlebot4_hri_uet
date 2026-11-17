#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Launch file cho node emotion_tb4_node trong package turtlebot4_emotion
Cho phép truyền tham số linh hoạt từ dòng lệnh.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Khai báo các đối số dòng lệnh (Launch Arguments)
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace của TurtleBot 4 (mặc định rỗng, topic: /cmd_vel)'
    )

    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value='0',
        description='Index webcam (vd: 0, 1) hoặc đường dẫn file video (vd: /path/to/video.mp4)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Tần số gửi lệnh vận tốc Twist lên /cmd_vel (Hz)'
    )

    window_size_arg = DeclareLaunchArgument(
        'window_size',
        default_value='5',
        description='Kích thước cửa sổ trượt (N) để bỏ phiếu đa số biểu cảm'
    )

    no_face_timeout_arg = DeclareLaunchArgument(
        'no_face_timeout',
        default_value='1.0',
        description='Thời gian (giây) mất khuôn mặt trước khi tự động dừng robot'
    )

    auto_undock_arg = DeclareLaunchArgument(
        'auto_undock',
        default_value='true',
        description='Tự động gọi action /undock khi khởi động node nếu robot ở trên trạm sạc'
    )

    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.2',
        description='Giới hạn vận tốc tịnh tiến tối đa (m/s)'
    )

    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.0',
        description='Giới hạn vận tốc góc tối đa (rad/s)'
    )

    show_gui_arg = DeclareLaunchArgument(
        'show_gui',
        default_value='true',
        description='Bật/Tắt cửa sổ hiển thị OpenCV HUD'
    )

    # Khởi tạo Node
    emotion_node = Node(
        package='turtlebot4_emotion',
        executable='emotion_tb4_node',
        name='emotion_tb4_node',
        output='screen',
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'video_source': LaunchConfiguration('video_source'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'window_size': LaunchConfiguration('window_size'),
            'no_face_timeout': LaunchConfiguration('no_face_timeout'),
            'auto_undock': LaunchConfiguration('auto_undock'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'show_gui': LaunchConfiguration('show_gui'),
        }]
    )

    return LaunchDescription([
        robot_namespace_arg,
        video_source_arg,
        publish_rate_arg,
        window_size_arg,
        no_face_timeout_arg,
        auto_undock_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        show_gui_arg,
        emotion_node,
    ])

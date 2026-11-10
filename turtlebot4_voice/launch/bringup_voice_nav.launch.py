#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bringup_voice_nav.launch.py:
Khởi chạy trọn vẹn toàn bộ hệ thống phân vùng bản đồ và điều hướng robot bằng giọng nói:
1. Mô phỏng Ignition Gazebo với thế giới văn phòng office.sdf và TurtleBot 4.
2. Nạp bản đồ office_map.yaml và chạy Nav2 stack (Localization + Planners + Controllers).
3. Khởi chạy RViz2 trực quan hóa toàn diện.
4. Tự động thiết lập Initial Pose tại trạm sạc dock (x=0, y=0, yaw=0).
5. Khởi chạy regions_marker_node hiển thị nhãn các khu vực 3D trên RViz.
6. Khởi chạy voice_nav_node xử lý NLU và gửi action goal tới Nav2.
7. Khởi chạy speech_node (nhận dạng giọng nói Vosk / Text fallback).
"""

import os
import glob
import subprocess

# Đảm bảo thiết lập biến môi trường FastDDS UDP-only trước khi bất kỳ module ROS 2 nào nạp DDS
fastdds_xml = '/home/ductri/workspaces/ur_gz/src/turtlebot4_voice/config/fastdds_udp_only.xml'
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_xml
os.environ['ROS_LOCALHOST_ONLY'] = '1'

# Dọn dẹp sạch sẽ các tệp shared memory và semaphores cũ trong /dev/shm
for shm_f in glob.glob('/dev/shm/fastrtps*') + glob.glob('/dev/shm/sem.*'):
    try:
        os.remove(shm_f)
    except Exception:
        pass

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_voice = FindPackageShare('turtlebot4_voice')

    default_regions_file = PathJoinSubstitution([
        pkg_voice, 'config', 'regions.yaml'
    ])
    default_map_file = PathJoinSubstitution([
        pkg_voice, 'maps', 'office_map.yaml'
    ])

    # Khai báo các tham số khởi chạy
    regions_arg = DeclareLaunchArgument(
        'regions_file',
        default_value=default_regions_file,
        description='Đường dẫn tới file cấu hình regions.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Đường dẫn tới file bản đồ office_map.yaml'
    )

    engine_arg = DeclareLaunchArgument(
        'engine',
        default_value='vosk',
        choices=['vosk', 'whisper'],
        description='Engine nhận dạng giọng nói (vosk hoặc whisper)'
    )

    input_mode_arg = DeclareLaunchArgument(
        'input_mode',
        default_value='text',
        choices=['text', 'mic', 'file'],
        description='Chế độ nhập lệnh: text (bàn phím terminal), mic (microphone), file (wav)'
    )

    wav_file_arg = DeclareLaunchArgument(
        'wav_file',
        default_value='',
        description='Đường dẫn file WAV khi dùng input_mode:=file'
    )

    sim_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Mở giao diện RViz2'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        choices=['true', 'false'],
        description='Mở giao diện GUI riêng hiển thị từ ngữ thu nhận từ microphone và trạng thái'
    )

    # 1. Khởi chạy Mô phỏng + Nav2 + RViz (đã tích hợp dummy_hardware_services bên trong)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_voice, 'launch', 'sim_bringup.launch.py'])
        ]),
        launch_arguments=[
            ('nav2', 'true'),
            ('rviz', LaunchConfiguration('rviz')),
            ('map', LaunchConfiguration('map')),
        ]
    )

    # 2. Node hiển thị nhãn các khu vực (MarkerArray) trên RViz2
    marker_node = Node(
        package='turtlebot4_voice',
        executable='regions_marker_node',
        name='regions_marker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'regions_file': LaunchConfiguration('regions_file')
        }]
    )

    # 3. Node NLU và điều hướng robot
    voice_nav_node = Node(
        package='turtlebot4_voice',
        executable='voice_nav_node',
        name='voice_nav_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'regions_file': LaunchConfiguration('regions_file'),
            'similarity_threshold': 0.75
        }]
    )

    # 4. Node nhận dạng giọng nói (chạy trong terminal riêng hoặc background)
    speech_node = Node(
        package='turtlebot4_voice',
        executable='speech_node',
        name='speech_node',
        output='screen',
        parameters=[{
            'engine': LaunchConfiguration('engine'),
            'input_mode': LaunchConfiguration('input_mode'),
            'wav_file': LaunchConfiguration('wav_file'),
            'model_path': '/home/ductri/VSCode/hri/models/vosk-model-small-vn-0.4'
        }]
    )

    # 5. Giao diện GUI riêng hiển thị từ ngữ thu nhận từ microphone
    dashboard_gui_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('gui')),
        actions=[
            Node(
                package='turtlebot4_voice',
                executable='voice_dashboard_gui',
                name='voice_dashboard_gui',
                output='screen'
            )
        ]
    )

    fastdds_profile_path = PathJoinSubstitution([pkg_voice, 'config', 'fastdds_udp_only.xml'])
    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile_path)
    set_localhost = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')
    disable_bond_timeout = SetParameter(name='bond_timeout', value=0.0)

    return LaunchDescription([
        set_fastdds,
        set_localhost,
        disable_bond_timeout,
        regions_arg,
        map_arg,
        engine_arg,
        input_mode_arg,
        wav_file_arg,
        sim_rviz_arg,
        gui_arg,
        sim_bringup,
        marker_node,
        voice_nav_node,
        speech_node,
        dashboard_gui_node,
    ])

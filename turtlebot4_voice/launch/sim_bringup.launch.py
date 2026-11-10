#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_bringup.launch.py:
Khởi chạy thế giới mô phỏng văn phòng (office.sdf) cùng TurtleBot 4 tiêu chuẩn.
Khắc phục triệt để lỗi tên world trong ROS-Ignition parameter bridge (tránh lỗi remap // và SIGABRT).

Hỗ trợ:
- Chế độ cơ bản: chỉ mô phỏng (Gazebo)
- Chế độ SLAM: slam:=true để quét bản đồ
- Chế độ Localization + Nav2: nav2:=true hoặc localization:=true để nạp bản đồ và điều hướng
- RViz2: rviz:=true
"""

import os
import glob
import subprocess
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable, TimerAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Đóng mọi tiến trình Gazebo zombie cũ trước khi khởi động instance mới
    subprocess.run(['pkill', '-9', '-f', 'ign gazebo server'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Đọc giá trị các cờ dưới dạng boolean Python trước khi nạp các launch con
    nav2_enabled = LaunchConfiguration('nav2').perform(context).lower() in ['true', '1']
    slam_enabled = LaunchConfiguration('slam').perform(context).lower() in ['true', '1']
    loc_enabled = LaunchConfiguration('localization').perform(context).lower() in ['true', '1']
    rviz_enabled = LaunchConfiguration('rviz').perform(context).lower() in ['true', '1']

    pkg_voice = FindPackageShare('turtlebot4_voice')
    pkg_tb4_ignition = FindPackageShare('turtlebot4_ignition_bringup')
    pkg_tb4_nav = FindPackageShare('turtlebot4_navigation')
    pkg_tb4_viz = FindPackageShare('turtlebot4_viz')
    pkg_ros_ign_gz = FindPackageShare('ros_ign_gazebo')

    # 0. Node giả lập hardware services (chặn lỗi service stop_motor / oakd unavailable)
    dummy_hw_node = Node(
        package='turtlebot4_voice',
        executable='dummy_hardware_services',
        name='dummy_hardware_services',
        output='screen'
    )

    # 1. Khởi chạy Ignition Gazebo với file office.sdf
    gui_config_path = PathJoinSubstitution([
        pkg_tb4_ignition, 'gui', LaunchConfiguration('model'), 'gui.config'
    ])
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_ros_ign_gz, 'launch', 'ign_gazebo.launch.py'])
        ]),
        launch_arguments={
            'ign_args': [
                LaunchConfiguration('world_sdf'),
                ' -r -v 4 --gui-config ',
                gui_config_path
            ]
        }.items()
    )

    # Clock bridge (GZ -> ROS 2)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    )

    # Ground Truth Pose bridge (Gazebo Ignition /world/warehouse/pose/info -> ROS 2 _internal/sim_ground_truth_pose)
    pose_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_info_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/warehouse/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        remappings=[
            ('/world/warehouse/pose/info', '_internal/sim_ground_truth_pose')
        ]
    )

    # 2. Spawn Robot TurtleBot 4 & Dock (dùng tb4_spawn tùy chỉnh: spawn trong không gian tự do, không kẹt dock)
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_voice, 'launch', 'tb4_spawn.launch.py'])
        ]),
        launch_arguments=[
            ('world', 'warehouse'),
            ('model', LaunchConfiguration('model')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('nav2', 'false'),
            ('localization', 'false'),
            ('slam', 'false'),
            ('rviz', 'false'),
        ]
    )

    actions = [
        SetParameter(name='bond_timeout', value=0.0),
        dummy_hw_node,
        ign_gazebo_launch,
        clock_bridge,
        pose_info_bridge,
        robot_spawn_launch,
    ]

    # 3. SLAM Toolbox (chế độ quét bản đồ)
    if slam_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
                ]),
                launch_arguments={
                    'sync': 'false',
                    'use_sim_time': 'true',
                }.items()
            )
        )

    localization_params = PathJoinSubstitution([pkg_voice, 'config', 'localization.yaml'])
    nav2_params = PathJoinSubstitution([pkg_voice, 'config', 'nav2.yaml'])
    rviz_config = PathJoinSubstitution([pkg_voice, 'config', 'turtlebot4_voice.rviz'])

    # 4. Localization độc lập (khi không chạy Nav2)
    if loc_enabled and not nav2_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])
                ]),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'true',
                    'params': localization_params,
                }.items()
            )
        )

    # 5. Nav2 Stack (tuần tự nạp localization sau 10s và planners/costmaps sau 15s để Gazebo hoàn toàn ổn định)
    if nav2_enabled:
        # 5a. Ground-truth TF broadcaster (map -> odom) — phát ngay lập tức
        # để RViz luôn hiển thị robot đúng vị trí thực trong Gazebo
        actions.append(
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='turtlebot4_voice',
                        executable='sim_ground_truth_odom',
                        name='sim_ground_truth_odom',
                        output='screen',
                        parameters=[{'use_sim_time': True}]
                    )
                ]
            )
        )

        # 5b. Localization (AMCL) — tf_broadcast: false trong localization.yaml
        # AMCL vẫn chạy cho hiển thị particle swarm và /amcl_pose
        actions.append(
            TimerAction(
                period=10.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'localization.launch.py'])
                        ]),
                        launch_arguments=[
                            ('map', LaunchConfiguration('map')),
                            ('use_sim_time', 'true'),
                            ('params', localization_params),
                        ]
                    )
                ]
            )
        )

        # 5c. Nav2 planners/controllers
        actions.append(
            TimerAction(
                period=15.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'nav2.launch.py'])
                        ]),
                        launch_arguments=[
                            ('use_sim_time', 'true'),
                            ('params_file', nav2_params),
                        ]
                    )
                ]
            )
        )

    # 6. RViz2 với cấu hình chuẩn turtlebot4_voice.rviz (khớp góc nhìn Gazebo và hiển thị markers)
    if rviz_enabled:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ],
                output='screen'
            )
        )

    return actions

def generate_launch_description():
    pkg_voice = FindPackageShare('turtlebot4_voice')
    pkg_tb4_ignition = FindPackageShare('turtlebot4_ignition_bringup')
    pkg_tb4_nav = FindPackageShare('turtlebot4_navigation')
    pkg_tb4_viz = FindPackageShare('turtlebot4_viz')
    pkg_ros_ign_gz = FindPackageShare('ros_ign_gazebo')

    # Các thư mục chứa resource Ignition Gazebo
    pkg_voice_dir = get_package_share_directory('turtlebot4_voice')
    pkg_tb4_ign_dir = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_tb4_desc_dir = get_package_share_directory('turtlebot4_description')
    pkg_create_ign_dir = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_create_desc_dir = get_package_share_directory('irobot_create_description')
    pkg_tb4_gui_dir = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_create_plugins_dir = get_package_share_directory('irobot_create_ignition_plugins')

    # Đường dẫn file world SDF và map YAML
    default_world_sdf = os.path.join(pkg_voice_dir, 'worlds', 'office.sdf')
    default_map_path = os.path.join(pkg_voice_dir, 'maps', 'office_map.yaml')

    # Thiết lập biến môi trường tìm kiếm model và mesh của Gazebo
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_voice_dir, 'worlds'),
            os.path.join(pkg_tb4_ign_dir, 'worlds'),
            os.path.join(pkg_create_ign_dir, 'worlds'),
            str(Path(pkg_tb4_desc_dir).parent.resolve()),
            str(Path(pkg_create_desc_dir).parent.resolve()),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ])
    )

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_tb4_gui_dir, 'lib'),
            os.path.join(pkg_create_plugins_dir, 'lib'),
            os.environ.get('IGN_GUI_PLUGIN_PATH', '')
        ])
    )

    # Khai báo arguments
    world_arg = DeclareLaunchArgument(
        'world_sdf',
        default_value=default_world_sdf,
        description='Đường dẫn tuyệt đối đến file office.sdf'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='standard',
        choices=['standard', 'lite'],
        description='Dòng robot TurtleBot 4 (standard hoặc lite)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        choices=['true', 'false'],
        description='Mở RViz2 trực quan hóa'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        choices=['true', 'false'],
        description='Chạy SLAM Toolbox để quét bản đồ'
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        choices=['true', 'false'],
        description='Chạy Nav2 stack (Planners, Controllers, Costmaps)'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        choices=['true', 'false'],
        description='Chạy Localization (Map Server + AMCL)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Đường dẫn tuyệt đối đến file office_map.yaml'
    )

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Tọa độ X ban đầu')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Tọa độ Y ban đầu')
    z_arg = DeclareLaunchArgument('z', default_value='0.01', description='Tọa độ Z ban đầu')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Góc Yaw ban đầu (rad)')

    fastdds_profile_path = os.path.join(pkg_voice_dir, 'config', 'fastdds_udp_only.xml')
    set_fastdds = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile_path)
    set_localhost = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')
    disable_bond_timeout = SetParameter(name='bond_timeout', value=0.0)

    return LaunchDescription([
        set_fastdds,
        set_localhost,
        disable_bond_timeout,
        ign_resource_path,
        ign_gui_plugin_path,
        world_arg,
        model_arg,
        rviz_arg,
        slam_arg,
        nav2_arg,
        localization_arg,
        map_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])

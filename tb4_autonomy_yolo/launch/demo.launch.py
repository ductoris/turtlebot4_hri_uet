"""demo.launch.py - Khởi động đồng thời Localization (AMCL), Nav2, RViz2 và YOLOv8 node.

Cách dùng:
  Terminal 1: ros2 launch tb4_autonomy_yolo sim.launch.py
  Terminal 2: ros2 launch tb4_autonomy_yolo demo.launch.py
  Terminal 3: ros2 run tb4_autonomy_yolo nav_node --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_self = get_package_share_directory('tb4_autonomy_yolo')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_self, 'maps', 'home_room.yaml'),
        description='Đường dẫn file map.yaml')
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    # Khởi động Nav2 + Localization + RViz2
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_self, 'launch', 'nav.launch.py'])),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
        }.items())

    # Khởi động YOLO node nhận diện đối tượng
    yolo_node = Node(
        package='tb4_autonomy_yolo',
        executable='yolo_node',
        name='yolo_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'show_window': True,
            'conf_threshold': 0.40,
        }],
        output='screen'
    )

    return LaunchDescription([
        map_arg,
        sim_arg,
        rviz_arg,
        nav_launch,
        # Trì hoãn yolo_node 2s để camera feed và ROS bridge ổn định
        TimerAction(period=2.0, actions=[yolo_node])
    ])

"""Localization (map_server + AMCL) + Nav2 với bản đồ home_room, dùng launch của TurtleBot 4.

  ros2 launch tb4_autonomy_yolo nav.launch.py
  ros2 launch tb4_autonomy_yolo nav.launch.py map:=/duong/dan/map.yaml rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_self = get_package_share_directory('tb4_autonomy_yolo')
    pkg_nav = get_package_share_directory('turtlebot4_navigation')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_self, 'maps', 'home_room.yaml'),
        description='File map.yaml')
    sim_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', choices=['true', 'false'],
        description='Khởi động RViz2 trực quan hóa Navigation + YOLO')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'localization.launch.py'])),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'nav2.launch.py'])),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items())

    rviz_config = os.path.join(pkg_self, 'rviz', 'nav_yolo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # Khởi chạy chuẩn giống turbot4_simulation
    return LaunchDescription([
        map_arg,
        sim_arg,
        rviz_arg,
        localization,
        nav2,
        rviz_node,
    ])


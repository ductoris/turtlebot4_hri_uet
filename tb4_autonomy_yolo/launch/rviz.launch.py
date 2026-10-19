"""rviz.launch.py - Khởi động RViz2 với cấu hình chuẩn cho TurtleBot 4, Navigation 2 và YOLO.

Tính năng:
  - Hiển thị RobotModel TurtleBot 4 (Visual 3D).
  - Hiển thị Odometry (/odom) và trục tọa độ TF.
  - Hiển thị Bản đồ (/map) và AMCL Particle Cloud.
  - Cho phép dùng công cụ 2D Pose Estimate (/initialpose) và 2D Nav Goal (/goal_pose).
  - Hiển thị cửa sổ phụ camera YOLO (/yolo/image_annotated).

Chạy:
  ros2 launch tb4_autonomy_yolo rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_self = get_package_share_directory('tb4_autonomy_yolo')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Sử dụng thời gian mô phỏng (clock)'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_self, 'rviz', 'nav_yolo.rviz'),
        description='File cấu hình .rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        sim_time_arg,
        rviz_config_arg,
        rviz_node,
    ])

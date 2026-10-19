"""Mở Ignition Gazebo với world home_room (phòng khách mini) + spawn TurtleBot 4.

Lý do cần file launch riêng: ignition.launch.py của TurtleBot 4 ghi đè
IGN_GAZEBO_RESOURCE_PATH nên không tìm thấy world nằm ngoài gói của nó.
File này lặp lại đúng các bước đó nhưng nạp world của package này.

  ros2 launch tb4_autonomy_yolo sim.launch.py
  ros2 launch tb4_autonomy_yolo sim.launch.py x:=0.0 y:=0.0 yaw:=0.0 rviz:=false
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_self = get_package_share_directory('tb4_autonomy_yolo')
    pkg_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    pkg_gui = get_package_share_directory('turtlebot4_ignition_gui_plugins')
    pkg_tb4_desc = get_package_share_directory('turtlebot4_description')
    pkg_create_desc = get_package_share_directory('irobot_create_description')
    pkg_create_ign = get_package_share_directory('irobot_create_ignition_bringup')
    pkg_create_plugins = get_package_share_directory('irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    args = [
        DeclareLaunchArgument('world_file',
                              default_value=os.path.join(pkg_self, 'worlds', 'home_room.sdf'),
                              description='Đường dẫn file .sdf (tên world bên trong phải là "warehouse")'),
        DeclareLaunchArgument('model', default_value='standard',
                              choices=['standard', 'lite']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_self, 'worlds'), ':',
               os.path.join(pkg_bringup, 'worlds'), ':',
               os.path.join(pkg_create_ign, 'worlds'), ':',
               str(Path(pkg_tb4_desc).parent.resolve()), ':',
               str(Path(pkg_create_desc).parent.resolve())])

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[os.path.join(pkg_gui, 'lib'), ':', os.path.join(pkg_create_plugins, 'lib')])

    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])),
        launch_arguments=[('ign_args', [
            LaunchConfiguration('world_file'), ' -r -v 2 --gui-config ',
            PathJoinSubstitution([pkg_bringup, 'gui', LaunchConfiguration('model'),
                                  'gui.config'])])])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'turtlebot4_spawn.launch.py'])),
        launch_arguments=[
            ('namespace', ''),
            ('model', LaunchConfiguration('model')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', '0.02'),
            ('yaw', LaunchConfiguration('yaw')),
        ])

    return LaunchDescription(args + [ign_resource_path, ign_gui_plugin_path,
                                     ignition_gazebo, clock_bridge, robot_spawn])

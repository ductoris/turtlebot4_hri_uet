from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_tb4_ignition = FindPackageShare('turtlebot4_ignition_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Ignition Gazebo world (warehouse, depot, maze)'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='standard',
        description='TurtleBot 4 model (standard, lite)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 alongside Gazebo'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py'])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'model': LaunchConfiguration('model'),
            'rviz': LaunchConfiguration('rviz'),
            'slam': 'false',
            'nav2': 'false',
        }.items()
    )

    return LaunchDescription([
        world_arg,
        model_arg,
        rviz_arg,
        sim_launch,
    ])

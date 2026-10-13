from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_tb4_nav = FindPackageShare('turtlebot4_navigation')

    sync_arg = DeclareLaunchArgument(
        'sync',
        default_value='false',
        description='Whether to run SLAM Toolbox in synchronous mode (true/false). Asynchronous (false) is recommended for simulation.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
        ]),
        launch_arguments={
            'sync': LaunchConfiguration('sync'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        sync_arg,
        use_sim_time_arg,
        slam_launch,
    ])

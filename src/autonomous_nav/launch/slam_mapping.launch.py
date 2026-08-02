from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params')
    nav2_params = LaunchConfiguration('nav2_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml'),
            description='SLAM Toolbox parameter file',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file',
        ),

        # Launch SLAM Toolbox (online async for mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params,
            }.items(),
        ),

        # Launch Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
            }.items(),
        ),
    ])

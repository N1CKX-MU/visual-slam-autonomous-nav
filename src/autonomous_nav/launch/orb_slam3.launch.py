from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autonomous_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    voc_file = LaunchConfiguration('voc_file')
    settings_file = LaunchConfiguration('settings_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.expanduser('~/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
            description='Path to ORB vocabulary file',
        ),
        DeclareLaunchArgument(
            'settings_file',
            default_value=os.path.join(pkg_dir, 'config', 'orb_slam3.yaml'),
            description='Path to ORB-SLAM3 settings file',
        ),

        # ORB-SLAM3 RGB-D node
        Node(
            package='ros2_orb_slam3',
            executable='rgbd_node',
            name='orb_slam3',
            parameters=[{
                'voc_file': voc_file,
                'settings_file': settings_file,
                'use_sim_time': use_sim_time,
            }],
            # Remap to match Gazebo Harmonic camera topics
            # Verify actual topic names with: ros2 topic list | grep camera
            remappings=[
                ('/camera/rgb', '/camera/image_raw'),
                ('/camera/depth', '/camera/depth/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
            ],
            output='screen',
        ),

        # Bridge ORB-SLAM3 pose to Nav2 TF
        Node(
            package='autonomous_nav',
            executable='orb_slam_bridge',
            name='orb_slam_bridge',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])

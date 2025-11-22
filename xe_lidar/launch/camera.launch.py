import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_size': [640,480],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical'
            }]
        )
    ])

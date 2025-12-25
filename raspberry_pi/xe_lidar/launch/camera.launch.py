import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV', description='Pixel format: YUYV, MJPG, etc.'),
        DeclareLaunchArgument('width', default_value='640', description='Image width'),
        DeclareLaunchArgument('height', default_value='480', description='Image height'),
        DeclareLaunchArgument('fps', default_value='30', description='Frames per second'),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            namespace='camera',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_size': [640, 480],  # Mặc định, có thể override bằng parameters
                'pixel_format': LaunchConfiguration('pixel_format'),
                'time_per_frame': [1, 30],  # 30 FPS mặc định
                'camera_frame_id': 'camera_link_optical'
            }]
        )
    ])

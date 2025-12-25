import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640', description='Image width'),
        DeclareLaunchArgument('height', default_value='480', description='Image height'),
        DeclareLaunchArgument('fps', default_value='30', description='Frames per second'),
        # Node camera dùng OpenCV (không cần v4l2_camera package)
        Node(
            package='xe_lidar',
            executable='camera_node.py',
            output='screen',
            name='camera_node',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': 'camera_link_optical'
            }]
        )
    ])

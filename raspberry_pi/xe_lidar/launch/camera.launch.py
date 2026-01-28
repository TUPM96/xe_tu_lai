import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width', default_value='640', description='Image width (default: 640)'),
        DeclareLaunchArgument('height', default_value='480', description='Image height (default: 480)'),
        DeclareLaunchArgument('fps', default_value='30', description='Frames per second'),
        DeclareLaunchArgument('center_offset_x', default_value='0', description='Pixels: dương = dịch nội dung sang phải (chỉnh khi "giữa hình" lệch trái)'),
        DeclareLaunchArgument('center_offset_y', default_value='0', description='Pixels: dương = dịch nội dung xuống'),
        
        # Static transform từ base_link đến camera_link_optical (cần cho rviz)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link_optical']
        ),
        
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
                'frame_id': 'camera_link_optical',
                'center_offset_x': LaunchConfiguration('center_offset_x'),
                'center_offset_y': LaunchConfiguration('center_offset_y')
            }]
        )
    ])

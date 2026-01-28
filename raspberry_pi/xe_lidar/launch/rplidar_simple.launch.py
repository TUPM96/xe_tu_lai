"""
Launch file đơn giản để chỉ chạy LiDAR
Không cần nhiều tham số, chỉ cần port là đủ
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Tham số duy nhất: serial port
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port của RPLIDAR'
    )

    return LaunchDescription([
        serial_port_arg,
        
        # Static transform từ base_link đến laser_frame (cần cho rviz)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'  # Mặc định, không cần tham số
            }]
        )
    ])

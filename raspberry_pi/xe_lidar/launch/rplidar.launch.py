import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port to which the RPLIDAR is connected'
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Scan mode of the RPLIDAR'
    )

    return LaunchDescription([
        serial_port_arg,
        scan_mode_arg,

        # Static transform từ base_link đến laser_frame (cần cho rviz)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': LaunchConfiguration('scan_mode')
            }]
        )
    ])

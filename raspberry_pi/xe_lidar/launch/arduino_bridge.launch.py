import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'xe_lidar'
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port của Arduino (ví dụ: /dev/ttyACM0 hoặc /dev/ttyUSB0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate của Serial (mặc định: 115200)'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect',
        default_value='true',
        description='Tự động phát hiện cổng Arduino (true/false)'
    )
    
    # Arduino Bridge Node
    arduino_bridge_node = Node(
        package=package_name,
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'auto_detect': LaunchConfiguration('auto_detect'),
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        auto_detect_arg,
        arduino_bridge_node,
    ])



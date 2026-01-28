"""
Launch file để chạy xe tự lái Ackermann chỉ dùng LiDAR
Tương tự project GitHub nhưng phù hợp với Ackermann steering
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'xe_lidar'
    
    print("=" * 60)
    print("🚗 KHỞI ĐỘNG XE TỰ LÁI ACKERMANN - CHỈ DÙNG LIDAR")
    print("=" * 60)
    print(f"📦 Package: {package_name}")
    print("=" * 60)
    
    # Tham số LiDAR
    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port của RPLIDAR'
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Chế độ quét của RPLIDAR'
    )
    
    # Tham số điều khiển
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.3',
        description='Tốc độ tối đa (m/s)'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='0.5',
        description='Khoảng cách an toàn để tránh vật cản (m)'
    )
    
    wall_follow_distance_arg = DeclareLaunchArgument(
        'wall_follow_distance',
        default_value='0.4',
        description='Khoảng cách bám tường (m)'
    )
    
    front_angle_range_arg = DeclareLaunchArgument(
        'front_angle_range',
        default_value='90.0',
        description='Góc phía trước để kiểm tra vật cản (degrees)'
    )
    
    servo_center_angle_arg = DeclareLaunchArgument(
        'servo_center_angle',
        default_value='100.0',
        description='Góc servo giữa (degrees)'
    )
    
    servo_min_angle_arg = DeclareLaunchArgument(
        'servo_min_angle',
        default_value='55.0',
        description='Góc servo tối thiểu (degrees)'
    )
    
    servo_max_angle_arg = DeclareLaunchArgument(
        'servo_max_angle',
        default_value='145.0',
        description='Góc servo tối đa (degrees)'
    )
    
    # Static transform từ base_link đến laser_frame
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    # RPLIDAR Node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': LaunchConfiguration('scan_mode')
        }]
    )
    
    # Arduino Bridge Node
    arduino_serial_port_arg = DeclareLaunchArgument(
        'arduino_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port của Arduino'
    )
    
    motor_min_pwm_arg = DeclareLaunchArgument(
        'motor_min_pwm',
        default_value='100',
        description='PWM tối thiểu cho motor (0-255)'
    )
    
    arduino_bridge_node = Node(
        package=package_name,
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('arduino_serial_port'),
            'baudrate': 115200,
            'auto_detect': True,
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'motor_min_pwm': LaunchConfiguration('motor_min_pwm'),
        }]
    )
    
    # Lidar Ackermann Drive Node
    lidar_drive_node = Node(
        package=package_name,
        executable='lidar_ackermann_drive.py',
        name='lidar_ackermann_drive',
        output='screen',
        parameters=[{
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': 1.0,
            'safe_distance': LaunchConfiguration('safe_distance'),
            'front_angle_range': LaunchConfiguration('front_angle_range'),
            'wall_follow_distance': LaunchConfiguration('wall_follow_distance'),
            'servo_center_angle': LaunchConfiguration('servo_center_angle'),
            'servo_min_angle': LaunchConfiguration('servo_min_angle'),
            'servo_max_angle': LaunchConfiguration('servo_max_angle'),
            'servo_angle_smoothing': 0.7,
        }]
    )
    
    print("📋 TÓM TẮT CÁC NODE:")
    print("   1. RPLIDAR Node - /dev/ttyUSB0")
    print("   2. Arduino Bridge - /dev/ttyACM0")
    print("      └─ Subscribe: /cmd_vel, /servo_angle_cmd")
    print("   3. Lidar Ackermann Drive Node")
    print("      └─ Subscribe: /scan (LiDAR)")
    print("      └─ Publish: /cmd_vel, /servo_angle_cmd")
    print("")
    print("📊 FLOW:")
    print("   LiDAR -> Lidar Drive -> /cmd_vel + /servo_angle_cmd -> Arduino Bridge -> Arduino")
    print("=" * 60)
    
    return LaunchDescription([
        lidar_serial_port_arg,
        scan_mode_arg,
        arduino_serial_port_arg,
        max_linear_speed_arg,
        motor_min_pwm_arg,
        safe_distance_arg,
        wall_follow_distance_arg,
        front_angle_range_arg,
        servo_center_angle_arg,
        servo_min_angle_arg,
        servo_max_angle_arg,
        laser_tf_node,
        lidar_node,
        arduino_bridge_node,
        lidar_drive_node
    ])

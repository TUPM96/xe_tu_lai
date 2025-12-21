import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'xe_lidar'
    
    print("=" * 60)
    print("🚀 KHỞI ĐỘNG AUTONOMOUS DRIVE VỚI ARDUINO")
    print("=" * 60)
    print(f"📦 Package: {package_name}")
    print("=" * 60)
    
    # Robot state publisher (chỉ để visualize, không cần ros2_control)
    print("📡 Node 1: Robot State Publisher (RSP) - Ackermann 4 bánh")
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp_ackermann.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'false'  # Không dùng ros2_control khi dùng Arduino
        }.items()
    )
    
    # LiDAR
    print("📡 Node 2: RPLIDAR Node")
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
    
    # Camera
    print("📷 Node 3: Camera Node")
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Device camera'
    )
    
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        output='screen',
        namespace='camera',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'time_per_frame': [1, 6],
            'camera_frame_id': 'camera_link_optical'
        }]
    )
    
    # Arduino Bridge Node (nhận cmd_vel và gửi tới Arduino)
    print("🔌 Node 4: Arduino Bridge")
    arduino_serial_port_arg = DeclareLaunchArgument(
        'arduino_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port của Arduino'
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
        }]
    )
    
    # Autonomous Drive Node (Camera: lane detection, LiDAR: obstacle avoidance)
    print("🧠 Node 5: Autonomous Drive Node")
    print("   Subscribe: /scan (LiDAR), /camera/image_raw (Camera)")
    print("   Publish: /cmd_vel (điều khiển xe) -> Arduino Bridge -> Arduino")
    autonomous_drive_node = Node(
        package=package_name,
        executable='obstacle_avoidance.py',
        name='autonomous_drive',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'min_distance': 0.5,
            'safe_distance': 0.8,
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'front_angle_range': 60,
            'use_camera': True,
            'camera_topic': '/camera/image_raw'
        }]
    )
    
    print("=" * 60)
    print("📋 TÓM TẮT CÁC NODE:")
    print("   1. Robot State Publisher (RSP) - chỉ để visualize")
    print("   2. RPLIDAR Node - /dev/ttyUSB0")
    print("   3. Camera Node - /dev/video0")
    print("   4. Arduino Bridge - /dev/ttyACM0")
    print("      └─ Subscribe: /cmd_vel")
    print("      └─ Send to: Arduino qua Serial")
    print("   5. Autonomous Drive Node")
    print("      └─ Subscribe: /scan, /camera/image_raw")
    print("      └─ Publish: /cmd_vel")
    print("")
    print("📊 FLOW:")
    print("   Autonomous Drive -> /cmd_vel -> Arduino Bridge -> Arduino -> Motors/Servo")
    print("=" * 60)
    
    return LaunchDescription([
        lidar_serial_port_arg,
        scan_mode_arg,
        video_device_arg,
        arduino_serial_port_arg,
        rsp,
        lidar_node,
        camera_node,
        arduino_bridge_node,
        autonomous_drive_node
    ])



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
    
    # Joint State Publisher để publish joint states (cần cho RSP hiển thị khung xe)
    # Với Arduino, joints được điều khiển bởi Arduino, nhưng RSP cần joint_states để publish transforms
    # Dùng joint_state_publisher (không GUI) với giá trị mặc định = 0
    print("📊 Node 1.5: Joint State Publisher - publish joint states với giá trị mặc định")
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
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
    
    # Static transform từ base_link đến laser_frame (cần cho rviz)
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
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
    
    # Static transform từ base_link đến camera_link_optical (cần cho rviz)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link_optical']
    )
    
    # Camera
    print("📷 Node 3: Camera Node")
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Device camera'
    )
    
    camera_node = Node(
        package='xe_lidar',
        executable='camera_node.py',
        name='camera_node',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'width': 640,
            'height': 480,
            'fps': 30,
            'frame_id': 'camera_link_optical'
        }]
    )
    
    # Arduino Bridge Node (nhận cmd_vel và gửi tới Arduino)
    print("🔌 Node 4: Arduino Bridge")
    arduino_serial_port_arg = DeclareLaunchArgument(
        'arduino_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port của Arduino'
    )
    
    # Launch arguments cho tốc độ (dùng chung cho cả autonomous_drive và arduino_bridge)
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.3',
        description='Tốc độ tối đa (m/s)'
    )

    motor_min_pwm_arg = DeclareLaunchArgument(
        'motor_min_pwm',
        default_value='100',
        description='PWM tối thiểu cho motor (0-255)'
    )

    # Tham số khoảng cách an toàn LiDAR
    min_distance_arg = DeclareLaunchArgument(
        'min_distance',
        default_value='0.5',
        description='Khoang cach toi thieu de dung (m)'
    )

    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='0.8',
        description='Khoang cach an toan de tranh vat can (m)'
    )

    lane_threshold_c_arg = DeclareLaunchArgument(
        'lane_threshold_c',
        default_value='25',
        description='Nguong C cho lane detection (cao hon = chi nhan mau den hon)'
    )

    # Tham số làm mượt steering để tránh giật
    lane_offset_smoothing_arg = DeclareLaunchArgument(
        'lane_offset_smoothing',
        default_value='0.7',
        description='He so lam muot offset (0.0=khong smooth, 0.9=rat smooth)'
    )

    lane_dead_zone_arg = DeclareLaunchArgument(
        'lane_dead_zone',
        default_value='0.05',
        description='Vung chet - bo qua offset nho hon gia tri nay'
    )

    # Bật/tắt từng cảm biến trong node tự lái
    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Su dung LiDAR de tranh vat can (true/false)'
    )

    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Su dung camera de bam lan (true/false)'
    )

    # Tham số góc LiDAR phía trước để tránh vật cản
    front_angle_range_arg = DeclareLaunchArgument(
        'front_angle_range',
        default_value='60',
        description='Goc phia truoc (degrees) dung de kiem tra vat can bang LiDAR'
    )

    # Tham số điều khiển P cho bám làn
    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='0.5',
        description='He so P cho PID bam lan'
    )


    # Hệ số giảm tốc khi vào cua (khi đang đánh lái)
    cornering_speed_factor_arg = DeclareLaunchArgument(
        'cornering_speed_factor',
        default_value='0.6',
        description='He so giam toc khi vao cua (0.0-1.0), vi du 0.6 = 60% toc do'
    )

    # Tham số state machine cho việc rẽ (hard-coded turn)
    turn_distance_arg = DeclareLaunchArgument(
        'turn_distance',
        default_value='0.5',
        description='Khoang cach re (m) - chay 50cm roi moi xet tiep'
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.2',
        description='Toc do khi re (m/s)'
    )

    turn_trigger_threshold_arg = DeclareLaunchArgument(
        'turn_trigger_threshold',
        default_value='0.3',
        description='Nguong offset de kich hoat re (0.0-1.0)'
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
            'min_distance': LaunchConfiguration('min_distance'),
            'safe_distance': LaunchConfiguration('safe_distance'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': 1.0,
            'front_angle_range': LaunchConfiguration('front_angle_range'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_camera': LaunchConfiguration('use_camera'),
            'camera_topic': '/camera/image_raw',
            # Tham số điều khiển P cho bám làn
            'kp': LaunchConfiguration('kp'),
            # Tham số lane detection (Hough + adaptive threshold)
            'lane_threshold_c': LaunchConfiguration('lane_threshold_c'),
            'lane_threshold_c': LaunchConfiguration('lane_threshold_c'),
            # Tham số làm mượt để tránh giật góc lái
            'lane_offset_smoothing': LaunchConfiguration('lane_offset_smoothing'),
            'lane_dead_zone': LaunchConfiguration('lane_dead_zone'),
            'cornering_speed_factor': LaunchConfiguration('cornering_speed_factor'),
            # Tham số state machine cho việc rẽ
            'turn_distance': LaunchConfiguration('turn_distance'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'turn_trigger_threshold': LaunchConfiguration('turn_trigger_threshold'),
        }]
    )
    
    print("=" * 60)
    print("📋 TÓM TẮT CÁC NODE:")
    print("   1. Robot State Publisher (RSP) - chỉ để visualize")
    print("   1.5. Joint State Publisher - publish joint states (cần cho RSP)")
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
        max_linear_speed_arg,
        motor_min_pwm_arg,
        min_distance_arg,
        safe_distance_arg,
        lane_threshold_c_arg,
        lane_offset_smoothing_arg,
        lane_dead_zone_arg,
        use_lidar_arg,
        use_camera_arg,
        front_angle_range_arg,
        kp_arg,
        cornering_speed_factor_arg,
        turn_distance_arg,
        turn_speed_arg,
        turn_trigger_threshold_arg,
        rsp,
        joint_state_publisher_node,  # Thêm joint state publisher để RSP hiển thị khung xe
        laser_tf_node,
        lidar_node,
        camera_tf_node,
        camera_node,
        arduino_bridge_node,
        autonomous_drive_node
    ])



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'xe_lidar'
    
    print("=" * 60)
    print("🚀 KHỞI ĐỘNG AUTONOMOUS DRIVE (REAL HARDWARE)")
    print("=" * 60)
    print(f"📦 Package: {package_name}")
    print("=" * 60)
    
    # Robot state publisher (sử dụng launch file có sẵn) - Ackermann 4 bánh
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
            'use_ros2_control': 'true'
        }.items()
    )
    
    # Twist mux để quản lý các nguồn cmd_vel
    print("🔄 Node 2: Twist Mux")
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/ackermann_steering_controller/cmd_vel_unstamped')],
        output='screen'
    )

    # Controller manager
    print("🎮 Node 3: Controller Manager (delay 3s)")
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers_ackermann.yaml'
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
        output='screen'
    )

    # Delay controller manager
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    # Spawn controllers
    print("⚙️  Node 4: Ackermann Steering Controller Spawner (sau khi controller manager start)")
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    delayed_ackermann_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_controller_spawner]
        )
    )

    print("📊 Node 5: Joint State Broadcaster (sau khi controller manager start)")
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        output='screen'
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )
    
    # LiDAR
    print("📡 Node 6: RPLIDAR Node")
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
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
            'serial_port': LaunchConfiguration('serial_port'),
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
    print("📷 Node 7: Camera Node")
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
    
    # Autonomous Drive Node (Camera: lane detection, LiDAR: obstacle avoidance)
    print("🧠 Node 8: Autonomous Drive Node")
    print("   Subscribe: /scan (LiDAR), /camera/image_raw (Camera)")
    print("   Publish: /cmd_vel (điều khiển xe)")
    autonomous_drive_node = Node(
        package=package_name,
        executable='obstacle_avoidance.py',
        name='autonomous_drive',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'min_distance': 0.5,
            'safe_distance': 0.8,
            'max_linear_speed': 0.24,  # giam ~20% so voi 0.3 de xe di cham hon
            'max_angular_speed': 1.0,
            'front_angle_range': 60,
            'use_camera': True,
            'camera_topic': '/camera/image_raw'
        }]
    )
    
    print("=" * 60)
    print("📋 TÓM TẮT CÁC NODE:")
    print("   1. Robot State Publisher (RSP) - Ackermann 4 bánh - ngay lập tức")
    print("   2. Twist Mux - ngay lập tức")
    print("   3. Controller Manager - sau 3 giây")
    print("   4. Ackermann Steering Controller - sau khi controller manager start")
    print("   5. Joint State Broadcaster - sau khi controller manager start")
    print("   6. RPLIDAR Node - ngay lập tức")
    print("   7. Camera Node - ngay lập tức")
    print("   8. Autonomous Drive Node - ngay lập tức")
    print("=" * 60)
    
    return LaunchDescription([
        serial_port_arg,
        scan_mode_arg,
        video_device_arg,
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_ackermann_controller_spawner,
        delayed_joint_broad_spawner,
        laser_tf_node,
        lidar_node,
        camera_tf_node,
        camera_node,
        autonomous_drive_node
    ])


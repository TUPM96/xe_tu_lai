import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description"""
    package_name = 'xe_lidar'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # Log thông tin
    print("=" * 60)
    print("🚀 KHỞI ĐỘNG SIMULATION")
    print("=" * 60)
    print(f"📦 Package: {package_name}")
    print(f"📁 Share directory: {pkg_share}")
    print("=" * 60)
    
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'road_map.world'),
        description='Path to world file'
    )
    
    world = LaunchConfiguration('world')
    
    # Robot state publisher với sim time (Ackermann 4 bánh)
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
            'use_sim_time': 'true',
            'use_ros2_control': 'false'  # Dùng Gazebo plugin thay vì ros2_control
        }.items()
    )
    
    # Gazebo
    print("🌍 Node 2: Gazebo Simulator")
    print(f"   World file: {world}")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot vào Gazebo (delay để đợi Gazebo khởi động)
    # z = wheel_radius (0.034) để bánh xe chạm đất
    print("🤖 Node 3: Spawn Robot Entity (delay 5s)")
    print("   Position: x=-5.0, y=-1.0, z=0.034 (bánh xe chạm đất)")
    print("   ⚠️  LiDAR plugin sẽ tự động load khi robot spawn")
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '-5.0',
            '-y', '-1.0',
            '-z', '0.034',  # Bánh xe chạm đất (wheel_radius)
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])
    
    # Autonomous Drive Node với sim time
    print("🧠 Node 4: Autonomous Drive")
    print("   Subscribe: /scan (LiDAR), /camera/image_raw (Camera)")
    print("   Publish: /cmd_vel (điều khiển xe)")
    autonomous_drive_node = Node(
        package=package_name,
        executable='obstacle_avoidance.py',
        name='autonomous_drive',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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
    print("   1. Robot State Publisher (RSP) - ngay lập tức")
    print("   2. Gazebo Simulator - ngay lập tức")
    print("   3. Spawn Robot Entity - sau 5 giây")
    print("      └─ LiDAR plugin tự động load khi spawn")
    print("   4. Autonomous Drive Node - ngay lập tức")
    print("      └─ Subscribe: /scan (LiDAR), /camera/image_raw (Camera)")
    print("      └─ Publish: /cmd_vel (điều khiển xe)")
    print("=" * 60)
    print("🔍 Để kiểm tra LiDAR, chạy lệnh sau trong terminal khác:")
    print("   ros2 topic list | grep scan")
    print("   ros2 topic echo /scan --once")
    print("=" * 60)
    
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        delayed_spawn_entity,
        autonomous_drive_node,
    ])

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
    package_name = 'xe_lidar'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    # Lấy đường dẫn đến lib directory
    pkg_share_dir = os.path.dirname(pkg_share)  # install/xe_lidar/share -> install/xe_lidar
    pkg_lib_dir = os.path.join(pkg_share_dir, 'lib', package_name)  # install/xe_lidar/lib/xe_lidar
    
    # Log thông tin
    print("=" * 60)
    print("🚀 KHỞI ĐỘNG SIMULATION ACKERMANN")
    print("=" * 60)
    print(f"📦 Package: {package_name}")
    print(f"📁 Share directory: {pkg_share}")
    print(f"📁 Lib directory: {pkg_lib_dir}")
    print("=" * 60)
    
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'road_map.world'),
        description='Path to world file'
    )
    
    world = LaunchConfiguration('world')
    
    # Robot state publisher với sim time (Ackermann)
    print("📡 Node 1: Robot State Publisher (RSP)")
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
            'use_ros2_control': 'false'  # Dùng Gazebo plugin cho simulation (không cần gazebo_ros2_control)
        }.items()
    )
    
    # Gazebo
    print("🌍 Node 2: Gazebo Simulator")
    world_default = os.path.join(pkg_share, 'worlds', 'road_map.world')
    print(f"   World file: {world_default}")
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
    # Spawn ở đầu đường, giữa làn đường bên phải
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
            '-x', '-5.0',  # Đầu đường
            '-y', '-1.0',  # Làn đường bên phải
            '-z', '0.034',  # Bánh xe chạm đất (wheel_radius)
            '-Y', '0.0'    # Hướng về phía trước
        ],
        output='screen'
    )
    
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])
    
    # Không cần controller manager khi dùng Gazebo plugin
    # Gazebo plugin (libgazebo_ros_ackermann_drive.so) sẽ tự động xử lý cmd_vel
    
    # Autonomous Drive Node với sim time (Camera: lane detection, LiDAR: obstacle avoidance)
    # Delay để đợi robot được spawn và LiDAR sẵn sàng
    print("🧠 Node 4: Autonomous Drive (delay 10s)")
    print("   Script: obstacle_avoidance.py")
    # Tìm file script - ưu tiên lib, sau đó source directory
    pkg_lib_file = os.path.join(pkg_lib_dir, 'obstacle_avoidance.py')
    
    # Tìm file script trong các vị trí có thể
    script_path = None
    if os.path.exists(pkg_lib_file):
        script_path = pkg_lib_file
    else:
        # Tìm trong source directory
        pkg_share_abs = get_package_share_directory(package_name)
        # Từ install/xe_lidar/share/xe_lidar -> src/xe_lidar/scripts
        pkg_source_script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(pkg_share_abs))),
            'src', package_name, 'scripts', 'obstacle_avoidance.py'
        )
        if os.path.exists(pkg_source_script):
            script_path = pkg_source_script
        else:
            # Fallback: tìm từ workspace root
            workspace_root = os.path.dirname(os.path.dirname(pkg_share_abs))
            fallback_path = os.path.join(workspace_root, 'src', package_name, 'scripts', 'obstacle_avoidance.py')
            if os.path.exists(fallback_path):
                script_path = fallback_path
    
    # Nếu vẫn không tìm thấy, dùng đường dẫn tương đối từ share
    if not script_path or not os.path.exists(script_path):
        script_path = os.path.join(pkg_share, '..', '..', 'lib', package_name, 'obstacle_avoidance.py')
        # Normalize đường dẫn
        script_path = os.path.normpath(script_path)
    
    # Kiểm tra file có tồn tại không
    if not os.path.exists(script_path):
        # Thử tìm trong thư mục scripts của package share (nếu có)
        alt_path = os.path.join(pkg_share, 'scripts', 'obstacle_avoidance.py')
        if os.path.exists(alt_path):
            script_path = alt_path
        else:
            # Log warning và dùng đường dẫn mặc định
            import sys
            print(f"⚠️  [WARNING] Không tìm thấy obstacle_avoidance.py, đang dùng: {script_path}", file=sys.stderr)
    
    if script_path and os.path.exists(script_path):
        print(f"   ✅ Tìm thấy script: {script_path}")
    else:
        print(f"   ❌ Không tìm thấy script tại: {script_path}")
    
    # Dùng ExecuteProcess với đường dẫn trực tiếp đến script
    autonomous_drive_node = ExecuteProcess(
        cmd=['python3', os.path.abspath(script_path),
             '--ros-args',
             '-r', '__node:=autonomous_drive',
             '-p', 'use_sim_time:=true',
             '-p', 'min_distance:=0.5',
             '-p', 'safe_distance:=0.8',
             '-p', 'max_linear_speed:=0.3',
             '-p', 'max_angular_speed:=1.0',
             '-p', 'front_angle_range:=60',
             '-p', 'use_camera:=true',
             '-p', 'camera_topic:=/camera/image_raw'],
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'}
    )
    
    # Delay autonomous drive node để đợi robot spawn và LiDAR sẵn sàng
    delayed_autonomous_drive = TimerAction(period=10.0, actions=[autonomous_drive_node])
    
    print("=" * 60)
    print("📋 TÓM TẮT CÁC NODE:")
    print("   1. Robot State Publisher (RSP) - ngay lập tức")
    print("   2. Gazebo Simulator - ngay lập tức")
    print("   3. Spawn Robot Entity - sau 5 giây")
    print("      └─ LiDAR plugin tự động load khi spawn")
    print("   4. Autonomous Drive Node - sau 10 giây")
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
        delayed_autonomous_drive,
    ])


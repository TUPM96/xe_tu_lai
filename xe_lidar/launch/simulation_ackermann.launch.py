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
    
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'road_map.world'),
        description='Path to world file'
    )
    
    world = LaunchConfiguration('world')
    
    # Robot state publisher với sim time (Ackermann)
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
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '-5.0',  # Đầu đường
            '-y', '-1.0',  # Làn đường bên phải
            '-z', '0.1',
            '-Y', '0.0'    # Hướng về phía trước
        ],
        output='screen'
    )
    
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])
    
    # Không cần controller manager khi dùng Gazebo plugin
    # Gazebo plugin (libgazebo_ros_ackermann_drive.so) sẽ tự động xử lý cmd_vel
    
    # Autonomous Drive Node với sim time (Camera: lane detection, LiDAR: obstacle avoidance)
    # Delay để đợi robot được spawn và LiDAR sẵn sàng
    # Tạo thư mục libexec và symlink nếu chưa có (để ROS2 tìm thấy)
    import subprocess
    pkg_libexec_dir = os.path.join(pkg_share_dir, 'libexec', package_name)
    pkg_lib_file = os.path.join(pkg_lib_dir, 'obstacle_avoidance.py')
    pkg_libexec_file = os.path.join(pkg_libexec_dir, 'obstacle_avoidance.py')
    
    # Tạo thư mục libexec nếu chưa có
    os.makedirs(pkg_libexec_dir, exist_ok=True)
    
    # Tạo symlink nếu chưa có
    if not os.path.exists(pkg_libexec_file):
        if os.path.exists(pkg_lib_file):
            os.symlink(pkg_lib_file, pkg_libexec_file)
    
    # Dùng Node với executable (ROS2 sẽ tìm trong libexec)
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
    
    # Delay autonomous drive node để đợi robot spawn và LiDAR sẵn sàng
    delayed_autonomous_drive = TimerAction(period=8.0, actions=[autonomous_drive_node])
    
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        delayed_spawn_entity,
        delayed_autonomous_drive,
    ])


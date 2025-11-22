import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler, ExecuteProcess, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description(context):
    """Generate launch description với world path đã được xử lý"""
    package_name = 'xe_lidar'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    # Lấy world argument và xử lý đường dẫn
    world_arg = context.launch_configurations.get('world', 'road_map.world')
    
    # Nếu chỉ là tên file (không có '/' hoặc '\'), thêm đường dẫn package
    if '/' not in world_arg and '\\' not in world_arg:
        world_path = os.path.join(pkg_share, 'worlds', world_arg)
    else:
        world_path = world_arg
    
    # Robot state publisher với sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false'  # Dùng Gazebo plugin thay vì ros2_control
        }.items()
    )
    
    # Gazebo với world path đã xử lý
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot vào Gazebo (delay để đợi Gazebo khởi động)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', '-5.0',
            '-y', '-1.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])
    
    # Autonomous Drive Node với sim time
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
    
    return [
        rsp,
        gazebo,
        delayed_spawn_entity,
        autonomous_drive_node,
    ]


def generate_launch_description_wrapper():
    """Wrapper function để tạo launch description"""
    package_name = 'xe_lidar'
    
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='road_map.world',
        description='World file name (e.g., test_map.world) or full path'
    )
    
    return LaunchDescription([
        world_file_arg,
        OpaqueFunction(function=generate_launch_description),
    ])

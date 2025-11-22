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
    
    # World file argument
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'road_map.world'),
        description='Path to world file'
    )
    
    world = LaunchConfiguration('world')
    
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
    
    # Autonomous Drive Node với sim time (Camera: lane detection, LiDAR: obstacle avoidance)
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
    
    # RViz2 để xem simulation (tùy chọn)
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'main.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        delayed_spawn_entity,
        autonomous_drive_node,
        # rviz_node,  # Uncomment nếu muốn tự động mở RViz
    ])


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
            'use_ros2_control': 'true'  # Dùng ros2_control cho Ackermann
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
    
    # Controller manager
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
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )
    
    delayed_ackermann_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_controller_spawner]
        )
    )
    
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
    
    return LaunchDescription([
        world_file_arg,
        rsp,
        gazebo,
        delayed_spawn_entity,
        delayed_controller_manager,
        delayed_ackermann_spawner,
        delayed_joint_broad_spawner,
        autonomous_drive_node,
    ])


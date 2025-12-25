import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'xe_lidar'
    
    # Robot State Publisher (RSP) để visualize khung xe
    print("📡 Robot State Publisher (RSP) - Ackermann 4 bánh")
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
    print("📊 Joint State Publisher - publish joint states với giá trị mặc định")
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port của Arduino (ví dụ: /dev/ttyACM0 hoặc /dev/ttyUSB0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate của Serial (mặc định: 115200)'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect',
        default_value='true',
        description='Tự động phát hiện cổng Arduino nếu port được chỉ định không tồn tại (true/false). Nếu chỉ định port cụ thể, sẽ ưu tiên dùng port đó.'
    )
    
    # Arduino Bridge Node
    print("🔌 Arduino Bridge Node")
    arduino_bridge_node = Node(
        package=package_name,
        executable='arduino_bridge.py',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'auto_detect': LaunchConfiguration('auto_detect'),
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        auto_detect_arg,
        rsp,
        joint_state_publisher_node,  # Cần cho RSP hiển thị khung xe
        arduino_bridge_node,
    ])



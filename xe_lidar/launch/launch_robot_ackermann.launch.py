import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'xe_lidar'

    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(package_name), 'launch', 'rsp_ackermann.launch.py')]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items())

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(package="twist_mux", executable="twist_mux", parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/ackermann_steering_controller/cmd_vel_unstamped')])

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers_ackermann.yaml')

    controller_manager = Node(package="controller_manager", executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file])

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    ackermann_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller"], )

    delayed_ackermann_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=controller_manager, on_start=[ackermann_controller_spawner], ))

    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"], )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(target_action=controller_manager, on_start=[joint_broad_spawner], ))

    # Launch them all!
    return LaunchDescription([rsp,
        twist_mux, delayed_controller_manager, delayed_ackermann_spawner, delayed_joint_broad_spawner])


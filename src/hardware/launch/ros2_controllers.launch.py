from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share_directory = get_package_share_directory('hardware')
    controllers_yaml = os.path.join(pkg_share_directory, 'config', 'arm_controllers.yaml')

    # ros2_control node (controller manager)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_yaml],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    # Delay spawning to let controller_manager initialize
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner, joint_trajectory_spawner]
    )
    
    return LaunchDescription([
        controller_manager,
        delayed_spawners
    ])

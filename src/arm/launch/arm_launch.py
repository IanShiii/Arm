from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_directory = get_package_share_directory('arm')
    urdf_file = os.path.join(pkg_share_directory, 'urdf', 'arm.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share_directory, 'config', 'arm_controllers.yaml')
    rviz_config_file = os.path.join(pkg_share_directory, 'config', 'rviz_config.rviz')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

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

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(delayed_spawners)
    ld.add_action(rviz2)

    return ld

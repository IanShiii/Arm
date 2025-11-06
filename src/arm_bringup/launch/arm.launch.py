from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'arm.urdf.xacro')
    rviz_config_file = os.path.join(get_package_share_directory('arm_bringup'), 'config', 'rviz_config.rviz')
    kinematics_config_file = os.path.join(get_package_share_directory('moveit2'), 'config', 'kinematics.yaml')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hardware'),
                'launch',
                'ros2_controllers.launch.py'
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('moveit2'),
                'launch',
                'move_group.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    planner_node = Node(
        package='planning',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'arm.kinematics_solver': 'trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin',
            'arm.position_only_ik': True,
            'arm.solve_type': 'Speed',
            'arm.kinematics_solver_timeout': 0.005,
            'use_sim_time': True
        }]
    )

    delayed_planner_node = TimerAction(
        period=3.0,  # seconds
        actions=[
            planner_node
        ]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_controllers_launch,
        move_group_launch,
        # delayed_planner_node,
        joy_node,
        rviz2
    ])

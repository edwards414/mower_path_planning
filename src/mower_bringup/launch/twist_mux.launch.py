import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    twist_mux_config = os.path.join(
        get_package_share_directory('mower_bringup'),
        'config',
        'twist_mux_topics.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/cmd_vel_guard_input')],
    )

    manual_velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='manual_velocity_guard',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_command_session': True,
        }],
        remappings=[
            ('cmd_vel_in', '/app_joy_cmd'),
            ('cmd_vel_out', '/joy_cmd'),
            ('command_clock', '/manual_command_clock'),
        ],
    )

    velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='velocity_command_guard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_guard_input'),
            ('cmd_vel_out', '/drivetrain_guarded_cmd_vel'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        manual_velocity_guard,
        twist_mux,
        velocity_guard,
    ])

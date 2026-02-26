import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    twist_mux_config = os.path.join(
        get_package_share_directory('nav2_gps_waypoint_follower'),
        'config',
        'twist_mux_topics.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        # 將 twist_mux 的輸出 /cmd_vel_out remap 到 diff_drive_controller 接收的 topic
        remappings=[('/cmd_vel_out', '/diff_controller/cmd_vel')],
    )

    return LaunchDescription([
        twist_mux,
    ])

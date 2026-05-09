from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    flutter_adapter_node = Node(
        package='mower_mission',
        executable='flutter_adapter_node',
        name='flutter_adapter',
        output='screen',
    )

    return LaunchDescription([
        flutter_adapter_node,
    ])

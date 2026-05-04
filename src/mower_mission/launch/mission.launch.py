from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path_record_node = Node(
        package='mower_mission',
        executable='path_record_node',
        name='path_record_node',
        output='screen',
    )

    map_manage_node = Node(
        package='mower_mission',
        executable='map_manage_node',
        name='map_manage_node',
        output='screen',
    )

    coverage_node = Node(
        package='mower_mission',
        executable='coverage_node',
        name='coverage_node',
        output='screen',
    )

    return LaunchDescription([
        path_record_node,
        map_manage_node,
        coverage_node,
    ])

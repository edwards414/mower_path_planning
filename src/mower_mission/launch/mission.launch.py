from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_temp_dock_pose_publisher = LaunchConfiguration(
        'launch_temp_dock_pose_publisher'
    )

    declare_launch_temp_dock_pose_publisher = DeclareLaunchArgument(
        'launch_temp_dock_pose_publisher',
        default_value='false',
        description='Launch temporary fixed /detected_dock_pose publisher',
    )

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

    nav_action_server = Node(
        package='mower_mission',
        executable='nav_action_server',
        output='screen',
    )

    docking_manager_node = Node(
        package='mower_mission',
        executable='docking_manager_node',
        name='docking_manager_node',
        output='screen',
    )

    temp_dock_pose_publisher = Node(
        package='mower_mission',
        executable='temp_dock_pose_publisher',
        name='temp_dock_pose_publisher',
        output='screen',
        condition=IfCondition(launch_temp_dock_pose_publisher),
    )

    return LaunchDescription([
        declare_launch_temp_dock_pose_publisher,
        path_record_node,
        map_manage_node,
        coverage_node,
        nav_action_server,
        docking_manager_node,
        temp_dock_pose_publisher,
    ])

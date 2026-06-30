import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    heartbeat_source_topic = LaunchConfiguration('heartbeat_source_topic')

    declare_heartbeat_source_topic = DeclareLaunchArgument(
        'heartbeat_source_topic',
        default_value='/odom',
        description='Topic whose freshness drives the /robot/online heartbeat',
    )

    auto_coverage = LaunchConfiguration('auto_coverage')

    declare_auto_coverage = DeclareLaunchArgument(
        'auto_coverage',
        default_value='false',
        description='After launch, auto-drive load_zone_list -> create_free_space'
                    ' -> create_risk_map -> generate_coverage_path so a path is'
                    ' ready (uses saved zones in zone_record/).',
    )

    # Auto-coverage driver (only when auto_coverage:=true). A node that
    # sequences load_zone_list -> create_free_space -> create_risk_map ->
    # generate_coverage_path, WAITING for each async step to finish (e.g.
    # /free_space_inflated, /risk_map_inflated) before the next — robust to
    # node-startup timing, unlike fixed timers.
    auto_coverage_node = Node(
        package='mower_mission',
        executable='auto_coverage_node',
        name='auto_coverage',
        output='screen',
        condition=IfCondition(auto_coverage),
    )

    # rosbridge + rosapi so the Flutter app can connect to `make mission`.
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_bringup'),
                'launch', 'rosbridge.launch.py',
            )
        )
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

    flutter_adapter_node = Node(
        package='mower_mission',
        executable='flutter_adapter_node',
        name='flutter_adapter',
        output='screen',
    )

    docking_manager_node = Node(
        package='mower_mission',
        executable='docking_manager_node',
        name='docking_manager_node',
        output='screen',
    )

    # Robot liveness heartbeat: publishes /robot/online (LWT-style). Uses wall
    # clock (use_sim_time=False) so staleness reflects real message arrival.
    heartbeat_node = Node(
        package='mower_mission',
        executable='heartbeat_node',
        name='robot_heartbeat',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'source_topic': heartbeat_source_topic,
            'stale_timeout_s': 2.0,
            'publish_rate_hz': 2.0,
        }],
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
        declare_heartbeat_source_topic,
        declare_auto_coverage,
        rosbridge_launch,
        path_record_node,
        map_manage_node,
        coverage_node,
        nav_action_server,
        flutter_adapter_node,
        docking_manager_node,
        heartbeat_node,
        temp_dock_pose_publisher,
        auto_coverage_node,
    ])

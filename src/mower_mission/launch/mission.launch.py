import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rosbridge_address = LaunchConfiguration('rosbridge_address')
    zone_record_dir = LaunchConfiguration('zone_record_dir')
    sites_dir = LaunchConfiguration('sites_dir')
    require_navigation_health = LaunchConfiguration(
        'require_navigation_health'
    )
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for mission nodes',
    )
    declare_rosbridge_address = DeclareLaunchArgument(
        'rosbridge_address',
        default_value='127.0.0.1',
        description='Address passed to rosbridge_websocket',
    )
    declare_zone_record_dir = DeclareLaunchArgument(
        'zone_record_dir',
        default_value='zone_record',
        description='Persistent directory for active zone/risk/channel JSON',
    )
    declare_sites_dir = DeclareLaunchArgument(
        'sites_dir',
        default_value='~/.mower/sites',
        description='Persistent directory for named WGS84 site snapshots',
    )
    declare_require_navigation_health = DeclareLaunchArgument(
        'require_navigation_health',
        default_value='true',
        description='Require fresh pose and precise GPS before/during Nav2',
    )
    declare_gps_fix_topic = DeclareLaunchArgument(
        'gps_fix_topic',
        default_value='/fix',
        description='GPS fix topic that is also consumed by navsat_transform',
    )
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(auto_coverage),
    )

    # rosbridge + rosapi so the Flutter app can connect to `make mission`.
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_bringup'),
                'launch', 'rosbridge.launch.py',
            )
        ),
        launch_arguments={'address': rosbridge_address}.items(),
    )

    # ── Bag recorder ─────────────────────────────────────────────────────────
    # Recording is opt-in until a retention or working R2 upload policy is
    # configured; otherwise an unattended process can fill the mower disk.
    record = LaunchConfiguration('record')
    declare_record = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Opt in to mission recording (requires retention/upload)',
    )
    robot_id = LaunchConfiguration('robot_id')
    declare_robot_id = DeclareLaunchArgument('robot_id', default_value='mower')
    output_root = LaunchConfiguration('output_root')
    declare_output_root = DeclareLaunchArgument(
        'output_root', default_value='~/mower_bags')
    r2_env_file = LaunchConfiguration('r2_env_file')
    declare_r2_env_file = DeclareLaunchArgument(
        'r2_env_file',
        default_value='',
        description="Gitignored .env with R2 creds; '' = read R2_* env vars.",
    )
    git_repo_dir = LaunchConfiguration('git_repo_dir')
    declare_git_repo_dir = DeclareLaunchArgument(
        'git_repo_dir', default_value='')

    # Gracefully skip if mower_recorder isn't built.
    recorder_entries = []
    try:
        recorder_entries.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mower_recorder'),
                    'launch', 'record.launch.py',
                )
            ),
            condition=IfCondition(record),
            launch_arguments={
                'robot_id': robot_id,
                'output_root': output_root,
                'autostart': 'true',
                'r2_env_file': r2_env_file,
                'git_repo_dir': git_repo_dir,
                'gps_topic': gps_fix_topic,
            }.items(),
        ))
    except Exception:
        pass

    path_record_node = Node(
        package='mower_mission',
        executable='path_record_node',
        name='path_record_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_dir': zone_record_dir,
            'sites_dir': sites_dir,
        }],
    )

    map_manage_node = Node(
        package='mower_mission',
        executable='map_manage_node',
        # Keep the node's native name: FlutterAdapter, mower_qt and the public
        # parameter contract all address /map_manage/{get,set}_parameters.
        name='map_manage',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    coverage_node = Node(
        package='mower_mission',
        executable='coverage_node',
        # Node name must stay 'boustrophedon_coverage': the flutter_adapter
        # (/boustrophedon_coverage/get_parameters), the Flutter app + mower_qt
        # (/boustrophedon_coverage/set_parameters) and system_test all target it.
        name='boustrophedon_coverage',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav_action_server = Node(
        package='mower_mission',
        executable='nav_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_navigation_health': require_navigation_health,
            'gps_fix_topic': gps_fix_topic,
        }],
    )

    flutter_adapter_node = Node(
        package='mower_mission',
        executable='flutter_adapter_node',
        name='flutter_adapter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_temp_dock_pose_publisher),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rosbridge_address,
        declare_zone_record_dir,
        declare_sites_dir,
        declare_require_navigation_health,
        declare_gps_fix_topic,
        declare_launch_temp_dock_pose_publisher,
        declare_heartbeat_source_topic,
        declare_auto_coverage,
        declare_record,
        declare_robot_id,
        declare_output_root,
        declare_r2_env_file,
        declare_git_repo_dir,
        rosbridge_launch,
        *recorder_entries,
        path_record_node,
        map_manage_node,
        coverage_node,
        nav_action_server,
        flutter_adapter_node,
        heartbeat_node,
        temp_dock_pose_publisher,
        auto_coverage_node,
    ])

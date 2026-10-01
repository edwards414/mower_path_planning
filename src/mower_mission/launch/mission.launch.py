import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# `rust_daemon:=true` moves every enabled mower_rs module into one mower_rsd
# process (started by robot.launch.py), so the separate binaries must not also
# start. The Python fallbacks keep their own `UnlessCondition(rust_*)`.
_TRUE = "('true', '1', 'yes', 'on')"


def _rust_binary(flag, rust_daemon):
    """Run the separate mower_rs binary: switch on and daemon not running."""
    return IfCondition(PythonExpression([
        "'", flag, "'.lower() in ", _TRUE,
        " and '", rust_daemon, "'.lower() not in ", _TRUE,
    ]))


def _rust_only_binary(rust_daemon):
    """Run a mower_rs binary that has no rclpy fallback (so no rust_* switch)
    as its own process unless mower_rsd already hosts it: the exact
    complement of robot.launch.py's _truthy(rust_daemon), which always puts
    such a module in the daemon's set."""
    return IfCondition(PythonExpression([
        "'", rust_daemon, "'.strip().lower() not in ", _TRUE,
    ]))


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
        default_value='/odom_slow',
        description='Topic whose freshness drives the /robot/online heartbeat',
    )

    rust_daemon = LaunchConfiguration('rust_daemon')

    declare_rust_daemon = DeclareLaunchArgument(
        'rust_daemon',
        default_value='false',
        description='The enabled mower_rs modules, plus the always-on '
                    'coverage planner, run inside one mower_rsd process '
                    'started by robot.launch.py (one r2r Context, one DDS '
                    'participant), so the separate binaries stay down here; '
                    'the rclpy fallbacks still follow their own rust_* '
                    'switch. Only meaningful when robot.launch.py includes '
                    'this file: standalone, true leaves those modules '
                    '(coverage included) unhosted. See src/mower_rs/README.md.',
    )

    rust_status = LaunchConfiguration('rust_status')

    declare_rust_status = DeclareLaunchArgument(
        'rust_status',
        default_value='false',
        description='Run the mower_rs (Rust) robot_status process instead of '
                    'the rclpy heartbeat_node + robot_info_node + '
                    'telemetry_node. Same topics, services and files; see '
                    'src/mower_rs/README.md.',
    )

    rust_adapter = LaunchConfiguration('rust_adapter')
    rust_record = LaunchConfiguration('rust_record')

    declare_rust_record = DeclareLaunchArgument(
        'rust_record',
        default_value='false',
        description='Run the mower_rs (Rust) mower_record process instead of '
                    'the rclpy path_record_node (same services, topics and '
                    'files).',
    )

    rust_nav = LaunchConfiguration('rust_nav')

    declare_rust_nav = DeclareLaunchArgument(
        'rust_nav',
        default_value='false',
        description='Run the mower_rs (Rust) mower_nav process instead of '
                    'the rclpy nav_action_server (same actions, services, '
                    'safety heartbeat and admission rules).',
    )

    rust_battery = LaunchConfiguration('rust_battery')

    declare_rust_battery = DeclareLaunchArgument(
        'rust_battery',
        default_value='false',
        description='Run the mower_rs (Rust) mower_battery process instead '
                    'of the rclpy battery_state_node (same estimator, '
                    'topics and parameters).',
    )

    rust_pid_autotune = LaunchConfiguration('rust_pid_autotune')

    declare_rust_pid_autotune = DeclareLaunchArgument(
        'rust_pid_autotune',
        default_value='false',
        description='Run the mower_rs (Rust) mower_pid_autotune process '
                    'instead of the rclpy pid_autotune_node (same service, '
                    'status JSON, base side channels and parameters).',
    )

    rust_map = LaunchConfiguration('rust_map')

    declare_rust_map = DeclareLaunchArgument(
        'rust_map',
        default_value='false',
        description='Run the mower_rs (Rust) mower_map process instead of '
                    'the rclpy map_manage_node (same services, latched maps, '
                    'parameters and occupancy bytes).',
    )

    declare_rust_agent = DeclareLaunchArgument(
        'rust_agent',
        default_value='false',
        description='Run the mower_rs fleet agent instead of the Python '
                    'mower_agent (same backend protocol).',
    )

    declare_rust_bridge = DeclareLaunchArgument(
        'rust_bridge',
        default_value='false',
        description='Run the mower_rs WebSocket bridge instead of '
                    'rosbridge_auth_proxy + rosbridge_websocket + rosapi.',
    )

    declare_rust_adapter = DeclareLaunchArgument(
        'rust_adapter',
        default_value='false',
        description='Run the mower_rs (Rust) mower_adapter process instead of '
                    'the rclpy flutter_adapter_node (same /adapter/* topics).',
    )

    # The two switches mower.launch.py starts mower_base / mower_localize with
    # (robot.launch.py passes the same value to both files). Those modules
    # publish /odom_slow and /odometry/global_slow themselves, so each one
    # holds down the topic_tools throttle that would otherwise make that copy
    # (odom_throttle / global_odom_throttle below). Independent of
    # rust_daemon: the modules do the same inside mower_rsd.
    rust_base = LaunchConfiguration('rust_base')

    declare_rust_base = DeclareLaunchArgument(
        'rust_base',
        default_value='false',
        description='mower_rs mower_base publishes /odom (and /odom_slow), '
                    'so odom_throttle does not start. Must match the '
                    'rust_base given to mower.launch.py.',
    )

    rust_localize = LaunchConfiguration('rust_localize')

    declare_rust_localize = DeclareLaunchArgument(
        'rust_localize',
        default_value='false',
        description='mower_rs mower_localize publishes /odometry/global (and '
                    '/odometry/global_slow), so global_odom_throttle does not '
                    'start. Must match the rust_localize given to '
                    'mower.launch.py.',
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
        launch_arguments={
            'address': rosbridge_address,
            'rust_bridge': LaunchConfiguration('rust_bridge'),
            'rust_agent': LaunchConfiguration('rust_agent'),
            'rust_daemon': rust_daemon,
        }.items(),
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
        condition=UnlessCondition(rust_record),
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_dir': zone_record_dir,
            'sites_dir': sites_dir,
            'robot_pose_source_topic': '/odometry/global_slow',
        }],
    )

    # rust_record:=true -- the same recorder as an r2r process
    # (src/mower_rs/crates/mower_record): identical replies, files and lists.
    mower_record_rs = Node(
        package='mower_rs',
        executable='mower_record',
        name='path_record_node',
        output='screen',
        condition=_rust_binary(rust_record, rust_daemon),
        parameters=[{
            'save_dir': zone_record_dir,
            'sites_dir': sites_dir,
            'robot_pose_source_topic': '/odometry/global_slow',
        }],
    )

    map_manage_node = Node(
        package='mower_mission',
        executable='map_manage_node',
        # Keep the node's native name: FlutterAdapter, mower_qt and the public
        # parameter contract all address /map_manage/{get,set}_parameters.
        name='map_manage',
        output='screen',
        condition=UnlessCondition(rust_map),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # rust_map:=true -- the same map manager as an r2r process
    # (src/mower_rs/crates/mower_map): pixel-exact OpenCV ports, identical
    # /map_grid bytes.
    mower_map_rs = Node(
        package='mower_rs',
        executable='mower_map',
        name='map_manage',
        output='screen',
        condition=_rust_binary(rust_map, rust_daemon),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # The coverage planner: an r2r process linked directly against the
    # mower_coverage_core Rust library (src/mower_rs/crates/mower_coverage).
    # There is no rclpy version, so it has no rust_* switch: it always runs,
    # as this binary or, with rust_daemon:=true, as the 'coverage' module of
    # mower_rsd. Node name must stay 'boustrophedon_coverage': the
    # flutter_adapter (/boustrophedon_coverage/get_parameters), the Flutter
    # app + mower_qt (/boustrophedon_coverage/set_parameters) and system_test
    # all target it.
    mower_coverage_rs = Node(
        package='mower_rs',
        executable='mower_coverage',
        name='boustrophedon_coverage',
        output='screen',
        condition=_rust_only_binary(rust_daemon),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav_action_server = Node(
        package='mower_mission',
        executable='nav_action_server',
        output='screen',
        condition=UnlessCondition(rust_nav),
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_navigation_health': require_navigation_health,
            'gps_fix_topic': gps_fix_topic,
        }],
    )

    # rust_nav:=true -- the same navigation coordinator as an r2r process
    # (src/mower_rs/crates/mower_nav): node name, actions, services, the
    # 20 Hz /navigation_coordinator_lock heartbeat and every admission /
    # health rule are unchanged; differential-tested against the rclpy node.
    mower_nav_rs = Node(
        package='mower_rs',
        executable='mower_nav',
        name='nav_action_server',
        output='screen',
        condition=_rust_binary(rust_nav, rust_daemon),
        parameters=[{
            'require_navigation_health': require_navigation_health,
            'gps_fix_topic': gps_fix_topic,
        }],
    )

    flutter_adapter_node = Node(
        package='mower_mission',
        executable='flutter_adapter_node',
        name='flutter_adapter',
        output='screen',
        condition=UnlessCondition(rust_adapter),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_pose_source_topic': '/odometry/global_slow',
        }],
    )

    # rust_adapter:=true -- the same relay as an r2r process
    # (src/mower_rs/crates/mower_adapter): identical /adapter/* JSON, a 214 KB
    # map grid encodes in ~2 ms instead of ~60 ms.
    mower_adapter_rs = Node(
        package='mower_rs',
        executable='mower_adapter',
        name='flutter_adapter',
        output='screen',
        condition=_rust_binary(rust_adapter, rust_daemon),
        parameters=[{'robot_pose_source_topic': '/odometry/global_slow'}],
    )

    # /odom is 50 Hz and rclpy costs ~5 ms per Odometry message on the
    # LubanCat (throttled A55), i.e. ~28 % of a core per Python subscriber.
    # The status nodes below only need a few Hz, so they read this C++
    # throttled copy instead.
    #
    # rust_base:=true -- mower_base publishes /odom_slow itself, from every
    # /odom it publishes, by the same rule (mower_rs_common::throttle;
    # odom_slow_topic / odom_slow_rate_hz in mower_rsd.yaml), which saves
    # this process and its ~1.3 ms per /odom message.
    odom_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='odom_throttle',
        output='screen',
        condition=UnlessCondition(rust_base),
        arguments=['messages', '/odom', '5.0', '/odom_slow'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Same idea for the map-frame pose. /odometry/global is the EKF map
    # filter's 30 Hz map -> base_footprint estimate; flutter_adapter and
    # path_record_node are pointed at this 5 Hz copy (robot_pose_source_topic)
    # instead of running a tf2 TransformListener, which would cost each of
    # them the full 77 Hz /tf stream (~50 % of a core per node in rclpy).
    #
    # rust_localize:=true -- mower_localize's ekf_filter_node_map publishes
    # /odometry/global_slow itself, by the same rule (odometry_slow_topic /
    # odometry_slow_rate_hz in mower_rsd.yaml).
    global_odom_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='global_odom_throttle',
        output='screen',
        condition=UnlessCondition(rust_localize),
        arguments=[
            'messages', '/odometry/global', '5.0', '/odometry/global_slow',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Robot liveness heartbeat: publishes /robot/online (LWT-style). Uses wall
    # clock (use_sim_time=False) so staleness reflects real message arrival.
    heartbeat_node = Node(
        package='mower_mission',
        executable='heartbeat_node',
        name='robot_heartbeat',
        output='screen',
        condition=UnlessCondition(rust_status),
        parameters=[{
            'use_sim_time': False,
            'source_topic': heartbeat_source_topic,
            'stale_timeout_s': 2.0,
            'publish_rate_hz': 2.0,
        }],
    )

    # Version / update state for the app (/robot/info) and the host updater
    # hand-off (/system/update, /system/restart).
    robot_info_node = Node(
        package='mower_mission',
        executable='robot_info_node',
        name='robot_info',
        output='screen',
        condition=UnlessCondition(rust_status),
        parameters=[{'use_sim_time': False, 'odom_topic': '/odom_slow'}],
    )

    # Read-only parameter feed for the desktop dashboard (/robot/telemetry):
    # GPS/RTK, IMU, wheel PID, lights, LTE signal, versions in one JSON topic.
    telemetry_node = Node(
        package='mower_mission',
        executable='telemetry_node',
        name='telemetry',
        output='screen',
        condition=UnlessCondition(rust_status),
        parameters=[{
            'use_sim_time': False,
            'gps_fix_topic': gps_fix_topic,
            'odom_topic': '/odom_slow',
        }],
    )

    # rust_status:=true -- the three status nodes above as one r2r process
    # (src/mower_rs/crates/robot_status). Same topics, services, state-dir
    # files and LED behaviour; ~1 % of a core instead of ~30 %.
    robot_status_rs = Node(
        package='mower_rs',
        executable='robot_status',
        name='robot_status',
        output='screen',
        condition=_rust_binary(rust_status, rust_daemon),
        parameters=[{
            'heartbeat_source_topic': heartbeat_source_topic,
            'heartbeat_stale_timeout_s': 2.0,
            'heartbeat_publish_rate_hz': 2.0,
            'odom_topic': '/odom_slow',
            'gps_fix_topic': gps_fix_topic,
        }],
    )

    # Real-pack /battery_state from the RS485 pack meter (0x89) with the STM32
    # ADC (0x8A) as fallback. The battery simulator stays off on the real robot.
    battery_state_node = Node(
        package='mower_mission',
        executable='battery_state_node',
        name='battery_state',
        output='screen',
        condition=UnlessCondition(rust_battery),
        parameters=[{
            'use_sim_time': False,
            # Pack voltage from the RS485 meter in the pack lead; the
            # charger is "present" while the pack sits at its CV.
            'charger_present_min_v': 25.0,
            # Flip to True (and set capacity_ah) once the meter's shunt
            # carries the pack current -> coulomb counting (docs/BATTERY.md).
            'meter_current_wired': False,
            'capacity_ah': 0.0,
        }],
    )

    # rust_battery:=true -- the same estimator as an r2r process
    # (src/mower_rs/crates/mower_battery): identical BatteryState output.
    mower_battery_rs = Node(
        package='mower_rs',
        executable='mower_battery',
        name='battery_state',
        output='screen',
        condition=_rust_binary(rust_battery, rust_daemon),
        parameters=[{
            'charger_present_min_v': 25.0,
            'meter_current_wired': False,
            'capacity_ah': 0.0,
        }],
    )

    # Wheel PID auto-tune (dashboard button -> /pid_autotune service). Drives
    # the wheels through open-loop steps via the mower_hardware side channels,
    # so it only makes sense on the real base.
    pid_autotune_node = Node(
        package='mower_mission',
        executable='pid_autotune_node',
        name='pid_autotune',
        output='screen',
        condition=UnlessCondition(rust_pid_autotune),
        parameters=[{'use_sim_time': False}],
    )

    # rust_pid_autotune:=true -- the same identification + SIMC maths and
    # session state machine as an r2r process
    # (src/mower_rs/crates/mower_pid_autotune).
    mower_pid_autotune_rs = Node(
        package='mower_rs',
        executable='mower_pid_autotune',
        name='pid_autotune',
        output='screen',
        condition=_rust_binary(rust_pid_autotune, rust_daemon),
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
        declare_rust_daemon,
        declare_rust_status,
        declare_rust_adapter,
        declare_rust_record,
        declare_rust_nav,
        declare_rust_battery,
        declare_rust_pid_autotune,
        declare_rust_map,
        declare_rust_agent,
        declare_rust_bridge,
        declare_rust_base,
        declare_rust_localize,
        declare_auto_coverage,
        declare_record,
        declare_robot_id,
        declare_output_root,
        declare_r2_env_file,
        declare_git_repo_dir,
        rosbridge_launch,
        odom_throttle,
        global_odom_throttle,
        *recorder_entries,
        path_record_node,
        mower_record_rs,
        map_manage_node,
        mower_map_rs,
        mower_coverage_rs,
        nav_action_server,
        mower_nav_rs,
        flutter_adapter_node,
        mower_adapter_rs,
        heartbeat_node,
        robot_info_node,
        telemetry_node,
        robot_status_rs,
        battery_state_node,
        mower_battery_rs,
        pid_autotune_node,
        mower_pid_autotune_rs,
        temp_dock_pose_publisher,
        auto_coverage_node,
    ])

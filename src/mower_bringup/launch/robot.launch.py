"""Launch the real mower stack and its app-facing mission services once."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enforce_production_safety(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    require_health = LaunchConfiguration(
        'require_navigation_health'
    ).perform(context)
    gps_fix_topic = LaunchConfiguration('gps_fix_topic').perform(context)
    if use_sim_time.strip().lower() not in {'0', 'false', 'no', 'off'}:
        raise RuntimeError(
            'robot.launch.py is production-only and requires '
            'use_sim_time:=false'
        )
    if require_health.strip().lower() not in {'1', 'true', 'yes', 'on'}:
        raise RuntimeError(
            'robot.launch.py forbids disabling navigation health checks; '
            'use a simulation/test launch for development'
        )
    if gps_fix_topic.strip() != '/fix':
        raise RuntimeError(
            'robot.launch.py requires the canonical /fix GPS topic; remap '
            'the receiver output to /fix instead of changing this contract'
        )
    return []


# rust_daemon:=true -- every enabled mower_rs module in ONE process
# (src/mower_rs/crates/mower_rsd) instead of one binary each: one r2r Context,
# so one DDS participant and one discovery/executor thread set rather than
# eleven to fourteen (docs/ROS_FREE_PLAN.md Phase A5, ~35 % of a core on the
# LubanCat is mostly that overhead). Node names, namespaces, topics, services,
# actions and parameters are unchanged, so nothing else on the graph — or in
# the app — can tell the difference.
#
# The module set is derived from the same rust_* switches the separate
# binaries use, so a roll-back is `rust_daemon:=false` and nothing else. The
# rclpy fallbacks are untouched: a module whose rust_* switch is false still
# runs as its Python node.
_DAEMON_MODULES = (
    # module id      launch switch
    ('base', 'rust_base'),
    ('status', 'rust_status'),
    ('guards', 'rust_guards'),
    ('imu', 'rust_imu'),
    ('gps', 'enable_gps'),
    ('localize', 'rust_localize'),
    ('map', 'rust_map'),
    ('coverage', 'rust_coverage'),
    ('nav', 'rust_nav'),
    ('record', 'rust_record'),
    ('adapter', 'rust_adapter'),
    ('battery', 'rust_battery'),
    ('pid_autotune', 'rust_pid_autotune'),
    ('bridge', 'rust_bridge'),
    ('agent', 'rust_agent'),
)

# Remappings the separate binaries get from their launch `remappings=`. In one
# process they have to be scoped by node name (`-r <node>:<from>:=<to>`, which
# rcl applies per node); an unprefixed rule would hit every node.
_DAEMON_REMAPS = {
    'imu': ['imu:imu/data_raw:=imu/data'],
    'guards': [
        'manual_velocity_guard:cmd_vel_in:=/app_joy_cmd',
        'manual_velocity_guard:cmd_vel_out:=/joy_cmd',
        'manual_velocity_guard:command_clock:=/manual_command_clock',
        'velocity_command_guard:cmd_vel_in:=/cmd_vel_guard_input',
        'velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel',
    ],
    # dual_ekf_navsat.launch.py's remappings= for the three localization
    # nodes; navsat's gps/fix rule is added with the gps_fix_topic value.
    'localize': [
        'ekf_filter_node_odom:odometry/filtered:=odometry/local',
        'ekf_filter_node_odom:imu:=imu/data',
        'ekf_filter_node_map:odometry/filtered:=odometry/global',
        'ekf_filter_node_map:imu:=imu/data',
        'navsat_transform:imu:=imu/data',
        'navsat_transform:odometry/filtered:=odometry/global',
    ],
}


def _truthy(value):
    return value.strip().lower() in {'true', '1', 'yes', 'on'}


def _rust_daemon_node(context):
    """The single mower_rsd process, or nothing."""
    if not _truthy(LaunchConfiguration('rust_daemon').perform(context)):
        return []
    modules = [
        module for module, switch in _DAEMON_MODULES
        if _truthy(LaunchConfiguration(switch).perform(context))
    ]
    if not modules:
        raise RuntimeError(
            'rust_daemon:=true but no rust_* switch is on: nothing to run'
        )

    bringup = get_package_share_directory('mower_bringup')
    params_file = LaunchConfiguration('rust_daemon_params_file').perform(context)
    parameters = [params_file]

    arguments = ['--modules', ','.join(modules)]
    # The two modules that are configured on the command line rather than with
    # ROS parameters; the values are the same ones rosbridge.launch.py passes.
    if 'bridge' in modules:
        arguments += ['--module-args', ' '.join([
            'bridge=--address', LaunchConfiguration('rosbridge_address').perform(context),
            '--port', '9090',
            '--loopback-port', '9091',
            '--policy', os.path.join(bringup, 'config', 'ws_bridge.yaml'),
        ])]
    if 'agent' in modules:
        arguments += ['--module-args',
                      'agent=--gate ws://127.0.0.1:9090 '
                      '--rosbridge ws://127.0.0.1:9091']
    if 'gps' in modules:
        parameters.append(
            LaunchConfiguration('gps_params_file').perform(context))
    remaps = [r for module in modules for r in _DAEMON_REMAPS.get(module, [])]
    if 'localize' in modules:
        # The file the C++ nodes read, keyed by the same node names: the one
        # source of the filter and navsat settings. After mower_rsd.yaml, so
        # it wins on any key it sets, like gps.yaml; an override goes through
        # localize_params_file, which reaches either stack.
        parameters.append(
            LaunchConfiguration('localize_params_file').perform(context))
        remaps.append('navsat_transform:gps/fix:=' + LaunchConfiguration(
            'gps_fix_topic').perform(context))
    if remaps:
        arguments.append('--ros-args')
        for rule in remaps:
            arguments += ['-r', rule]

    # No `name=`: launch_ros would emit a bare `-r __node:=...`, which renames
    # every node in the process. mower_rsd names its nodes itself.
    return [Node(
        package='mower_rs',
        executable='mower_rsd',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        arguments=arguments,
        parameters=parameters,
    )]


def generate_launch_description():
    """Build the single production entry point for the mower container."""
    mower_bringup_dir = get_package_share_directory('mower_bringup')
    mower_mission_dir = get_package_share_directory('mower_mission')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_physical_joystick = LaunchConfiguration(
        'enable_physical_joystick'
    )
    physical_joystick_enable_button = LaunchConfiguration(
        'physical_joystick_enable_button'
    )
    enable_keyboard_teleop = LaunchConfiguration('enable_keyboard_teleop')
    rosbridge_address = LaunchConfiguration('rosbridge_address')
    zone_record_dir = LaunchConfiguration('zone_record_dir')
    sites_dir = LaunchConfiguration('sites_dir')
    output_root = LaunchConfiguration('output_root')
    record = LaunchConfiguration('record')
    robot_id = LaunchConfiguration('robot_id')
    r2_env_file = LaunchConfiguration('r2_env_file')
    git_repo_dir = LaunchConfiguration('git_repo_dir')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')
    enable_gps = LaunchConfiguration('enable_gps')
    gps_params_file = LaunchConfiguration('gps_params_file')
    localize_params_file = LaunchConfiguration('localize_params_file')
    require_navigation_health = LaunchConfiguration(
        'require_navigation_health'
    )
    nav_composition = LaunchConfiguration('nav_composition')
    rust_status = LaunchConfiguration('rust_status')
    rust_adapter = LaunchConfiguration('rust_adapter')
    rust_record = LaunchConfiguration('rust_record')
    rust_nav = LaunchConfiguration('rust_nav')
    rust_battery = LaunchConfiguration('rust_battery')
    rust_pid_autotune = LaunchConfiguration('rust_pid_autotune')
    rust_map = LaunchConfiguration('rust_map')
    rust_coverage = LaunchConfiguration('rust_coverage')
    rust_agent = LaunchConfiguration('rust_agent')
    rust_guards = LaunchConfiguration('rust_guards')
    rust_imu = LaunchConfiguration('rust_imu')
    rust_localize = LaunchConfiguration('rust_localize')
    rust_bridge = LaunchConfiguration('rust_bridge')
    rust_base = LaunchConfiguration('rust_base')
    rust_daemon = LaunchConfiguration('rust_daemon')

    mower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_bringup_dir, 'launch', 'mower.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_physical_joystick': enable_physical_joystick,
            'physical_joystick_enable_button': (
                physical_joystick_enable_button
            ),
            'enable_keyboard_teleop': enable_keyboard_teleop,
            'gps_fix_topic': gps_fix_topic,
            'enable_gps': enable_gps,
            'gps_params_file': gps_params_file,
            'localize_params_file': localize_params_file,
            'rust_guards': rust_guards,
            'rust_imu': rust_imu,
            'rust_base': rust_base,
            'rust_localize': rust_localize,
            'nav_composition': nav_composition,
            'rust_daemon': rust_daemon,
        }.items(),
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mower_mission_dir, 'launch', 'mission.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rosbridge_address': rosbridge_address,
            'zone_record_dir': zone_record_dir,
            'sites_dir': sites_dir,
            'output_root': output_root,
            'record': record,
            'robot_id': robot_id,
            'r2_env_file': r2_env_file,
            'git_repo_dir': git_repo_dir,
            'gps_fix_topic': gps_fix_topic,
            'require_navigation_health': require_navigation_health,
            'rust_status': rust_status,
            'rust_adapter': rust_adapter,
            'rust_record': rust_record,
            'rust_nav': rust_nav,
            'rust_battery': rust_battery,
            'rust_pid_autotune': rust_pid_autotune,
            'rust_map': rust_map,
            'rust_coverage': rust_coverage,
            'rust_agent': rust_agent,
            'rust_bridge': rust_bridge,
            'rust_daemon': rust_daemon,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock for both bringup and mission',
        ),
        DeclareLaunchArgument(
            'enable_physical_joystick',
            default_value='false',
            description='Enable deadman-gated physical joystick teleop',
        ),
        DeclareLaunchArgument(
            'physical_joystick_enable_button',
            default_value='4',
            description='Joy button index used as the deadman switch',
        ),
        DeclareLaunchArgument(
            'enable_keyboard_teleop',
            default_value='false',
            description='Enable keyboard teleop through twist_mux',
        ),
        DeclareLaunchArgument(
            'rosbridge_address',
            default_value='127.0.0.1',
            description='rosbridge listen address for proxy or trusted LAN',
        ),
        DeclareLaunchArgument(
            'zone_record_dir',
            default_value='zone_record',
            description='Persistent active mission geometry directory',
        ),
        DeclareLaunchArgument(
            'sites_dir',
            default_value='~/.mower/sites',
            description='Persistent named site directory',
        ),
        DeclareLaunchArgument(
            'output_root',
            default_value='~/mower_bags',
            description='Persistent rosbag output directory',
        ),
        DeclareLaunchArgument(
            'record',
            default_value='false',
            description='Opt in to mission recording after retention setup',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='mower',
            description='Recorder robot identifier',
        ),
        DeclareLaunchArgument(
            'r2_env_file',
            default_value='',
            description='Gitignored recorder R2 credential file',
        ),
        DeclareLaunchArgument(
            'git_repo_dir',
            default_value='',
            description='Optional repository path recorded in bag metadata',
        ),
        DeclareLaunchArgument(
            'gps_fix_topic',
            default_value='/fix',
            description='Canonical GPS fix shared by localization and health',
        ),
        DeclareLaunchArgument(
            'enable_gps',
            default_value='false',
            description='Run the u-blox receiver driver in this container '
                        '(publishes gps_fix_topic; needs /dev/gps_rtk)',
        ),
        DeclareLaunchArgument(
            'gps_params_file',
            default_value=os.path.join(
                mower_bringup_dir, 'config', 'gps.yaml'
            ),
            description='mower_gps parameter file; point at a copy under '
                        '~/.mower to try receiver settings without a new image',
        ),
        DeclareLaunchArgument(
            'localize_params_file',
            default_value=os.path.join(
                get_package_share_directory('mower_nav2'), 'config',
                'dual_ekf_navsat_params.yaml'
            ),
            description='EKF and navsat_transform parameter file, read by the '
                        'C++ nodes and mower_localize alike; point at a copy '
                        'under ~/.mower to try other values without a new '
                        'image',
        ),
        DeclareLaunchArgument(
            'nav_composition',
            default_value='true',
            description='Run the Nav2 servers as components in one '
                        'component_container_isolated process',
        ),
        DeclareLaunchArgument(
            'require_navigation_health',
            default_value='true',
            description='Fail closed when pose or precise GPS becomes stale',
        ),
        # Staged roll-out of the mower_rs (Rust) processes, one switch per
        # process so the safety-critical guards can follow the status node.
        DeclareLaunchArgument(
            'rust_status',
            default_value='false',
            description='mower_rs robot_status instead of the rclpy '
                        'heartbeat / robot_info / telemetry nodes',
        ),
        DeclareLaunchArgument(
            'rust_adapter',
            default_value='false',
            description='mower_rs mower_adapter instead of the rclpy '
                        'flutter_adapter_node',
        ),
        DeclareLaunchArgument(
            'rust_record',
            default_value='false',
            description='mower_rs mower_record instead of the rclpy '
                        'path_record_node',
        ),
        DeclareLaunchArgument(
            'rust_nav',
            default_value='false',
            description='mower_rs mower_nav instead of the rclpy '
                        'nav_action_server',
        ),
        DeclareLaunchArgument(
            'rust_battery',
            default_value='false',
            description='mower_rs mower_battery instead of the rclpy '
                        'battery_state_node',
        ),
        DeclareLaunchArgument(
            'rust_pid_autotune',
            default_value='false',
            description='mower_rs mower_pid_autotune instead of the rclpy '
                        'pid_autotune_node',
        ),
        DeclareLaunchArgument(
            'rust_map',
            default_value='false',
            description='mower_rs mower_map instead of the rclpy '
                        'map_manage_node',
        ),
        DeclareLaunchArgument(
            'rust_coverage',
            default_value='false',
            description='mower_rs mower_coverage instead of the rclpy '
                        'coverage_node',
        ),
        DeclareLaunchArgument(
            'rust_agent',
            default_value='false',
            description='mower_rs mower_agent instead of the Python fleet agent',
        ),
        DeclareLaunchArgument(
            'rust_guards',
            default_value='false',
            description='mower_rs velocity_command_guard for both guard '
                        'instances instead of the rclpy guard',
        ),
        DeclareLaunchArgument(
            'rust_imu',
            default_value='false',
            description='mower_rs mower_imu instead of the wit_ros2_imu driver',
        ),
        DeclareLaunchArgument(
            'rust_localize',
            default_value='false',
            description='mower_rs mower_localize instead of the two '
                        'robot_localization ekf_node processes and '
                        'navsat_transform_node (same node names, topics, '
                        'transforms and services)',
        ),
        DeclareLaunchArgument(
            'rust_bridge',
            default_value='false',
            description='mower_rs mower_ws_bridge instead of rosbridge_auth_proxy '
                        '+ rosbridge_websocket + rosapi',
        ),
        DeclareLaunchArgument(
            'rust_base',
            default_value='false',
            description='mower_rs mower_base instead of the whole '
                        'ros2_control chain: ros2_control_node '
                        '(controller_manager), mower_hardware::MowerSystem, '
                        'diff_controller, joint_state_broadcaster and their '
                        'two spawners. Same serial protocol, same /odom, '
                        '/joint_states and /mower_base/* contract; '
                        'robot_state_publisher stays and keeps its '
                        '/joint_states input. Flip only after a supervised '
                        'drive.',
        ),
        DeclareLaunchArgument(
            'rust_daemon',
            default_value='false',
            description='Run every enabled mower_rs module inside one '
                        'mower_rsd process (one r2r Context / DDS '
                        'participant) instead of one binary each. The module '
                        'set is derived from the rust_* switches above plus '
                        'enable_gps; node names, topics, services and '
                        'parameters are unchanged.',
        ),
        DeclareLaunchArgument(
            'rust_daemon_params_file',
            default_value=os.path.join(
                mower_bringup_dir, 'config', 'mower_rsd.yaml'
            ),
            description='Per-node parameter sections for mower_rsd; copy into '
                        '~/.mower and point here to change them without a new '
                        'image',
        ),
        OpaqueFunction(function=_enforce_production_safety),
        mower_launch,
        mission_launch,
        OpaqueFunction(function=_rust_daemon_node),
    ])

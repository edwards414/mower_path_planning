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
    require_navigation_health = LaunchConfiguration(
        'require_navigation_health'
    )
    rust_status = LaunchConfiguration('rust_status')
    rust_guards = LaunchConfiguration('rust_guards')

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
            'rust_guards': rust_guards,
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
            'rust_guards',
            default_value='false',
            description='mower_rs velocity_command_guard for both guard '
                        'instances instead of the rclpy guard',
        ),
        OpaqueFunction(function=_enforce_production_safety),
        mower_launch,
        mission_launch,
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# `rust_daemon:=true` moves every enabled mower_rs module into one mower_rsd
# process (started by robot.launch.py), so the separate binaries must not also
# start. The Python fallbacks keep their own `UnlessCondition(rust_*)`.
_TRUE = "('true', '1', 'yes', 'on')"

# The app stamps each manual command with this robot's own clock
# (/manual_command_clock), so the age bounds the whole phone <-> relay <->
# robot round trip. The guard default (0.25 s) is never met over the
# Cloudflare relay (~0.15-0.2 s service round trip even from a wired LAN,
# more over 4G): every command was dropped as stale while the video kept
# working. Manual guard only; the final guard keeps its default. Keep in step
# with the app's _manualCommandClockTimeout (800 ms) and mower_rsd.yaml.
MANUAL_MAX_INPUT_AGE_S = 0.8


def _rust_binary(flag, rust_daemon):
    """Run the separate mower_rs binary: switch on and daemon not running."""
    return IfCondition(PythonExpression([
        "'", flag, "'.lower() in ", _TRUE,
        " and '", rust_daemon, "'.lower() not in ", _TRUE,
    ]))



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    # rust_guards:=true swaps both velocity guards for the mower_rs binary
    # (src/mower_rs/crates/velocity_command_guard): same rules, same
    # remappings and parameters, ~1 % of a core each instead of 13-18 %.
    # The Rust guard stamps with the wall clock only, so it is production
    # only (robot.launch.py already enforces use_sim_time:=false there).
    rust_guards = LaunchConfiguration('rust_guards')
    rust_daemon = LaunchConfiguration('rust_daemon')

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

    manual_guard_remappings = [
        ('cmd_vel_in', '/app_joy_cmd'),
        ('cmd_vel_out', '/joy_cmd'),
        ('command_clock', '/manual_command_clock'),
    ]
    final_guard_remappings = [
        ('cmd_vel_in', '/cmd_vel_guard_input'),
        ('cmd_vel_out', '/drivetrain_guarded_cmd_vel'),
    ]

    manual_velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='manual_velocity_guard',
        output='screen',
        condition=UnlessCondition(rust_guards),
        parameters=[{
            'use_sim_time': use_sim_time,
            'require_command_session': True,
            'max_input_age_s': MANUAL_MAX_INPUT_AGE_S,
        }],
        remappings=manual_guard_remappings,
    )

    velocity_guard = Node(
        package='mower_bringup',
        executable='velocity_command_guard',
        name='velocity_command_guard',
        output='screen',
        condition=UnlessCondition(rust_guards),
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=final_guard_remappings,
    )

    manual_velocity_guard_rs = Node(
        package='mower_rs',
        executable='velocity_command_guard',
        name='manual_velocity_guard',
        output='screen',
        condition=_rust_binary(rust_guards, rust_daemon),
        parameters=[{'require_command_session': True,
                     'max_input_age_s': MANUAL_MAX_INPUT_AGE_S}],
        remappings=manual_guard_remappings,
    )

    velocity_guard_rs = Node(
        package='mower_rs',
        executable='velocity_command_guard',
        name='velocity_command_guard',
        output='screen',
        condition=_rust_binary(rust_guards, rust_daemon),
        remappings=final_guard_remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rust_guards', default_value='false'),
        DeclareLaunchArgument('rust_daemon', default_value='false'),
        manual_velocity_guard,
        manual_velocity_guard_rs,
        twist_mux,
        velocity_guard,
        velocity_guard_rs,
    ])

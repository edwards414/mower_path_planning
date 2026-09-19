from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def _reject_sim_time_for_real_hardware(context):
    value = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    if value not in {'0', 'false', 'no', 'off'}:
        raise RuntimeError(
            'mower.launch.py is the real-hardware entry point and forbids '
            'use_sim_time:=true because actuator watchdogs require wall time; '
            'use sim_with_nav.launch.py for simulation'
        )
    return []


def generate_launch_description():
    mower_bringup_dir = get_package_share_directory('mower_bringup')
    mower_nav2_dir = get_package_share_directory('mower_nav2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_apriltag_docking = LaunchConfiguration('enable_apriltag_docking')
    nav_autostart = LaunchConfiguration('nav_autostart')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    enable_physical_joystick = LaunchConfiguration(
        'enable_physical_joystick'
    )
    physical_joystick_enable_button = LaunchConfiguration(
        'physical_joystick_enable_button'
    )
    enable_keyboard_teleop = LaunchConfiguration('enable_keyboard_teleop')
    gps_fix_topic = LaunchConfiguration('gps_fix_topic')
    enable_gps = LaunchConfiguration('enable_gps')
    gps_params_file = LaunchConfiguration('gps_params_file')

    declare_rust_guards = DeclareLaunchArgument(
        'rust_guards',
        default_value='false',
        description='Run the mower_rs velocity guards instead of the rclpy ones',
    )

    declare_rust_imu = DeclareLaunchArgument(
        'rust_imu',
        default_value='false',
        description='Run the mower_rs WIT IMU driver instead of wit_ros2_imu',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Launch dual EKF and navsat_transform',
    )
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Launch the Nav2 stack',
    )
    declare_enable_apriltag_docking = DeclareLaunchArgument(
        'enable_apriltag_docking',
        default_value='false',
        description='Launch rear-camera AprilTag docking detector',
    )
    declare_nav_autostart = DeclareLaunchArgument(
        'nav_autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes',
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            mower_nav2_dir,
            'config',
            'nav2_no_map_params.yaml',
        ),
        description='Full path to the Nav2 parameters file',
    )
    declare_enable_physical_joystick = DeclareLaunchArgument(
        'enable_physical_joystick',
        default_value='false',
        description=(
            'Launch the physical joystick driver and deadman-gated teleop'
        ),
    )
    declare_physical_joystick_enable_button = DeclareLaunchArgument(
        'physical_joystick_enable_button',
        default_value='4',
        description='Joy button index used as the physical deadman switch',
    )
    declare_enable_keyboard_teleop = DeclareLaunchArgument(
        'enable_keyboard_teleop',
        default_value='false',
        description='Launch keyboard teleop through twist_mux',
    )
    declare_gps_fix_topic = DeclareLaunchArgument(
        'gps_fix_topic',
        default_value='/fix',
        description='Canonical GPS fix topic used by navsat and health gates',
    )
    declare_enable_gps = DeclareLaunchArgument(
        'enable_gps',
        default_value='false',
        description=(
            'Run the u-blox receiver driver (mower_rs mower_gps) here and '
            'publish gps_fix_topic; needs /dev/gps_rtk'
        ),
    )
    declare_gps_params_file = DeclareLaunchArgument(
        'gps_params_file',
        default_value=os.path.join(mower_bringup_dir, 'config', 'gps.yaml'),
        description='mower_gps parameter file (device, rate_hz, frame_id, '
                    'timeouts)',
    )

    robot_description_path = os.path.join(
        get_package_share_directory('mower_description'),
        'mower_robot',
        'real_robot.xacro'
    )
    robot_description = {
        'robot_description': xacro.process_file(robot_description_path).toxml()
    }

    mower_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mower_controller'),
                'launch',
                'controller_test.launch.py'
            )
        ),
        launch_arguments={
            'publish_robot_state_publisher': 'false',
        }.items(),
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'twist_mux.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rust_guards': LaunchConfiguration('rust_guards'),
        }.items(),
    )

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_nav2_dir,
                'launch',
                'dual_ekf_navsat.launch.py',
            )
        ),
        condition=IfCondition(enable_localization),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gps_fix_topic': gps_fix_topic,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_nav2_dir,
                'launch',
                'navigation.launch.py',
            )
        ),
        condition=IfCondition(enable_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': nav_autostart,
            # This is the real-robot bringup. Never publish synthetic battery
            # telemetry here; simulation launch files keep their own default.
            'launch_battery_simulator': 'false',
            # Keep the production mux/coordinator as the only path to the
            # drivetrain even if navigation.launch.py defaults change later.
            'cmd_vel_output_topic': '/nav_cmd_vel',
        }.items(),
    )

    apriltag_docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mower_bringup_dir,
                'launch',
                'apriltag_docking.launch.py',
            )
        ),
        condition=IfCondition(enable_apriltag_docking),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    teleop_keyboard = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'mower_teleop',
            'teleop_keyboard',
            '--ros-args',
            '-r',
            '/cmd_vel:=/keyboard_cmd_vel',
        ],
        output='screen',
        condition=IfCondition(enable_keyboard_teleop),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    rust_imu = LaunchConfiguration('rust_imu')
    # Both drivers exit on a serial I/O error so that no stale-looking IMU
    # keeps the navigation health gate open. The USB hub on the LubanCat
    # re-enumerates the CH341 adapter every hour or two (18 times in a day
    # on 2026-09-19), so launch has to bring the driver back once
    # /dev/imu_usb reappears; until then the gate stays closed.
    wit_ros2_imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        output='screen',
        condition=UnlessCondition(rust_imu),
        respawn=True,
        respawn_delay=2.0,
        remappings=[('imu/data_raw', 'imu/data')],
    )
    # rust_imu:=true -- the same WIT serial driver as an r2r process
    # (src/mower_rs/crates/mower_imu), same backlog / freshness rules.
    mower_imu_node = Node(
        package='mower_rs',
        executable='mower_imu',
        name='imu',
        output='screen',
        condition=IfCondition(rust_imu),
        respawn=True,
        respawn_delay=2.0,
        remappings=[('imu/data_raw', 'imu/data')],
    )

    # u-blox ZED-F9P over USB, the only publisher of the canonical fix
    # topic, in this container like everything else (it used to be a
    # separate `gps` compose service with its own image; nothing isolated
    # it, the DDS graph is shared through network_mode/ipc host anyway).
    # mower_rs/mower_gps rather than the ROS ublox_gps node: that one never
    # exits on a dead port (it reposts the failed read forever and keeps the
    # hung-up tty open, so the re-enumerated receiver comes back on a minor
    # the container's static /dev/gps_rtk no longer points at). mower_gps
    # ends on any serial failure or when NAV-PVT stops, launch respawns it
    # like the IMU drivers, and the navigation health gate closes on the
    # stale fix in between.
    mower_gps_node = Node(
        package='mower_rs',
        executable='mower_gps',
        name='gps',
        output='screen',
        condition=IfCondition(enable_gps),
        respawn=True,
        respawn_delay=2.0,
        parameters=[gps_params_file, {'fix_topic': gps_fix_topic}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(enable_physical_joystick),
    )

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            'publish_stamped_twist': True,
            'require_enable_button': True,
            'enable_button': ParameterValue(
                physical_joystick_enable_button,
                value_type=int,
            ),
            'enable_turbo_button': -1,
            'axis_linear.x': 1,
            'axis_angular.yaw': 3,
            'scale_linear.x': 0.5,
            'scale_linear_turbo.x': 0.5,
            'scale_angular.yaw': 0.8,
            'scale_angular_turbo.yaw': 0.8,
        }],
        remappings=[('/cmd_vel', '/physical_joy_cmd')],
        condition=IfCondition(enable_physical_joystick),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rust_guards,
        declare_rust_imu,
        OpaqueFunction(function=_reject_sim_time_for_real_hardware),
        declare_enable_localization,
        declare_enable_navigation,
        declare_enable_apriltag_docking,
        declare_nav_autostart,
        declare_nav2_params_file,
        declare_enable_physical_joystick,
        declare_physical_joystick_enable_button,
        declare_enable_keyboard_teleop,
        declare_gps_fix_topic,
        declare_enable_gps,
        declare_gps_params_file,
        robot_state_publisher,
        wit_ros2_imu_node,
        mower_imu_node,
        mower_gps_node,
        mower_controller_launch,
        twist_mux_launch,
        robot_localization_launch,
        navigation_launch,
        apriltag_docking_launch,
        joy_node,
        teleop_joy,
        teleop_keyboard,
    ])

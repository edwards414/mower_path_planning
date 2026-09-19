"""Regression checks for real/sim drivetrain geometry and frame alignment."""

import ast
import re
from pathlib import Path

import pytest


SRC_DIR = Path(__file__).resolve().parents[2]
REAL_CONTROLLER = (
    SRC_DIR / 'mower_controller/controllers/diff_drive_controller.yaml'
)
DESCRIPTION_CONTROLLER = (
    SRC_DIR / 'mower_description/config/controllers.yaml'
)
MISSION_LAUNCH = SRC_DIR / 'mower_mission/launch/mission.launch.py'
MOWER_LAUNCH = SRC_DIR / 'mower_bringup/launch/mower.launch.py'
TWIST_MUX_CONFIG = SRC_DIR / 'mower_bringup/config/twist_mux_topics.yaml'
REAL_CONTROLLER_LAUNCH = (
    SRC_DIR / 'mower_controller/launch/controller_test.launch.py'
)
LOCAL_COMPOSE = SRC_DIR.parent / 'docker-compose.yaml'
DEPLOY_COMPOSE = SRC_DIR.parent / 'deploy/docker-compose.yaml'
ROBOT_LAUNCH = SRC_DIR / 'mower_bringup/launch/robot.launch.py'
DOCKERFILE = SRC_DIR.parent / 'Dockerfile'
NAV2_CONFIG = SRC_DIR / 'mower_nav2/config/nav2_no_map_params.yaml'
NAV2_LAUNCH = SRC_DIR / 'mower_nav2/launch/navigation.launch.py'
MAKEFILE = SRC_DIR.parent / 'Makefile'
MOWER_SYSTEM = SRC_DIR / 'mower_controller/src/mower_system.cpp'
STM_COMMS = SRC_DIR / 'mower_controller/src/Stm_Comms.cpp'
IMU_DRIVER = SRC_DIR / 'wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py'
IMU_DRIVER_RS = SRC_DIR / 'mower_rs/crates/mower_imu/src/main.rs'
LOCAL_MEDIAMTX = SRC_DIR.parent / 'mediamtx.yml'
DEPLOY_MEDIAMTX = SRC_DIR.parent / 'deploy/mediamtx.yml'
DUAL_EKF_LAUNCH = SRC_DIR / 'mower_nav2/launch/dual_ekf_navsat.launch.py'
SIM_LAUNCH = SRC_DIR / 'mower_bringup/launch/sim_with_nav.launch.py'
SMALL_TEST_LAUNCH = SRC_DIR / 'mower_bringup/launch/small_test.launch.py'
TELEOP_SETUP = SRC_DIR / 'mower_teleop/setup.py'
KEYBOARD_TELEOP = SRC_DIR / 'mower_teleop/mower_teleop/teleop_keyboard.py'
TWIST_MUX_LAUNCH = SRC_DIR / 'mower_bringup/launch/twist_mux.launch.py'
SYSTEM_TEST_LAUNCH = SRC_DIR / 'mower_bringup/launch/system_test.launch.py'
VELOCITY_GUARD_RS = (
    SRC_DIR / 'mower_rs/crates/velocity_command_guard/src/core.rs'
)
VELOCITY_GUARD_RS_MAIN = (
    SRC_DIR / 'mower_rs/crates/velocity_command_guard/src/main.rs'
)
NAV_SERVER_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_nav/src/main.rs'
NAV_SERVER_RS_STATE = SRC_DIR / 'mower_rs/crates/mower_nav/src/state.rs'
NAV_SERVER_RS_GEOMETRY = (
    SRC_DIR / 'mower_rs/crates/mower_nav/src/geometry.rs'
)
VELOCITY_GUARD = (
    SRC_DIR / 'mower_bringup/mower_bringup/velocity_command_guard.py'
)
AUTO_COVERAGE = (
    SRC_DIR / 'mower_mission/mower_mission/auto_coverage_node.py'
)


def _scalar(path: Path, key: str) -> str:
    match = re.search(
        rf'^\s*{re.escape(key)}:\s*([^\s#]+)',
        path.read_text(encoding='utf-8'),
        flags=re.MULTILINE,
    )
    assert match is not None, f'{key} missing from {path}'
    return match.group(1)


def test_real_controller_matches_description_wheel_radius():
    real_radius = float(_scalar(REAL_CONTROLLER, 'wheel_radius'))
    description_radius = float(
        _scalar(DESCRIPTION_CONTROLLER, 'wheel_radius')
    )

    # The wheel mesh is 0.18144 m in diameter, i.e. a nominal 0.09 m radius.
    assert real_radius == pytest.approx(0.09)
    assert real_radius == pytest.approx(description_radius)


def test_real_controller_uses_nav2_robot_base_frame():
    assert _scalar(REAL_CONTROLLER, 'base_frame_id') == 'base_footprint'
    assert (
        _scalar(REAL_CONTROLLER, 'base_frame_id')
        == _scalar(DESCRIPTION_CONTROLLER, 'base_frame_id')
    )


def test_map_manager_keeps_public_parameter_service_name():
    """Launch override must match adapter/Qt's /map_manage parameter clients."""
    launch_source = MISSION_LAUNCH.read_text(encoding='utf-8')
    map_node = re.search(
        r"map_manage_node\s*=\s*Node\((.*?)\n\s*\)",
        launch_source,
        flags=re.DOTALL,
    )
    assert map_node is not None
    assert re.search(r"\bname\s*=\s*['\"]map_manage['\"]", map_node.group(1))


def test_physical_joystick_is_opt_in_deadman_gated_and_isolated():
    """Neutral joystick traffic must never mask autonomous navigation."""
    launch_source = MOWER_LAUNCH.read_text(encoding='utf-8')
    mux_source = TWIST_MUX_CONFIG.read_text(encoding='utf-8')

    argument = re.search(
        r"DeclareLaunchArgument\(\s*['\"]enable_physical_joystick['\"]"
        r"(.*?)\n\s*\)",
        launch_source,
        flags=re.DOTALL,
    )
    assert argument is not None
    assert re.search(r"default_value\s*=\s*['\"]false['\"]", argument.group(1))
    assert "'require_enable_button': True" in launch_source
    assert "('/cmd_vel', '/physical_joy_cmd')" in launch_source
    assert "'scale_linear.x': 0.5" in launch_source
    assert "'scale_linear_turbo.x': 0.5" in launch_source
    assert "'scale_angular.yaw': 0.8" in launch_source
    assert "'scale_angular_turbo.yaw': 0.8" in launch_source
    assert 'topic   : /physical_joy_cmd' in mux_source
    assert 'topic   : /joy_cmd' in mux_source


def test_keyboard_teleop_is_opt_in_and_cannot_bypass_mux():
    source = MOWER_LAUNCH.read_text(encoding='utf-8')
    argument = re.search(
        r"DeclareLaunchArgument\(\s*['\"]enable_keyboard_teleop['\"]"
        r"(.*?)\n\s*\)",
        source,
        flags=re.DOTALL,
    )
    assert argument is not None
    assert re.search(r"default_value\s*=\s*['\"]false['\"]", argument.group(1))
    assert "'/cmd_vel:=/keyboard_cmd_vel'" in source
    tree = ast.parse(source)
    launch_items = next(
        node.args[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'LaunchDescription'
        and node.args
    )
    assert isinstance(launch_items, ast.List)
    assert any(
        isinstance(item, ast.Name) and item.id == 'teleop_keyboard'
        for item in launch_items.elts
    )
    make_source = MAKEFILE.read_text(encoding='utf-8')
    assert (
        'ros2 run mower_teleop teleop_keyboard --ros-args '
        '-r /cmd_vel:=/keyboard_cmd_vel'
    ) in make_source


def test_real_controller_process_bridges_command_and_odometry_topics():
    """The controller manager process owns the loaded controller remappings."""
    source = REAL_CONTROLLER_LAUNCH.read_text(encoding='utf-8')
    assert "'/drivetrain_guarded_cmd_vel'" in source
    assert "('/diff_controller/cmd_vel', '/cmd_vel')" not in source
    assert "('/diff_controller/odom', '/odom')" in source


def test_controller_and_hardware_fail_closed_on_stale_or_invalid_commands():
    assert float(_scalar(REAL_CONTROLLER, 'cmd_vel_timeout')) <= 0.25
    assert float(_scalar(DESCRIPTION_CONTROLLER, 'cmd_vel_timeout')) <= 0.25
    source = MOWER_SYSTEM.read_text(encoding='utf-8')
    assert 'std::isfinite(wheel_left_.cmd)' in source
    assert 'std::isfinite(wheel_right_.cmd)' in source
    assert 'std::isfinite(mower_blade_cmd_)' in source
    assert 'stm_comms_.setMotorValues(0, 0);' in source
    assert 'motor_status_is_fresh_and_acknowledged' in source
    assert 'return hardware_interface::return_type::ERROR;' in source


@pytest.mark.parametrize(
    'config', (REAL_CONTROLLER, DESCRIPTION_CONTROLLER)
)
def test_controller_safety_zero_crosses_rate_limiter_within_one_cycle(config):
    """Keep the 50 Hz controller from stretching a guard stop by ~0.5 s."""
    update_period_s = 1.0 / 50.0
    for axis in ('linear.x', 'angular.z'):
        max_velocity = float(_scalar(config, f'{axis}.max_velocity'))
        min_velocity = float(_scalar(config, f'{axis}.min_velocity'))
        max_deceleration = float(
            _scalar(config, f'{axis}.max_deceleration')
        )
        max_deceleration_reverse = float(
            _scalar(config, f'{axis}.max_deceleration_reverse')
        )
        assert max_deceleration < 0.0
        assert max_deceleration_reverse > 0.0
        assert abs(max_deceleration) * update_period_s >= max_velocity
        assert max_deceleration_reverse * update_period_s >= abs(min_velocity)

    source = config.read_text(encoding='utf-8')
    # Jazzy activates limits from finite values; do not restore deprecated
    # switches or zero-valued jerk limits that change limiter behaviour.
    assert '.has_velocity_limits:' not in source
    assert '.has_acceleration_limits:' not in source
    assert '.has_jerk_limits:' not in source
    assert '.min_acceleration:' not in source
    assert '.max_jerk:' not in source
    assert '.min_jerk:' not in source


def test_final_velocity_guard_owns_the_only_mux_to_controller_boundary():
    guard = VELOCITY_GUARD.read_text(encoding='utf-8')
    mux_launch = TWIST_MUX_LAUNCH.read_text(encoding='utf-8')
    system_test = SYSTEM_TEST_LAUNCH.read_text(encoding='utf-8')
    for launch in (mux_launch, system_test):
        assert "('/cmd_vel_out', '/cmd_vel_guard_input')" in launch
        assert "('cmd_vel_in', '/cmd_vel_guard_input')" in launch
        assert "('cmd_vel_in', '/app_joy_cmd')" in launch
        assert "('cmd_vel_out', '/joy_cmd')" in launch
        assert "'require_command_session': True" in launch
        assert "('command_clock', '/manual_command_clock')" in launch
    assert "('cmd_vel_out', '/drivetrain_guarded_cmd_vel')" in mux_launch
    assert "('cmd_vel_out', '/cmd_vel')" in system_test
    assert "declare_parameter('command_timeout_s', 0.20" in guard
    assert 'ClockType.STEADY_TIME' in guard
    assert 'math.isfinite' in guard
    assert 'velocity timestamp is too far in the future' in guard
    assert 'velocity timestamp is stale' in guard
    assert 'manual velocity command session is missing or stale' in guard
    assert 'self._publish_zero()' in guard
    assert 'output.header.stamp = self.get_clock().now().to_msg()' in guard
    assert 'stop.header.stamp = self.get_clock().now().to_msg()' in guard
    assert (
        _scalar(NAV2_CONFIG, 'stamp_smoothed_velocity_with_smoothing_time')
        == 'false'
    )


def test_rust_velocity_guard_keeps_the_same_rules_and_wiring():
    """rust_guards:=true swaps in mower_rs velocity_command_guard: same
    remappings, session requirement, limits, stop barriers and clocks."""
    core = VELOCITY_GUARD_RS.read_text(encoding='utf-8')
    main = VELOCITY_GUARD_RS_MAIN.read_text(encoding='utf-8')
    mux_launch = TWIST_MUX_LAUNCH.read_text(encoding='utf-8')
    assert "package='mower_rs'" in mux_launch
    assert "condition=IfCondition(rust_guards)" in mux_launch
    assert "condition=UnlessCondition(rust_guards)" in mux_launch
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')
    deploy_compose = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    for switch in ('rust_status', 'rust_adapter', 'rust_record', 'rust_nav', 'rust_guards', 'rust_imu', 'rust_bridge'):
        assert f"'{switch}': {switch}," in robot_launch
        assert f"{switch}:=${{{switch.upper()}:-false}}" in deploy_compose
    assert "parameters=[{'require_command_session': True}]" in mux_launch
    assert 'command_timeout_s", 0.20' in main
    assert 'max_input_age_s", 0.25' in main
    assert 'max_future_skew_s", 0.05' in main
    assert 'max_linear_x_m_s", 0.50' in main
    assert 'max_angular_z_rad_s", 1.00' in main
    for rule in (
        'velocity timestamp is too far in the future',
        'velocity source timestamp moved backward',
        'velocity timestamp is stale',
        'manual velocity command session is missing or stale',
        'manual velocity requires the robot command clock',
        'velocity contains NaN or infinity',
        'unsupported lateral or non-yaw velocity component',
        'linear velocity exceeds robot safety limit',
        'angular velocity exceeds robot safety limit',
        'replayed velocity cannot resume stopped motion',
    ):
        assert rule in core, rule
    assert 'is_finite()' in core
    # receipt timeouts on a steady clock, stamps from the robot's own clock
    assert 'Instant' in core
    assert 'msg.header.stamp = stamp_now()' in main
    assert 'publisher.publish(&twist(0.0, 0.0))' in main
    assert 'make_parameter_handler' not in main  # limits are immutable


def test_rust_nav_server_keeps_the_same_safety_rules_and_wiring():
    """rust_nav:=true swaps in mower_rs mower_nav for nav_action_server: the
    same node name, actions, services, admission/health rules, immutable
    safety defaults, bounded Nav2 waits and the 20 Hz fail-safe heartbeat."""
    main = NAV_SERVER_RS_MAIN.read_text(encoding='utf-8')
    state = NAV_SERVER_RS_STATE.read_text(encoding='utf-8')
    geometry = NAV_SERVER_RS_GEOMETRY.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_nav'" in mission_launch
    assert "condition=IfCondition(rust_nav)" in mission_launch
    assert "condition=UnlessCondition(rust_nav)" in mission_launch
    assert mission_launch.count("name='nav_action_server'") == 1
    assert 'r2r::Node::create(rctx, "nav_action_server", "")' in main
    for name in ('nav_action', 'nav_action_follow_path'):
        assert f'"{name}"' in main, name
    for service in (
        '/cancel_nav2', '/cencel_nav2', '/check_nav_status',
        '/confirm_navigation_dispatch', '/cancel_navigation_dispatch',
        '/mission_operation_lock',
    ):
        assert f'"{service}"' in main, service
    for topic in (
        '/navigation_coordinator_lock', '/navigation_safety_stop',
        '/nav_operation_active', '/joy_cmd', '/physical_joy_cmd',
        '/keyboard_cmd_vel', '/adapter/robot_pose', '/rosout',
    ):
        assert f'"{topic}"' in main, topic
    # immutable production safety defaults (no parameter service at all)
    assert 'make_parameter_handler' not in main
    assert '"require_navigation_health", true' in main
    assert '"navigation_health_timeout_s", 0.30' in main
    assert '"max_gps_horizontal_sigma_m", 0.015' in main
    assert '"manual_command_hold_s", 0.75' in main
    assert '"gps_fix_topic", "/fix"' in main
    assert '"imu_topic", "/imu/data"' in main
    # bounded Nav2 waits and generation-correlated terminal evidence
    for name in (
        'nav2_action_server_timeout_s', 'nav2_goal_response_timeout_s',
        'nav2_cancel_timeout_s', 'dispatch_confirmation_timeout_s',
    ):
        assert f'"{name}"' in main, name
    assert 'mark_nav2_dispatch_terminal(terminal_generation)' in main
    assert 'mark_nav2_task_uncertain(' in main
    assert 'Duration::from_millis(50)' in main  # 20 Hz heartbeat
    # the admission order and every block reason of the rclpy server
    for rule in (
        'previous Nav2 task termination is unconfirmed',
        'another navigation goal is active',
        'mission mutation is active: ',
        'manual velocity command is active',
        'robot pose/TF is unavailable or stale',
        'GPS fix is unavailable',
        'GPS fix is stale',
        'GPS odometry from navsat_transform is unavailable or stale',
        'IMU is unavailable or stale',
        'source timestamp is in the future',
        'source timestamp is stale',
        'robot pose frame must be map',
        'GPS has no valid fix',
        'GPS covariance is unknown',
        'horizontal covariance is degenerate',
        'IMU frame_id must be imu_link',
        'GPS odometry frame_id must be map',
    ):
        assert rule in state, rule
    assert '(-90.0..=90.0).contains(&lat)' in state
    assert '(-180.0..=180.0).contains(&lon)' in state
    assert 'Navigation/manual motion is active or termination is uncertain' in main
    assert 'navigation path requires at least two poses' in geometry
    assert 'canonical_dispatch_id' in geometry


def test_local_compose_builds_full_runtime_stage():
    source = LOCAL_COMPOSE.read_text(encoding='utf-8')
    lawan_service = source.split('  mediamtx:', maxsplit=1)[0]
    assert re.search(r'^\s+target:\s*runtime\s*$', lawan_service, re.MULTILINE)


def test_compose_persists_field_data_and_passes_absolute_paths():
    """Image replacement must not erase sites, working geometry, or bags."""
    for compose in (LOCAL_COMPOSE, DEPLOY_COMPOSE):
        source = compose.read_text(encoding='utf-8')
        # a named volume (local) or a host bind mount (deploy: ~/.mower, which
        # also carries host.request / firmware_sync.json) must back ~/.mower
        assert re.search(r'^\s+- \S+:/home/mower/\.mower\s*$', source, re.M), compose
        assert 'zone_record_dir:=/home/mower/.mower/zone_record' in source
        assert 'sites_dir:=/home/mower/.mower/sites' in source
        assert 'output_root:=/home/mower/.mower/bags' in source
        assert 'restart: unless-stopped' in source
        assert 'stop_grace_period: 60s' in source
        assert 'record:=false' in source


def test_production_rosbridge_binds_only_wireguard_address():
    source = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    assert 'rosbridge_address:=${ROSBRIDGE_ADDRESS:-10.77.0.2}' in source
    assert 'rosbridge_address:=0.0.0.0' not in source


def test_robot_launch_forwards_all_persistent_directory_arguments():
    source = ROBOT_LAUNCH.read_text(encoding='utf-8')
    for argument in (
        'zone_record_dir',
        'sites_dir',
        'output_root',
        'record',
        'robot_id',
        'r2_env_file',
        'git_repo_dir',
    ):
        assert f"'{argument}': {argument}" in source


def test_unarbitrated_docking_is_not_in_production_launches():
    mission = MISSION_LAUNCH.read_text(encoding='utf-8')
    nav2 = NAV2_LAUNCH.read_text(encoding='utf-8')
    assert 'docking_manager_node' not in mission
    assert 'opennav_docking' not in nav2
    assert "'docking_server'" not in nav2


def test_real_gps_topic_is_shared_and_simulation_uses_gazebo_topic():
    dual = DUAL_EKF_LAUNCH.read_text(encoding='utf-8')
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mission = MISSION_LAUNCH.read_text(encoding='utf-8')
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    assert "('gps/fix', gps_fix_topic)" in dual
    assert "'gps_fix_topic': gps_fix_topic" in robot
    assert "'gps_fix_topic': gps_fix_topic" in mower
    assert "'gps_fix_topic': gps_fix_topic" in mission
    for launch in (SIM_LAUNCH, SMALL_TEST_LAUNCH):
        assert "'gps_fix_topic': '/gps/fix'" in launch.read_text(
            encoding='utf-8'
        )


def test_legacy_small_test_is_fail_fast_and_system_test_owns_sim_graph():
    small_test = SMALL_TEST_LAUNCH.read_text(encoding='utf-8')
    system_test = SYSTEM_TEST_LAUNCH.read_text(encoding='utf-8')
    assert '_reject_deprecated_launch' in small_test
    assert 'system_test.launch.py launch_sim:=true' in small_test
    assert "LaunchConfiguration('launch_sim').perform(context)" in system_test
    assert 'requires launch_sim:=true' in system_test
    assert "executable='auto_coverage_node'" in system_test
    assert "'ros2', 'service', 'call'" not in system_test


def test_real_launch_forbids_sim_clock_and_keyboard_has_deadman_timeout():
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    keyboard = KEYBOARD_TELEOP.read_text(encoding='utf-8')
    assert '_reject_sim_time_for_real_hardware' in mower
    assert "declare_parameter('linear_input_timeout', 0.35)" in keyboard
    assert "declare_parameter('max_linear_speed', 0.5)" in keyboard
    assert "declare_parameter('max_angular_speed', 1.0)" in keyboard
    assert 'now - self.last_linear_input > self.linear_input_timeout' in keyboard
    deadman = keyboard.split(
        'def apply_deadman_and_ramp', maxsplit=1
    )[1].split('def run', maxsplit=1)[0]
    assert 'self.target_linear = 0.0' in deadman
    assert 'self.target_angular = 0.0' in deadman
    assert 'self.current_linear = 0.0' in deadman
    assert 'self.current_angular = 0.0' in deadman
    assert 'return was_active' in deadman
    assert 'return self.output_active or was_active' in deadman
    assert 'if self.apply_deadman_and_ramp(now, dt):' in keyboard
    assert "TwistStamped,\n            '/keyboard_cmd_vel'" in keyboard
    assert "create_publisher(TwistStamped, '/cmd_vel'" not in keyboard


def test_auto_coverage_failure_is_observable_to_launch_supervision():
    source = AUTO_COVERAGE.read_text(encoding='utf-8')
    system_test = SYSTEM_TEST_LAUNCH.read_text(encoding='utf-8')
    assert 'def run_sequence(self) -> bool:' in source
    assert 'startup sequence failed; exiting nonzero' in source
    assert 'raise SystemExit(exit_code)' in source
    assert 'OnProcessExit(' in system_test
    assert 'target_action=auto_coverage' in system_test
    assert "reason='auto_coverage exited before system shutdown'" in system_test


def test_blade_teleop_is_not_executable_without_controller_or_mcu_watchdog():
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    setup = TELEOP_SETUP.read_text(encoding='utf-8')
    controller_launch = REAL_CONTROLLER_LAUNCH.read_text(encoding='utf-8')
    controller_config = REAL_CONTROLLER.read_text(encoding='utf-8')
    hardware = MOWER_SYSTEM.read_text(encoding='utf-8')
    assert 'blade_teleop_joy = Node(' not in mower
    assert 'blade_teleop_joy =' not in setup
    assert "arguments=['mower_blade_controller']" not in controller_launch
    assert 'mower_blade_controller:' not in controller_config
    assert 'const int blade_permille = 0;' in hardware
    assert 'stm_comms_.setMowerBladeValue(blade_permille);' not in hardware
    assert hardware.count('stm_comms_.setMowerBladeValue(0);') >= 2


def test_stm_status_ack_is_correlated_and_faults_are_latched():
    comms = STM_COMMS.read_text(encoding='utf-8')
    hardware = MOWER_SYSTEM.read_text(encoding='utf-8')
    assert 'MotorStatus candidate{};' in comms
    assert 'candidate.commanded_left_permille != command->left_permille' in comms
    assert 'last_ack_progress_at_ = now;' in comms
    assert 'motor_status_.command_age_ms > command->timeout_ms' in comms
    assert 'expected_sequence.has_value()' in comms
    assert 'hardware_fault_latched_ = true;' in hardware
    assert 'return hardware_interface::return_type::ERROR;' in hardware


def test_imu_driver_discards_backlog_and_requires_fresh_complete_samples():
    source = IMU_DRIVER.read_text(encoding='utf-8')
    assert 'self._stop_event.wait(0.005)' in source
    assert 'for raw_byte in buff_data:' in source
    assert 'wt_imu.reset_input_buffer()' in source
    assert 'if buff_count > 88:' in source
    assert 'components_are_fresh = all(' in source
    assert "f'IMU serial read/parser failed: {exc}'" in source
    # the mower_rs driver (rust_imu:=true) keeps the same fail-closed rules
    rust = IMU_DRIVER_RS.read_text(encoding='utf-8')
    assert 'const MAX_BACKLOG_BYTES: u32 = 88;' in rust
    assert 'Duration::from_millis(200)' in rust
    assert 'Discarded oversized IMU serial backlog' in rust
    assert 'Discarded IMU serial backlog after a host timing gap' in rust
    assert 'acceleration/gyro components are stale' in rust
    assert 'std::process::exit(exit_code)' in rust


def test_mower_camera_exposes_only_required_protocols():
    for config in (LOCAL_MEDIAMTX, DEPLOY_MEDIAMTX):
        source = config.read_text(encoding='utf-8')
        for protocol in ('rtmp', 'hls', 'srt', 'moq'):
            assert re.search(
                rf'^\s*{protocol}:\s*false\s*$',
                source,
                flags=re.MULTILINE,
            )
    # The robot config (docs/BACKEND_ARCHITECTURE.md §8): WHEP for the app,
    # the control API on loopback only (mower_agent writes TURN credentials
    # there), publishing and the API restricted to loopback callers.
    production = DEPLOY_MEDIAMTX.read_text(encoding='utf-8')
    assert re.search(r'^webrtcAddress: :8889$', production, flags=re.MULTILINE)
    assert re.search(r'^api: yes$', production, flags=re.MULTILINE)
    assert re.search(r'^apiAddress: 127\.0\.0\.1:9997$', production, flags=re.MULTILINE)
    assert re.search(r'^webrtcICEServers2: \[\]$', production, flags=re.MULTILINE)
    assert 'ips: ["127.0.0.1", "::1"]' in production
    assert '- action: api' in production
    assert 'whip' not in production.lower()


def test_real_robot_health_gate_is_enabled_by_default():
    for path in (ROBOT_LAUNCH, MISSION_LAUNCH):
        source = path.read_text(encoding='utf-8')
        argument = re.search(
            r"DeclareLaunchArgument\(\s*['\"]require_navigation_health['\"]"
            r"(.*?)\n\s*\)",
            source,
            flags=re.DOTALL,
        )
        assert argument is not None
        assert re.search(
            r"default_value\s*=\s*['\"]true['\"]",
            argument.group(1),
        )


def test_every_nav2_motion_source_is_forced_through_the_mux():
    source = NAV2_LAUNCH.read_text(encoding='utf-8')
    assert source.count("('cmd_vel', '/cmd_vel_nav')") >= 2
    mux = TWIST_MUX_CONFIG.read_text(encoding='utf-8')
    assert 'topic   : /navigation_safety_stop' in mux
    assert 'topic   : /navigation_coordinator_lock' in mux
    assert re.search(
        r'navigation_safety_stop:.*?priority:\s*50',
        mux,
        flags=re.DOTALL,
    )
    assert re.search(
        r'navigation_coordinator:.*?timeout\s*:\s*0\.3.*?priority:\s*50',
        mux,
        flags=re.DOTALL,
    )


def test_nav2_clock_is_controlled_only_by_the_launch_argument():
    """Production nodes must not stay frozen on an absent simulation clock."""
    source = NAV2_CONFIG.read_text(encoding='utf-8')
    assert not re.search(
        r'^\s*use_sim_time:\s*(?:True|true)\s*$',
        source,
        flags=re.MULTILINE,
    )


def test_runtime_container_sets_non_root_home():
    source = DOCKERFILE.read_text(encoding='utf-8')
    runtime = source.split('FROM ros:${ROS_DISTRO}-ros-core AS runtime', 1)[1]
    runtime = runtime.split('FROM ros:${ROS_DISTRO}-ros-base AS gps_runtime', 1)[0]
    assert 'ENV HOME=/home/${USER_NAME}' in runtime


def test_nav2_local_and_global_costmaps_use_full_robot_radius():
    source = NAV2_CONFIG.read_text(encoding='utf-8')
    radii = re.findall(
        r'^\s*robot_radius:\s*([^\s#]+)',
        source,
        flags=re.MULTILINE,
    )
    assert len(radii) == 2
    assert [float(value) for value in radii] == [0.5, 0.5]
    map_topics = re.findall(
        r'^\s*map_topic:\s*["\']([^"\']+)["\']',
        source,
        flags=re.MULTILINE,
    )
    assert map_topics == ['/map_grid', '/map_grid_global']


def test_navfn_never_substitutes_a_nearby_coverage_goal():
    source = NAV2_CONFIG.read_text(encoding='utf-8')
    planner = source.split('GridBased:', maxsplit=1)[1].split(
        'smoother_server:', maxsplit=1
    )[0]
    assert re.search(r'^\s*tolerance:\s*0\.0\s*$', planner, re.MULTILINE)

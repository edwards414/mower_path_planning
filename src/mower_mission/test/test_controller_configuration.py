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
BUILD_WORKFLOW = SRC_DIR.parent / '.github/workflows/build.yml'
GPS_PARAMS = SRC_DIR / 'mower_bringup/config/gps.yaml'
BRINGUP_PACKAGE = SRC_DIR / 'mower_bringup/package.xml'
MOWER_RS_CMAKE = SRC_DIR / 'mower_rs/CMakeLists.txt'
MOWER_RS_CARGO = SRC_DIR / 'mower_rs/Cargo.toml'
GPS_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_gps/src/main.rs'
UBX_RS = SRC_DIR / 'mower_rs/crates/mower_ubx/src/lib.rs'
NAV2_CONFIG = SRC_DIR / 'mower_nav2/config/nav2_no_map_params.yaml'
NAV2_LAUNCH = SRC_DIR / 'mower_nav2/launch/navigation.launch.py'
MAKEFILE = SRC_DIR.parent / 'Makefile'
MOWER_SYSTEM = SRC_DIR / 'mower_controller/src/mower_system.cpp'
STM_COMMS = SRC_DIR / 'mower_controller/src/Stm_Comms.cpp'
IMU_DRIVER = SRC_DIR / 'wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py'
IMU_DRIVER_RS = SRC_DIR / 'mower_rs/crates/mower_imu/src/main.rs'
PID_AUTOTUNE_NODE = SRC_DIR / 'mower_mission/mower_mission/pid_autotune_node.py'
PID_TUNING = SRC_DIR / 'mower_mission/mower_mission/pid_tuning.py'
PID_AUTOTUNE_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_pid_autotune/src/main.rs'
PID_AUTOTUNE_RS_TUNING = SRC_DIR / 'mower_rs/crates/mower_pid_autotune/src/tuning.rs'
MAP_MANAGE_NODE = SRC_DIR / 'mower_mission/mower_mission/map_manage_node.py'
MAP_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_map/src/main.rs'
MAP_RS_GRID = SRC_DIR / 'mower_rs/crates/mower_map/src/grid.rs'
COVERAGE_NODE = SRC_DIR / 'mower_mission/mower_mission/coverage_node.py'
COVERAGE_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_coverage/src/main.rs'
AGENT_PY = SRC_DIR / 'mower_mission/mower_mission/mower_agent.py'
AGENT_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_agent/src/main.rs'
AGENT_RS_RELAY = SRC_DIR / 'mower_rs/crates/mower_agent/src/relay.rs'
ROSBRIDGE_LAUNCH = SRC_DIR / 'mower_bringup/launch/rosbridge.launch.py'
VERSION_PY = SRC_DIR / 'mower_mission/mower_mission/version.py'
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
BATTERY_RS_MAIN = SRC_DIR / 'mower_rs/crates/mower_battery/src/main.rs'
BATTERY_RS_ESTIMATOR = (
    SRC_DIR / 'mower_rs/crates/mower_battery/src/estimator.rs'
)
BATTERY_ESTIMATOR = (
    SRC_DIR / 'mower_mission/mower_mission/battery_estimator.py'
)
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
    """Keep the controller cycle from stretching a guard stop by ~0.5 s."""
    update_period_s = 1.0 / float(_scalar(config, 'update_rate'))
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
    for switch in ('rust_status', 'rust_adapter', 'rust_record', 'rust_nav', 'rust_battery', 'rust_pid_autotune', 'rust_map', 'rust_coverage', 'rust_agent', 'rust_guards', 'rust_imu', 'rust_bridge'):
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


def test_rust_pid_autotune_keeps_the_same_maths_and_wiring():
    """rust_pid_autotune:=true swaps in mower_rs mower_pid_autotune for
    pid_autotune_node: same node name, service, parameters, plausibility
    limits, flag bits and status states."""
    main = PID_AUTOTUNE_RS_MAIN.read_text(encoding='utf-8')
    tuning = PID_AUTOTUNE_RS_TUNING.read_text(encoding='utf-8')
    py_node = PID_AUTOTUNE_NODE.read_text(encoding='utf-8')
    py_tuning = PID_TUNING.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_pid_autotune'" in mission_launch
    assert "condition=IfCondition(rust_pid_autotune)" in mission_launch
    assert "condition=UnlessCondition(rust_pid_autotune)" in mission_launch
    assert mission_launch.count("name='pid_autotune'") == 2
    assert 'r2r::Node::create(ctx_r2r, "pid_autotune", "")' in main
    assert 'create_service::<PidAutotune::Service>("/pid_autotune"' in main
    # every declared parameter with the same default
    for name, default in re.findall(r"^\s+p\('(\w+)', ([^)]+)\)", py_node, re.M):
        rust_default = default.replace("'", '"').replace('[', '&[').replace('300.0', '300.0')
        assert f'"{name}", {rust_default}' in main, (name, default)
    # plausibility limits and flag bits are the Python ones
    for const in ('GAIN_RANGE', 'TAU_RANGE', 'DELAY_MAX', 'MIN_RESPONSE_RPM', 'MAX_FIT_RMSE_FRACTION', 'TAU_DESIGN_MIN'):
        py_value = re.search(rf'^{const} = ([^#\n]+)', py_tuning, re.M).group(1).strip()
        assert re.search(rf'pub const {const}: [^=]+= {re.escape(py_value)};', tuning), const
    for flag in ('PID_FLAG_CLOSED_LOOP = 0x01', 'PID_FLAG_FLASH_VALID = 0x02', 'PID_FLAG_LAST_APPLY_OK = 0x08',
                 'MOTOR_FLAG_DRIVER_ALARM = 0x04', 'POWER_STATE_RUNNING = 0'):
        assert flag in py_node
        name, value = flag.split(' = ')
        assert f'const {name}: i64 = {value};' in main, flag
    for state in ('precheck', 'open_loop', 'fitting', 'verify', 'review', 'saving', 'done', 'failed', 'aborted'):
        assert f'"{state}"' in main, state
    assert 'flash save attempt {attempt}, flash_diag=0x{diag:04x}' in main
    assert 'make_parameter_handler' not in main


def test_rust_map_manager_keeps_the_same_safety_rules_and_wiring():
    """rust_map:=true swaps in mower_rs mower_map for map_manage_node: same
    node name, services, latched topics, safety floor, fail-closed rules and
    user-facing messages."""
    main = MAP_RS_MAIN.read_text(encoding='utf-8')
    grid = MAP_RS_GRID.read_text(encoding='utf-8')
    py_node = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_map'" in mission_launch
    assert "condition=IfCondition(rust_map)" in mission_launch
    assert "condition=UnlessCondition(rust_map)" in mission_launch
    assert mission_launch.count("name='map_manage'") == 2
    assert 'r2r::Node::create(ctx, "map_manage", "")' in main
    assert 'const MIN_SAFE_INFLATE_RADIUS_M: f64 = 0.75;' in main
    assert 'MIN_SAFE_INFLATE_RADIUS_M = 0.75' in py_node
    for service in ('/create_risk_map', '/create_free_space', '/import_image_mask', '/create_chennal_map',
                    '/get_zone_map_list_srv', '/restore_free_space_coverage', '/map_manage/get_parameters',
                    '/map_manage/set_parameters', '/map_manage/set_parameters_atomically'):
        assert f'"{service}"' in main, service
    for topic in ('/free_space', '/free_space_inflated', '/risk_map', '/risk_map_inflated', '/chennal_map',
                  '/chennal_map_inflated', '/map_grid', '/map_grid_global', '/nav_operation_active', '/mission_operation_lock'):
        assert f'"{topic}"' in main, topic
    # every Chinese / English user-facing message of the Python node survives
    for text in re.findall(r"'([^'\n]*[\u4e00-\u9fff][^'\n]*)'", py_node):
        if '{' in text or text.endswith(': '):
            continue
        assert text in main or text in grid, text
    for text in ('held occupied until its risk map is ready', 'cannot fuse', 'base and channel grid geometries must match',
                 'base and risk grid orientations must match', 'channel point is outside the navigation map',
                 'mask_encoding must be base64_u8_row_major', 'robot_pose_header.frame_id must be map',
                 'inflate_radius_m cannot change while navigation is active or unknown', 'inflate_radius_m must be finite'):
        assert text in main or text in grid, text
    assert 'guard.acquire("refresh inflated maps")' in main
    assert 'make_parameter_handler' not in main


def test_rust_coverage_node_keeps_the_same_safety_rules_and_wiring():
    """rust_coverage:=true swaps in mower_rs mower_coverage for coverage_node:
    same node name, services, action, topics, parameter defaults, dispatch
    deadlines, retry cadence and user-facing messages."""
    main = COVERAGE_RS_MAIN.read_text(encoding='utf-8')
    py_node = COVERAGE_NODE.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_coverage'" in mission_launch
    assert "condition=IfCondition(rust_coverage)" in mission_launch
    assert "condition=UnlessCondition(rust_coverage)" in mission_launch
    assert mission_launch.count("name='boustrophedon_coverage'") == 2
    assert 'r2r::Node::create(r2r_ctx, "boustrophedon_coverage", "")' in main
    for name in ('/generate_coverage_path', '/zone_exec_path', '/run_zone_sequence', '/stop_zone_sequence',
                 '/get_zone_map_list_srv', '/confirm_navigation_dispatch', '/cancel_navigation_dispatch',
                 '/check_nav_status', '/get_channel_route', '/mission_operation_lock', 'nav_action_follow_path',
                 '/coverage_path', '/coverage_path_markers', '/coverage_invalid_segments', '/coverage_connectors',
                 '/risk_map', '/risk_map_inflated', '/nav_operation_active'):
        assert f'"{name}"' in main, name
    for name, default in re.findall(r"self\.declare_parameter\('(\w+)', ([^)]+)\)", py_node):
        rust_default = default.replace("'", '"').replace('True', 'true').replace('False', 'false')
        assert f'"{name}", {rust_default}' in main, (name, default)
    # the bounded dispatch: 3 s acceptance, 600 s result, 2 s confirmation, 2 s cancel cadence, 0.25 s fallback wait
    assert 'let acceptance_timeout_s = 3.0;' in main and 'let timeout_s = 600.0;' in main
    assert 'Duration::from_secs(2)' in main and 'Duration::from_millis(250)' in main
    assert '(0.5..=30.0).contains(&t)' in main
    for text in re.findall(r"'([^'\n]*[\u4e00-\u9fff][^'\n]*)'", py_node):
        if '{' in text or text.endswith(': '):
            continue
        assert text in main, text
    for text in ('Navigation action goal accepted', 'Zone not found', 'Zone coverage path is empty',
                 'Navigation action server unavailable', 'busy or a safety precondition failed',
                 'A previous zone-sequence goal is not terminal; the new accepted goal is being canceled',
                 'Zone sequence was canceled before goal dispatch', 'Navigation action completed unsuccessfully',
                 'is startup-only; restart coverage_node with the desired backend',
                 'coverage parameters cannot change while navigation or coverage generation is active/unknown',
                 'action is still not terminal; retrying correlated cancel',
                 'action cancellation acknowledgment timed out', 'navigation action is now terminal'):
        assert text in main, text
    assert 'make_parameter_handler' not in main


def test_rust_agent_keeps_the_same_backend_protocol_and_wiring():
    """rust_agent:=true swaps in mower_rs mower_agent for the Python fleet
    agent: same endpoints, constants, relay framing, control messages, log
    lines and launch arguments."""
    main = AGENT_RS_MAIN.read_text(encoding='utf-8')
    relay = AGENT_RS_RELAY.read_text(encoding='utf-8')
    py = AGENT_PY.read_text(encoding='utf-8')
    launch = ROSBRIDGE_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_agent'" in launch and launch.count("name='mower_agent'") == 2
    assert "condition=IfCondition(rust_agent)" in launch and "condition=UnlessCondition(rust_agent)" in launch
    assert launch.count("'--rosbridge', 'ws://127.0.0.1:9091'") == 2
    api = re.search(r'^ROBOT_API_VERSION = (\d+)', VERSION_PY.read_text(encoding='utf-8'), re.M).group(1)
    assert f'const ROBOT_API_VERSION: i64 = {api};' in main
    for const in ('HEARTBEAT_S = 10', 'TELEMETRY_THROTTLE_MS = 5000', 'REGISTER_RETRY_S = 30', 'RECONNECT_MAX_S = 60',
                  'HTTP_RELAY_TIMEOUT_S = 10', 'TURN_REFRESH_S = 3600', 'TURN_RETRY_S = 60'):
        assert const in py
        name, value = const.split(' = ')
        assert re.search(rf'const {name}: \w+ = {value};', main), const
    assert 'HTTP_RELAY_MAX_BODY: usize = 64 * 1024;' in main
    for text in ('/v1/robots/register', '/v1/relay/robot/', '/turn', '/v3/config/global/patch', 'webrtcICEServers2',
                 'ws://127.0.0.1:9090', 'ws://127.0.0.1:9091', 'http://127.0.0.1:8889', 'http://127.0.0.1:9997',
                 'MOWER_BACKEND_URL not set: agent idle (development mode)', 'MOWER_PROVISION_TOKEN not set: skipping registration',
                 'relay refused us: HTTP', 'reconnecting in', 'gate refused', 'gate unreachable', 'no such session',
                 'session ended', 'relay lost', 'not relayed', 'camera server unreachable', 'not implemented yet',
                 'another device_key holds this robot_id', 'bad provision token', '"X-Mower-Client"', '"@robot"'):
        assert text in main, text
    for const in ('SUBPROTOCOL: &str = "mrelay1"', 'SID_BYTES: usize = 8', 'T_TEXT: u8 = 0x01', 'T_TEXT_MORE: u8 = 0x11',
                  'T_BIN: u8 = 0x02', 'T_BIN_MORE: u8 = 0x12', 'CHUNK_SIZE: usize = 512 * 1024', 'MAX_MESSAGE: usize = 64 * 1024 * 1024'):
        assert const in relay, const
    assert '"transport=udp", "transport=tcp"' in main


def test_rust_battery_node_keeps_the_same_model_and_wiring():
    """rust_battery:=true swaps in mower_rs mower_battery for
    battery_state_node: same node name, topics, OCV table, launch
    parameters and BatteryState semantics."""
    main = BATTERY_RS_MAIN.read_text(encoding='utf-8')
    estimator = BATTERY_RS_ESTIMATOR.read_text(encoding='utf-8')
    python_estimator = BATTERY_ESTIMATOR.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_battery'" in mission_launch
    assert "condition=IfCondition(rust_battery)" in mission_launch
    assert "condition=UnlessCondition(rust_battery)" in mission_launch
    assert mission_launch.count("name='battery_state'") == 2
    assert mission_launch.count("'charger_present_min_v': 25.0") == 2
    assert mission_launch.count("'meter_current_wired': False") == 2
    assert 'r2r::Node::create(ctx, "battery_state", "")' in main
    for name, default in (
        ('base_telemetry_topic', '"/mower_base/telemetry"'),
        ('battery_topic', '"/battery_state"'),
        ('aon_battery_topic', '"/aon_battery_state"'),
        ('stale_timeout_s', '5.0'),
        ('charger_present_min_v', '25.0'),
        ('low_battery_pct', '20.0'),
        ('frame_id', '"base_footprint"'),
    ):
        assert f'"{name}", {default}' in main, name
    # the OCV table is the Python one, row for row
    python_rows = re.findall(r'\((\d\.\d\d), (\d\.\d\d)\)', python_estimator)
    rust_rows = re.findall(r'\((\d\.\d\d), (\d\.\d\d)\)', estimator)
    assert python_rows and rust_rows[:len(python_rows)] == python_rows
    assert 'msg.percentage = est.fraction as f32' in main  # 0..1, not %
    assert 'POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2' in main
    assert 'POWER_SUPPLY_HEALTH_DEAD: u8 = 3' in main
    assert 'POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4' in main
    assert 'make_parameter_handler' not in main


def test_imu_drivers_are_respawned_after_a_serial_failure():
    """Both IMU drivers exit on a USB I/O error (fail-closed for the health
    gate); launch must restart them once /dev/imu_usb is back."""
    source = MOWER_LAUNCH.read_text(encoding='utf-8')
    for executable in ("executable='wit_ros2_imu'", "executable='mower_imu'"):
        # the Node(...) block ends at the first line that is just ')'
        block = source.split(executable, 1)[1].split('\n    )\n', 1)[0]
        assert 'respawn=True' in block, executable
        assert 'respawn_delay=2.0' in block, executable


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
    # runtime is the last stage: the GPS driver ships inside it
    assert '\nFROM ' not in runtime
    assert 'ENV HOME=/home/${USER_NAME}' in runtime


def test_gps_driver_runs_inside_the_runtime_container():
    """The receiver driver is a node of mower.launch.py in the one robot
    image, not a second compose service with its own image: same DDS graph
    (network_mode/ipc host), one image tag to update, and the receiver
    parameters are versioned with the launch that consumes them. It is
    mower_rs/mower_gps, not the ROS ublox_gps node, because that one spins
    forever on a dead port instead of exiting for launch's respawn."""
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    head, block = mower.split("executable='mower_gps'", 1)
    assert head.rstrip().endswith("package='mower_rs',")
    block = block.split('\n    )\n', 1)[0]
    assert "name='gps'" in block
    assert 'condition=IfCondition(enable_gps)' in block
    assert 'respawn=True' in block
    assert 'respawn_delay=2.0' in block
    # the canonical topic goes in as a parameter, no remapping
    assert "parameters=[gps_params_file, {'fix_topic': gps_fix_topic}]" in block
    assert "package='ublox_gps'" not in mower

    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    assert "'enable_gps': enable_gps," in robot
    assert "'gps_params_file': gps_params_file," in robot

    params = GPS_PARAMS.read_text(encoding='utf-8')
    assert params.startswith('#') and '\ngps:\n' in params
    assert 'device: /dev/gps_rtk' in params
    # antenna link of real_robot.xacro so navsat_transform applies the offset
    assert 'frame_id: gps_link' in params
    assert 'rate_hz: 4.0' in params
    assert 'stale_timeout_s: 5.0' in params

    manifest = BRINGUP_PACKAGE.read_text(encoding='utf-8')
    assert '<exec_depend>mower_rs</exec_depend>' in manifest
    assert 'ublox' not in manifest
    cmake = MOWER_RS_CMAKE.read_text(encoding='utf-8')
    assert re.search(r'^set\(MOWER_RS_BINARIES .* mower_gps\)$', cmake, re.M)
    cargo = MOWER_RS_CARGO.read_text(encoding='utf-8')
    assert '"crates/mower_ubx"' in cargo and '"crates/mower_gps"' in cargo

    deploy = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    lawan = deploy.split('  mediamtx:', 1)[0]
    assert 'enable_gps:=${GPS:-true}' in lawan
    assert '- /dev/gps_rtk:/dev/gps_rtk' in lawan
    assert 'profiles:' not in deploy
    assert 'mower_path_planning-gps' not in deploy
    assert 'ublox' not in deploy

    dockerfile = DOCKERFILE.read_text(encoding='utf-8')
    assert 'gps_runtime' not in dockerfile
    workflow = BUILD_WORKFLOW.read_text(encoding='utf-8')
    assert 'gps_runtime' not in workflow
    assert 'image_suffix: -gps' not in workflow
    assert 'install/mower_rs/lib/mower_rs/mower_gps' in workflow


def test_gps_driver_fails_closed_and_keeps_the_ublox_gps_fix_contract():
    """Rules the navigation gate and navsat_transform rely on, checked at
    the source so a refactor cannot quietly relax them: exit on serial
    error / EOF / missing NAV-PVT, NO_FIX -> NaN, GBAS only for RTK fixed."""
    main = GPS_RS_MAIN.read_text(encoding='utf-8')
    assert 'returned EOF' in main
    assert 'no NAV-PVT from the receiver for' in main
    assert 'no NAV-PVT from the receiver within' in main
    assert 'std::process::exit(exit_code)' in main
    assert 'serial_thread.is_finished()' in main
    assert 'COVARIANCE_TYPE_DIAGONAL_KNOWN' in main
    ubx = UBX_RS.read_text(encoding='utf-8')
    assert 'matches!(self.fix_type, 2 | 3 | 4)' in ubx
    assert ('if self.carrier_solution() == CarrierSolution::Fixed {\n'
            '            2\n') in ubx
    assert 'if status < 0 { (f64::NAN, f64::NAN, f64::NAN) }' in ubx


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

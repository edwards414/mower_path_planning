"""Regression checks for real/sim drivetrain geometry and frame alignment."""

import ast
import json
import os
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
DATA_COLLECTION_HELPER = SRC_DIR.parent / 'deploy/host/mower-data-collection.sh'
DEPLOY_INSTALL = SRC_DIR.parent / 'deploy/install.sh'
ROBOT_LAUNCH = SRC_DIR / 'mower_bringup/launch/robot.launch.py'
DOCKERFILE = SRC_DIR.parent / 'Dockerfile'
BUILD_WORKFLOW = SRC_DIR.parent / '.github/workflows/build.yml'
GPS_PARAMS = SRC_DIR / 'mower_bringup/config/gps.yaml'
UDEV_RULES = SRC_DIR.parent / 'deploy/udev/99-mower.rules'
BRINGUP_PACKAGE = SRC_DIR / 'mower_bringup/package.xml'
MOWER_RS_CMAKE = SRC_DIR / 'mower_rs/CMakeLists.txt'
MOWER_RS_CARGO = SRC_DIR / 'mower_rs/Cargo.toml'
MOWER_RSD_PARAMS = SRC_DIR / 'mower_bringup/config/mower_rsd.yaml'
NAV2_CONFIG = SRC_DIR / 'mower_nav2/config/nav2_no_map_params.yaml'
NAV2_LAUNCH = SRC_DIR / 'mower_nav2/launch/navigation.launch.py'
MAKEFILE = SRC_DIR.parent / 'Makefile'
MOWER_SYSTEM = SRC_DIR / 'mower_controller/src/mower_system.cpp'
ENV_EXAMPLE = SRC_DIR.parent / 'deploy/.env.example'
STM_COMMS = SRC_DIR / 'mower_controller/src/Stm_Comms.cpp'
IMU_DRIVER = SRC_DIR / 'wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py'
PID_AUTOTUNE_NODE = SRC_DIR / 'mower_mission/mower_mission/pid_autotune_node.py'
PID_TUNING = SRC_DIR / 'mower_mission/mower_mission/pid_tuning.py'
MAP_MANAGE_NODE = SRC_DIR / 'mower_mission/mower_mission/map_manage_node.py'
MISSION_SETUP = SRC_DIR / 'mower_mission/setup.py'
AGENT_PY = SRC_DIR / 'mower_mission/mower_mission/mower_agent.py'
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
BATTERY_ESTIMATOR = (
    SRC_DIR / 'mower_mission/mower_mission/battery_estimator.py'
)
VELOCITY_GUARD = (
    SRC_DIR / 'mower_bringup/mower_bringup/velocity_command_guard.py'
)
AUTO_COVERAGE = (
    SRC_DIR / 'mower_mission/mower_mission/auto_coverage_node.py'
)
# What an rclpy fallback and the Rust module that replaced it must agree on.
# These tests check the Python side against them, and the crates' own tests
# the Rust side (E9-07: no test here reads a .rs file; the Rust halves of the
# checks that used to are tests of those crates now).
PID_AUTOTUNE_PARITY = (
    SRC_DIR / 'mower_rs/crates/mower_pid_autotune/tests/rclpy_parity.json'
)
MAP_PARITY = SRC_DIR / 'mower_rs/crates/mower_map/tests/rclpy_parity.json'
AGENT_PARITY = SRC_DIR / 'mower_rs/crates/mower_agent/tests/rclpy_parity.json'
BATTERY_PARITY = SRC_DIR / 'mower_rs/crates/mower_battery/tests/rclpy_parity.json'


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


def test_rust_daemon_runs_the_enabled_modules_in_one_process():
    """rust_daemon:=true starts one mower_rsd (one r2r Context, one DDS
    participant) with the module set derived from the same rust_* switches,
    and every separate mower_rs binary stays down. Node names, the per-node
    parameter file and the node-scoped remappings are the contract."""
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_rsd'" in robot_launch
    # derived from the existing switches, not a second source of truth
    for switch in ('rust_status', 'rust_guards', 'rust_imu', 'enable_gps',
                   'rust_map', 'rust_nav', 'rust_record',
                   'rust_adapter', 'rust_battery', 'rust_pid_autotune',
                   'rust_bridge', 'rust_agent'):
        assert f"'{switch}')," in robot_launch, switch
    # ... except the coverage planner, which has no rclpy version and
    # therefore no switch: always in the set
    assert "('coverage', None)," in robot_launch
    assert 'if switch is None' in robot_launch
    # a bare __node:= would rename every node in the process, so the Node
    # action must not carry a name= at all
    node_block = robot_launch.split("executable='mower_rsd',", 1)[1].split(
        ')]', 1)[0]
    assert 'name=' not in node_block
    # launch remappings become node-scoped rcl rules
    assert "'imu:imu/data_raw:=imu/data'" in robot_launch
    assert "'manual_velocity_guard:cmd_vel_in:=/app_joy_cmd'" in robot_launch
    assert "'velocity_command_guard:cmd_vel_out:=/drivetrain_guarded_cmd_vel'" \
        in robot_launch
    assert 'respawn=True' in robot_launch

    # one parameter section per node name of the module table (the table:
    # mower_rsd's module_table_carries_the_production_node_names)
    params = MOWER_RSD_PARAMS.read_text(encoding='utf-8')
    for node in ('robot_status', 'manual_velocity_guard',
                 'velocity_command_guard', 'map_manage',
                 'boustrophedon_coverage', 'nav_action_server',
                 'path_record_node', 'flutter_adapter', 'battery_state',
                 'pid_autotune'):
        assert f'\n{node}:\n  ros__parameters:' in params, node

    # the daemon and the separate binaries are mutually exclusive everywhere
    for launch in (MISSION_LAUNCH, TWIST_MUX_LAUNCH, ROSBRIDGE_LAUNCH,
                   MOWER_LAUNCH):
        text = launch.read_text(encoding='utf-8')
        assert '_rust_binary(' in text, launch.name
        assert "'rust_daemon'" in text, launch.name


def test_ws_bridge_bus_is_a_run_time_switch_that_is_off_by_default(monkeypatch):
    """ws_bridge_bus:=true (WS_BRIDGE_BUS) moves mower_ws_bridge onto
    mower_bus (--backend bus), and the bus bridge then carries the names only
    DDS has for it. It is the app's only door, the in-app update included,
    so it is a launch switch, off unless .env says otherwise, and the
    roll-back needs no new image. Off, mower_rsd's command line is today's."""
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mission = MISSION_LAUNCH.read_text(encoding='utf-8')
    rosbridge = ROSBRIDGE_LAUNCH.read_text(encoding='utf-8')
    for text in (robot, mission, rosbridge):
        assert re.search(r"'ws_bridge_bus',\n\s+default_value='false',", text)
    block = robot.split("if 'bridge' in modules:", 1)[1].split(
        "if 'agent' in modules:", 1)[0]
    assert ("if _truthy(LaunchConfiguration('ws_bridge_bus')"
            ".perform(context)):") in block
    assert "bridge_args += ['--backend', 'bus']" in block
    assert "'ws_bridge_bus': ws_bridge_bus," in robot
    assert "'ws_bridge_bus': LaunchConfiguration('ws_bridge_bus')," in mission
    # the separate binary (rust_daemon:=false): r2r unless the switch is on
    assert "'--backend', PythonExpression([" in rosbridge
    assert "\"'bus' if '\", ws_bridge_bus" in rosbridge
    assert 'ws_bridge_bus:=${WS_BRIDGE_BUS:-false}' in \
        DEPLOY_COMPOSE.read_text(encoding='utf-8')
    assert re.search(r'^WS_BRIDGE_BUS=false$',
                     ENV_EXAMPLE.read_text(encoding='utf-8'), re.M)
    # the Rust side reads the same argument: mower_rsd for the bus bridge's
    # plan, the binary for its runner (mower_rsd's
    # ws_bridge_is_on_the_bus_with_its_bus_backend, mower_ws_bridge's
    # the_backend_is_r2r_unless_told_otherwise and tests/source_rules.rs)

    # the behavioural half needs ROS launch (not in the python-tests job)
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    module = _load_launch_module(ROBOT_LAUNCH)
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: f'/share/{package}')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    configured = {switch: 'false' for _, switch in module._DAEMON_MODULES
                  if switch}
    configured.update(rust_daemon='true', rust_bridge='true',
                      rosbridge_address='10.77.0.2',
                      rust_daemon_params_file='/cfg/mower_rsd.yaml')
    today = ('bridge=--address 10.77.0.2 --port 9090 --loopback-port 9091 '
             '--policy /share/mower_bringup/config/ws_bridge.yaml')
    for value, extra in (('false', ''), ('0', ''), ('', ''),
                         ('true', ' --backend bus'), ('on', ' --backend bus')):
        context = LaunchContext()
        context.launch_configurations.update(configured, ws_bridge_bus=value)
        [node] = module._rust_daemon_node(context)
        arguments = node['arguments']
        bridge = [arguments[i + 1] for i, a in enumerate(arguments)
                  if a == '--module-args' and
                  arguments[i + 1].startswith('bridge=')]
        assert bridge == [today + extra], repr(value)


def test_rust_daemon_recorder_uses_the_launch_directories():
    """With rust_daemon the recorder must save where the separate mower_record
    does (zone_record_dir / sites_dir = the ~/.mower bind mount), not at the
    yaml's relative save_dir, which resolves against the container WORKDIR
    and loses every zone with the container. The override is node-scoped (a
    launch_ros dict would go to every node under /**) and listed after the
    yaml so it wins."""
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')
    block = robot_launch.split("if 'record' in modules:", 1)[1].split(
        'remaps =', 1)[0]
    assert "_node_params_file('path_record_node', {" in block
    assert "'save_dir': LaunchConfiguration('zone_record_dir')" in block
    assert "'sites_dir': LaunchConfiguration('sites_dir')" in block
    assert 'parameters.append(' in block
    assert "yaml.safe_dump({node: {'ros__parameters': params}}, f)" \
        in robot_launch


def _flat_ros_parameters(section, prefix=''):
    """A ros__parameters mapping the way rcl names it: nested keys joined by
    dots."""
    flat = {}
    for key, value in section.items():
        if isinstance(value, dict):
            flat.update(_flat_ros_parameters(value, f'{prefix}{key}.'))
        else:
            flat[prefix + key] = value
    return flat


def test_rust_nav_core_swaps_nav2_for_mower_nav_core_and_nothing_else(
        monkeypatch):
    """rust_nav_core:=true (compose RUST_NAV_CORE, default false): the nav
    module steers with mower_nav_core's NavStack (nav_backend nav_core,
    nav_core.pose_source tfbus, scoped to nav_action_server and listed after
    mower_rsd.yaml) and mower.launch.py gets enable_navigation:=false, which
    gates the navigation.launch.py include -- every nav2 server, or their
    container -- and nothing else. It needs rust_daemon, rust_nav and
    rust_localize and is refused before anything starts without them. False
    sets no nav parameter and nav2 runs, as before the switch existed."""
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    nav2 = NAV2_LAUNCH.read_text(encoding='utf-8')

    # declared off; .env switches it like the others
    argument = re.search(
        r"DeclareLaunchArgument\(\s*'rust_nav_core'(.*?)\n\s*\),",
        robot, flags=re.DOTALL)
    assert argument is not None
    assert "default_value='false'" in argument.group(1)
    lawan = DEPLOY_COMPOSE.read_text(encoding='utf-8').split('  mediamtx:', 1)[0]
    assert '- rust_nav_core:=${RUST_NAV_CORE:-false}' in lawan
    assert re.search(r'^RUST_NAV_CORE=false$',
                     ENV_EXAMPLE.read_text(encoding='utf-8'), re.M)

    # refused before the includes start anything
    items = robot.split('return LaunchDescription([', 1)[1]
    assert (items.index('OpaqueFunction(function=_require_nav_core_prerequisites),')
            < items.index('mower_launch,'))
    assert ("_NAV_CORE_NEEDS = ('rust_daemon', 'rust_nav', 'rust_localize')"
            in robot)

    # forwarded to mower.launch.py only, with _truthy's spellings, and always
    # as a value IfCondition accepts
    mower_include = robot.split("'mower.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert "'enable_navigation': _nav2_enabled(rust_nav_core)," in mower_include
    mission_include = robot.split("'mission.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert 'enable_navigation' not in mission_include
    helper = robot.split('def _nav2_enabled(', 1)[1].split('\ndef ', 1)[0]
    assert ('''"'false' if '", rust_nav_core, "'.strip().lower() in ", _TRUE,'''
            in helper)
    # with the flag off an explicit enable_navigation:=false is kept
    assert ('''" else '", LaunchConfiguration('enable_navigation', default='true'), "'",'''
            in helper)
    robot_true = ast.literal_eval(re.search(
        r"return value\.strip\(\)\.lower\(\) in (\{[^}]*\})", robot).group(1))
    assert set(ast.literal_eval(re.search(
        r'^_TRUE = "(.*)"$', robot, re.M).group(1))) == robot_true

    # mower.launch.py reads enable_navigation (default true, the value it gets
    # with rust_nav_core off) once: as the condition of the
    # navigation.launch.py include, whose one non-nav2 node, the battery
    # simulator, it switches off
    tree = ast.parse(mower)
    reads = [node for node in ast.walk(tree)
             if isinstance(node, ast.Name) and node.id == 'enable_navigation'
             and isinstance(node.ctx, ast.Load)]
    navigation_launch = next(
        node.value for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and [t.id for t in node.targets if isinstance(t, ast.Name)]
        == ['navigation_launch'])
    assert len(reads) == 1
    assert any(node is reads[0] for node in ast.walk(navigation_launch))
    assert mower.count("LaunchConfiguration('enable_navigation')") == 1
    assert re.search(r"'enable_navigation',\s*default_value='true',", mower)
    nav_include = mower.split('navigation_launch = IncludeLaunchDescription(',
                              1)[1].split('\n    )\n', 1)[0]
    assert "'navigation.launch.py'," in nav_include
    assert 'condition=IfCondition(enable_navigation),' in nav_include
    assert "'launch_battery_simulator': 'false'," in nav_include
    # ... which starts nav2 and nothing else
    for package in re.findall(r"package='([^']+)'", nav2):
        assert package.startswith('nav2_') or package in (
            'rclcpp_components', 'mower_mission'), package
    assert nav2.count("package='mower_mission'") == 1
    assert "executable='battery_simulator_node'" in nav2
    assert 'condition=IfCondition(launch_battery_simulator),' in nav2
    # ... and nav2 starts from nowhere else, while the mux and
    # robot_state_publisher are not behind that switch (rsp is behind E4's
    # rust_static_tf only)
    for launch in (ROBOT_LAUNCH, MOWER_LAUNCH, MISSION_LAUNCH,
                   TWIST_MUX_LAUNCH, DUAL_EKF_LAUNCH, ROSBRIDGE_LAUNCH,
                   REAL_CONTROLLER_LAUNCH):
        text = launch.read_text(encoding='utf-8')
        assert "package='nav2_" not in text, launch.name
        assert "executable='component_container" not in text, launch.name
    for name in ('twist_mux_launch = IncludeLaunchDescription(',
                 'robot_state_publisher = Node('):
        block = mower.split(name, 1)[1].split('\n    )\n', 1)[0]
        assert 'enable_navigation' not in block and 'nav_core' not in block, name
    mux_include = mower.split('twist_mux_launch = IncludeLaunchDescription(',
                              1)[1].split('\n    )\n', 1)[0]
    assert 'condition=' not in mux_include

    # the nav module's backend, node-scoped like the recorder's directories,
    # after mower_rsd.yaml -- which sets neither key, so with the switch off
    # mower_nav keeps its own defaults (backend nav2)
    block = robot.split("if 'nav' in modules and _truthy(", 1)[1].split(
        'remaps =', 1)[0]
    assert "LaunchConfiguration('rust_nav_core').perform(context)" in block
    assert "parameters.append(_node_params_file('nav_action_server', {" in block
    assert "'nav_backend': 'nav_core'," in block
    assert "'nav_core': {'pose_source': 'tfbus'}," in block
    assert robot.count("'nav_backend'") == 1
    import yaml
    nav_section = yaml.safe_load(MOWER_RSD_PARAMS.read_text(encoding='utf-8'))[
        'nav_action_server']['ros__parameters']
    assert not [key for key in _flat_ros_parameters(nav_section)
                if key == 'nav_backend' or key.startswith('nav_core.')]

    # switch.sh writes it on every switch (false unless given), checks the
    # compose passes it and the container got it
    switch = (SRC_DIR / 'mower_rs/tools/switch.sh').read_text(encoding='utf-8')
    passthrough = re.search(r'^PASSTHROUGH=\((.*?)\)$', switch,
                            re.M | re.S).group(1)
    assert "'rust_nav_core:=${RUST_NAV_CORE'" in passthrough
    for text in re.findall(r"'([^']+)'", passthrough):
        assert text in lawan, text
    assert '[RUST_NAV_CORE]=false' in switch
    assert ('for k in NAV_COMPOSITION RUST_BASE RUST_LOCALIZE RUST_DAEMON '
            'RUST_NAV_CORE RUST_MUX RUST_STATIC_TF RUST_RECORDER; do') in switch

    # the behavioural half needs ROS launch (not in the python-tests job)
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    module = _load_launch_module(ROBOT_LAUNCH)

    def context(**configurations):
        ctx = LaunchContext()
        ctx.launch_configurations.update(configurations)
        return ctx

    spellings = ('true', 'True', ' on', 'ON', 'yes', '1',
                 'false', 'FALSE', '0', 'off', 'no', 'maybe', '')
    for value in spellings:
        assert module._nav2_enabled(value).perform(context()) == (
            'false' if module._truthy(value) else 'true'), repr(value)
        assert module._nav2_enabled(value).perform(
            context(enable_navigation='false')) == 'false', repr(value)

    needs = {switch: 'true' for switch in module._NAV_CORE_NEEDS}
    for value in spellings:
        assert module._require_nav_core_prerequisites(
            context(rust_nav_core=value, **needs)) == [], repr(value)
        for switch in needs:
            missing = context(rust_nav_core=value, **{**needs, switch: 'off'})
            if not module._truthy(value):
                assert module._require_nav_core_prerequisites(missing) == []
                continue
            with pytest.raises(RuntimeError, match=f'got {switch}:=off:'):
                module._require_nav_core_prerequisites(missing)

    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: f'/share/{package}')
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    configured = {switch: 'false' for _, switch in module._DAEMON_MODULES
                  if switch}
    configured.update(rust_daemon='true', rust_nav='true',
                      rust_localize='true', gps_fix_topic='/fix',
                      rust_daemon_params_file='/cfg/mower_rsd.yaml',
                      localize_params_file='/cfg/localize.yaml')
    for value in spellings:
        [node] = module._rust_daemon_node(
            context(rust_nav_core=value, **configured))
        first, *extra, last = node['parameters']
        assert (first, last) == ('/cfg/mower_rsd.yaml', '/cfg/localize.yaml')
        if not module._truthy(value):
            assert extra == [], repr(value)
            continue
        [path] = extra
        written = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        Path(path).unlink()
        assert list(written) == ['nav_action_server'], repr(value)
        assert _flat_ros_parameters(
            written['nav_action_server']['ros__parameters']) == {
                'nav_backend': 'nav_core', 'nav_core.pose_source': 'tfbus'}


def test_rust_mux_replaces_twist_mux_only_inside_the_daemon():
    """rust_mux:=true (compose RUST_MUX, default false; E4): mower_rsd's mux
    module (mower_mux, node twist_mux) instead of the C++ twist_mux, on the
    same twist_mux_topics.yaml and the same output rule. It takes effect only
    with rust_daemon: twist_mux.launch.py starts the C++ node unless both are
    on, and robot.launch.py puts mux in the module set exactly then, so every
    spelling starts one mux, never two and never none."""
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    mux_launch = TWIST_MUX_LAUNCH.read_text(encoding='utf-8')

    # declared off, and threaded down robot -> mower -> twist_mux.launch.py
    for text in (robot, mower):
        argument = re.search(
            r"DeclareLaunchArgument\(\s*'rust_mux'(.*?)\n\s*\)", text,
            flags=re.DOTALL)
        assert argument is not None
        assert "default_value='false'" in argument.group(1)
    assert "DeclareLaunchArgument('rust_mux', default_value='false')," \
        in mux_launch
    mower_include = robot.split("'mower.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert "'rust_mux': rust_mux," in mower_include
    mux_include = mower.split('twist_mux_launch = IncludeLaunchDescription(',
                              1)[1].split('\n    )\n', 1)[0]
    assert "'rust_mux': LaunchConfiguration('rust_mux')," in mux_include
    assert "'rust_daemon': LaunchConfiguration('rust_daemon')," in mux_include
    # the C++ node, behind both switches; its include stays unconditional
    node = mux_launch.split('twist_mux = Node(', 1)[1].split('\n    )\n', 1)[0]
    assert 'condition=_unless_in_daemon(rust_mux, rust_daemon),' in node
    assert "remappings=[('/cmd_vel_out', '/cmd_vel_guard_input')]," in node
    assert 'condition=' not in mux_include

    # the module: after the guards, the same output rule, the same yaml
    assert "('guards', 'rust_guards'),\n    ('mux', 'rust_mux')," in robot
    assert "'mux': ['twist_mux:cmd_vel_out:=/cmd_vel_guard_input']," in robot
    block = robot.split("if 'mux' in modules:", 1)[1].split('env = {}', 1)[0]
    assert ("parameters.append(os.path.join(bringup, 'config', "
            "'twist_mux_topics.yaml'))") in block
    # the module table's mux entry (id, switch, node, the crate it runs):
    # mower_rsd's tests/module_table.rs and tests/source_rules.rs
    assert '"crates/mower_mux"' in MOWER_RS_CARGO.read_text(encoding='utf-8')
    # the twist_mux_topics.yaml key is the node name both muxes have
    assert TWIST_MUX_CONFIG.read_text(encoding='utf-8').startswith(
        'twist_mux:\n  ros__parameters:\n')

    # compose and .env, default off
    lawan = DEPLOY_COMPOSE.read_text(encoding='utf-8').split('  mediamtx:', 1)[0]
    assert '- rust_mux:=${RUST_MUX:-false}' in lawan
    assert re.search(r'^RUST_MUX=false$',
                     ENV_EXAMPLE.read_text(encoding='utf-8'), re.M)
    # switch.sh writes it every time, passes it, and checks the processes
    switch = (SRC_DIR / 'mower_rs/tools/switch.sh').read_text(encoding='utf-8')
    passthrough = re.search(r'^PASSTHROUGH=\((.*?)\)$', switch,
                            re.M | re.S).group(1)
    assert "'rust_mux:=${RUST_MUX'" in passthrough
    assert '[RUST_MUX]=false' in switch
    assert 'if truthy "$mux" && truthy "$daemon"; then' in switch
    assert '[ "$(n twist_mux)" -eq 0 ] || echo "C++ twist_mux still running"' \
        in switch
    assert '! rsd mux || echo "mower_rsd runs the mux module"' in switch

    # the behavioural half needs ROS launch (not in the python-tests job)
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    robot_module = _load_launch_module(ROBOT_LAUNCH)
    mux_module = _load_launch_module(TWIST_MUX_LAUNCH)
    spellings = ('true', 'True', ' on', 'ON', 'yes', '1',
                 'false', 'FALSE', '0', 'off', 'no', 'maybe', '')
    for flag in spellings:
        for daemon in spellings:
            rust = robot_module._truthy(flag) and robot_module._truthy(daemon)
            cpp = mux_module._unless_in_daemon(flag, daemon).evaluate(
                LaunchContext())
            assert cpp is not rust, (flag, daemon)


RECORD_LAUNCH = SRC_DIR / 'mower_recorder/launch/record.launch.py'


def _compose_service(compose, name):
    """One service's block of a compose file, as text."""
    return compose.split(f'\n  {name}:\n', 1)[1].split('\n\n  ', 1)[0]


def test_rust_recorder_replaces_the_python_recorder_and_is_off_by_default(
        tmp_path):
    """rust_recorder:=true (compose RUST_RECORDER, default false; E5-3a):
    mower_rsd's bag module (crates/mower_recorder: recorder_manager,
    bag_store, graph_snapshot) instead of the Python recorder, whose
    container then sleeps, so that /mower_recorder/* has one owner; with
    rust_daemon:=false the mower_recorder binary. Its parameters are what
    record.launch.py gives the Python nodes. Off by default everywhere: the
    module does not record yet (E5-3b), and with the switch off nothing
    runs that did not before and the recorder service runs what it ran."""
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mission = MISSION_LAUNCH.read_text(encoding='utf-8')
    compose = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    record = RECORD_LAUNCH.read_text(encoding='utf-8')

    # declared off in both launch files, threaded robot -> mission
    for text in (robot, mission):
        argument = re.search(
            r"DeclareLaunchArgument\(\s*'rust_recorder'(.*?)\n\s*\)", text,
            flags=re.DOTALL)
        assert argument is not None
        assert "default_value='false'" in argument.group(1)
    mission_include = robot.split("'mission.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert "'rust_recorder': rust_recorder," in mission_include
    # the module, last in the set, its id not the zone recorder's
    assert "    ('agent', 'rust_agent'),\n" in robot
    assert "    ('bag', 'rust_recorder'),\n)" in robot
    assert "('record', 'rust_record')," in robot
    block = robot.split("if 'bag' in modules:", 1)[1].split('env = {}', 1)[0]
    assert "_node_params_file('recorder_manager', {" in block
    assert "_node_params_file('bag_store', {" in block
    for key in ('robot_id', 'output_root', 'config_file',
                'qos_overrides_path', 'git_repo_dir', 'r2_env_file'):
        assert f"'{key}':" in block, key
    # the module table's bag entry (id, switch, the three nodes, the code
    # each runs): mower_rsd's tests/module_table.rs and tests/source_rules.rs
    assert '"crates/mower_recorder"' in MOWER_RS_CARGO.read_text(
        encoding='utf-8')
    assert 'mower_recorder' in re.search(
        r'^set\(MOWER_RS_BINARIES (.*)\)$',
        MOWER_RS_CMAKE.read_text(encoding='utf-8'), re.M).group(1).split()
    # the binary: switch on and daemon off, never `name=` (it would rename
    # all three nodes)
    node = mission.split('mower_recorder_rs = Node(', 1)[1].split(
        '\n    )\n', 1)[0]
    assert "executable='mower_recorder'," in node
    assert 'condition=_rust_binary_alone(rust_recorder, rust_daemon),' in node
    assert 'name=' not in node
    assert 'mower_recorder_rs,' in mission

    # record.launch.py's values: mower_rsd.yaml for the module, the binary's
    # dict for rust_daemon:=false
    import yaml
    sections = yaml.safe_load(MOWER_RSD_PARAMS.read_text(encoding='utf-8'))
    graph = sections['graph_snapshot']['ros__parameters']
    manager = sections['recorder_manager']['ros__parameters']
    store = sections['bag_store']['ros__parameters']
    watch = ast.literal_eval(
        re.search(r'watch_nodes = (\[.*?\])', record).group(1))
    dumps = ast.literal_eval(re.search(
        r"'params_dump_nodes': (\[.*?\]),", record, re.S).group(1))
    assert graph == {'period_s': 2.0, 'watch_nodes': watch,
                     'fault_topic': '/mower_recorder/fault'}
    assert watch == ['map_manage', 'boustrophedon_coverage']
    assert manager['params_dump_nodes'] == dumps
    assert manager['fault_topic'] == '/mower_recorder/fault'
    assert manager['gps_topic'] == '/fix'
    assert manager['min_free_mb'] == 2048
    assert store == {'upload_policy': 'wifi_or_dock',
                     'wifi_interfaces': ['wlan0'], 'dock_topic': ''}
    recorder = _compose_service(compose, 'recorder')
    assert '- autostart:=false' in recorder and manager['autostart'] is False
    assert f"- video_api_url:={manager['video_api_url']}" in recorder
    for text in ("'watch_nodes': ['map_manage', 'boustrophedon_coverage'],",
                 "'params_dump_nodes': ['/boustrophedon_coverage', "
                 "'/map_manage'],",
                 "'autostart': False,",
                 "'video_api_url': 'http://127.0.0.1:9997',",
                 "'upload_policy': 'wifi_or_dock',"):
        assert text in node, text

    # compose: lawan_node gets the switch and the recorder's robot id and R2
    # file; the recorder service runs today's launch unless it is on
    lawan = compose.split('  mediamtx:', 1)[0]
    assert '- rust_recorder:=${RUST_RECORDER:-false}' in lawan
    assert '- robot_id:=${MOWER_ROBOT_ID:-mower}' in lawan
    assert '- robot_id:=${MOWER_ROBOT_ID:-mower}' in recorder
    assert '- r2_env_file:=/home/mower/.mower/r2.env' in lawan
    assert '- r2_env_file:=/home/mower/.mower/r2.env' in recorder
    assert '- RUST_RECORDER=${RUST_RECORDER:-false}' in recorder
    assert re.search(r'^RUST_RECORDER=false$',
                     ENV_EXAMPLE.read_text(encoding='utf-8'), re.M)
    service = yaml.safe_load(compose)['services']['recorder']
    command = service['command']
    assert command[:2] == ['bash', '-c'] and command[3] == 'recorder'
    assert command[4:] == [
        'ros2', 'launch', 'mower_recorder', 'record.launch.py',
        'robot_id:=${MOWER_ROBOT_ID:-mower}',
        'output_root:=/home/mower/.mower/bags', 'autostart:=false',
        'r2_env_file:=/home/mower/.mower/r2.env',
        'video_api_url:=http://127.0.0.1:9997'], 'the launch as before'
    assert service['stop_signal'] == 'SIGINT'
    assert 'def _truthy(value):\n    return value.strip().lower() in ' \
        "{'true', '1', 'yes', 'on'}" in robot

    def truthy(value):
        return value.strip().lower() in {'true', '1', 'yes', 'on'}

    # the script as the container runs it: compose's $$ is a $; it sleeps
    # exactly when robot.launch.py puts the module in the set
    import shutil
    import subprocess
    script = command[2].replace('$$', '$')
    assert '${' not in command[2].replace('$${', '')
    stub = tmp_path / 'bin'
    stub.mkdir()
    (stub / 'sleep').write_text('#!/bin/sh\necho SLEPT "$@"\n')
    (stub / 'sleep').chmod(0o755)
    bash = shutil.which('bash')
    spellings = ('true', 'True', ' on', 'ON', 'yes', '1', 'false', 'FALSE',
                 '0', 'off', 'no', 'maybe', '')
    for value in spellings:
        out = subprocess.run(
            [bash, '-c', script, 'recorder', 'echo', 'LAUNCHED'],
            env={'PATH': f'{stub}:/usr/bin:/bin', 'RUST_RECORDER': value},
            capture_output=True, text=True, check=True).stdout.strip()
        assert out == ('SLEPT infinity' if truthy(value) else 'LAUNCHED'), \
            repr(value)

    # mower-data-collection.sh: start refuses while it is on, before it
    # touches anything (the root check comes after)
    helper = DATA_COLLECTION_HELPER.read_text(encoding='utf-8')
    assert helper.index('if rust_recorder_on; then') < helper.index(
        '    need_root\n    if systemctl is-active')
    (tmp_path / 'dc.yaml').write_text('services: {}\n')
    for value in spellings + ('"true"', "'on' # the module"):
        on = truthy(value.split(' #')[0].strip('"\''))
        if not on and os.geteuid() == 0:
            # as root, start would go on and really stop the host's services
            continue
        env_file = tmp_path / '.env'
        env_file.write_text(f'IMAGE_TAG=main\nRUST_RECORDER={value}\n')
        done = subprocess.run(
            [bash, str(DATA_COLLECTION_HELPER), 'start'],
            env={'PATH': '/usr/bin:/bin', 'ENV_FILE': str(env_file),
                 'DC_COMPOSE_FILE': str(tmp_path / 'dc.yaml'),
                 'MAIN_COMPOSE_FILE': str(tmp_path / 'none.yaml')},
            capture_output=True, text=True)
        assert done.returncode == 1, repr(value)
        if on:
            assert 'RUST_RECORDER=true' in done.stderr, repr(value)
            assert 'E5-5' in done.stderr and 'Nothing was changed' in \
                done.stderr
        else:
            assert done.stderr.strip() == 'run with sudo', repr(value)

    # switch.sh writes it every time, passes it, recreates the recorder
    # container and checks the processes
    switch = (SRC_DIR / 'mower_rs/tools/switch.sh').read_text(encoding='utf-8')
    passthrough = re.search(r'^PASSTHROUGH=\((.*?)\)$', switch,
                            re.M | re.S).group(1)
    assert "'rust_recorder:=${RUST_RECORDER'" in passthrough
    for text in re.findall(r"'([^']+)'", passthrough):
        assert text in lawan, text
    assert '[RUST_RECORDER]=false' in switch
    assert 'docker compose up -d lawan_node recorder' in switch
    assert 'rsd bag || echo "no mower_rsd running the bag module"' in switch
    assert '! rsd bag || echo "mower_rsd runs the bag module"' in switch

    # the behavioural half needs ROS launch (not in the python-tests job)
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    robot_module = _load_launch_module(ROBOT_LAUNCH)
    mission_module = _load_launch_module(MISSION_LAUNCH)
    assert robot_module._DAEMON_MODULES[-1] == ('bag', 'rust_recorder')
    # exactly one recorder for every spelling: the module, the binary, or
    # the recorder service's Python one (which sleeps on the switch alone)
    for flag in spellings:
        for daemon in spellings:
            binary = mission_module._rust_binary_alone(flag, daemon).evaluate(
                LaunchContext())
            module = robot_module._truthy(flag) and robot_module._truthy(daemon)
            python = not truthy(flag)
            assert [binary, module, python].count(True) == 1, (flag, daemon)


def test_production_twist_mux_config_never_meets_the_degenerate_rules():
    """twist_mux's rules for a timeout of 0 (fresh forever, also before the
    first message: it masks everything below it from the start), a priority
    of 0 (never selected, a priority-0 lock changes nothing) and a tie (the
    name that sorts first) are reproduced by both muxes, but production must
    not depend on them."""
    import yaml
    section = yaml.safe_load(TWIST_MUX_CONFIG.read_text(encoding='utf-8'))[
        'twist_mux']['ros__parameters']
    assert section['use_stamped'] is True
    handles = list(section['topics'].values()) + list(section['locks'].values())
    for handle in handles:
        assert isinstance(handle['timeout'], float) and handle['timeout'] > 0
        assert isinstance(handle['priority'], int) and handle['priority'] > 0
    priorities = [h['priority'] for h in section['topics'].values()]
    assert len(priorities) == len(set(priorities)), priorities


def test_rust_static_tf_replaces_robot_state_publisher_only_inside_the_daemon():
    """rust_static_tf:=true (compose RUST_STATIC_TF, default false; E4):
    mower_rsd's static_tf module (mower_static_tf, node static_tf_publisher)
    instead of robot_state_publisher, on the same URDF (robot.launch.py makes
    it with mower.launch.py's xacro call) and publishing what rsp publishes.
    It takes effect only with rust_daemon: mower.launch.py starts rsp unless
    both are on, and robot.launch.py puts static_tf in the module set exactly
    then, so every spelling starts one of the two."""
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    for text in (robot, mower):
        argument = re.search(
            r"DeclareLaunchArgument\(\s*'rust_static_tf'(.*?)\n\s*\)", text,
            flags=re.DOTALL)
        assert argument is not None
        assert "default_value='false'" in argument.group(1)
    mower_include = robot.split("'mower.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert "'rust_static_tf': rust_static_tf," in mower_include
    rsp = mower.split('robot_state_publisher = Node(', 1)[1].split(
        '\n    )\n', 1)[0]
    assert ("condition=_unless_in_daemon(LaunchConfiguration('rust_static_tf'),\n"
            "                                    LaunchConfiguration('rust_daemon')),") in rsp
    # the module, before localize, with the URDF rsp gets
    assert "('gps', 'enable_gps'),\n    ('static_tf', 'rust_static_tf'),\n" \
        "    ('localize', 'rust_localize')," in robot
    block = robot.split("if 'static_tf' in modules:", 1)[1].split(
        "if 'record' in modules:", 1)[0]
    assert "parameters.append(_node_params_file('static_tf_publisher', {" in block
    assert "'robot_description': _robot_description()," in block
    helper = robot.split('def _robot_description():', 1)[1].split('\ndef ', 1)[0]
    assert "get_package_share_directory('mower_description'), 'mower_robot'," \
        in helper
    assert "'real_robot.xacro')).toxml()" in helper
    assert "'mower_robot',\n        'real_robot.xacro'\n    )" in mower
    assert "xacro.process_file(robot_description_path).toxml()" in mower
    # the module table's static_tf entry (id, switch): mower_rsd's
    # tests/module_table.rs
    assert '"crates/mower_static_tf"' in MOWER_RS_CARGO.read_text(encoding='utf-8')
    lawan = DEPLOY_COMPOSE.read_text(encoding='utf-8').split('  mediamtx:', 1)[0]
    assert '- rust_static_tf:=${RUST_STATIC_TF:-false}' in lawan
    assert re.search(r'^RUST_STATIC_TF=false$',
                     ENV_EXAMPLE.read_text(encoding='utf-8'), re.M)
    switch = (SRC_DIR / 'mower_rs/tools/switch.sh').read_text(encoding='utf-8')
    passthrough = re.search(r'^PASSTHROUGH=\((.*?)\)$', switch,
                            re.M | re.S).group(1)
    assert "'rust_static_tf:=${RUST_STATIC_TF'" in passthrough
    assert '[RUST_STATIC_TF]=false' in switch
    assert ('[ "$(n robot_state_publisher)" -eq 0 ] || '
            'echo "C++ robot_state_publisher still running"') in switch

    pytest.importorskip('launch')
    pytest.importorskip('xacro')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    robot_module = _load_launch_module(ROBOT_LAUNCH)
    mower_module = _load_launch_module(MOWER_LAUNCH)
    spellings = ('true', 'True', ' on', 'ON', 'yes', '1',
                 'false', 'FALSE', '0', 'off', 'no', 'maybe', '')
    for flag in spellings:
        for daemon in spellings:
            rust = robot_module._truthy(flag) and robot_module._truthy(daemon)
            cpp = mower_module._unless_in_daemon(flag, daemon).evaluate(
                LaunchContext())
            assert cpp is not rust, (flag, daemon)


def test_rust_localize_replaces_the_two_ekfs_and_navsat_exclusively():
    """rust_localize:=true swaps the two robot_localization ekf_node processes
    and navsat_transform_node for one mower_rs mower_localize process with the
    same three node names. The switch must be exclusive: two publishers of
    map->odom would fight over tf and nav2 steers on it."""
    dual = DUAL_EKF_LAUNCH.read_text(encoding='utf-8')
    # every C++ node is gated off by the same switch that turns the module on
    assert dual.count('condition=cpp_localization,') == 3
    assert 'cpp_localization = _cpp_localization(rust_localize)' in dual
    assert "executable='mower_localize'" in dual
    assert 'condition=_rust_localize_binary(rust_localize, rust_daemon)' in dual
    # a bare __node:= would rename all three nodes in the process
    rust_block = dual.split("executable='mower_localize',", 1)[1].split(
        '),\n        ]', 1)[0]
    assert 'name=' not in rust_block
    assert 'respawn=True' in rust_block
    # the same params file and the same topic wiring as the C++ nodes, the
    # wiring scoped per node name because one process holds all three
    assert 'parameters=[rl_params_file],' in rust_block
    assert '_node_scoped_remap_args(remappings)' in rust_block
    assert dual.count("remappings=remappings['") == 3
    assert "('gps/fix', gps_fix_topic)," in dual

    # the switch reaches dual_ekf_navsat.launch.py from the production entry
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mower = MOWER_LAUNCH.read_text(encoding='utf-8')
    assert "('localize', 'rust_localize')," in robot
    assert "'rust_localize': rust_localize," in robot
    assert "'rust_localize',\n            default_value='true'," in robot
    assert "'rust_localize': LaunchConfiguration('rust_localize')," in mower

    # one module, three nodes (mower_rsd's tests/module_table.rs); under
    # mower_rsd they read the C++ nodes' own params file (no copy in
    # mower_rsd.yaml to drift) and get the same wiring. The one section
    # mower_rsd.yaml has is ekf_filter_node_map's slow copy, keys
    # robot_localization does not have.
    params = MOWER_RSD_PARAMS.read_text(encoding='utf-8')
    rl_params = (
        SRC_DIR / 'mower_nav2/config/dual_ekf_navsat_params.yaml'
    ).read_text(encoding='utf-8')
    import yaml
    rsd = yaml.safe_load(params)
    for node in ('ekf_filter_node_odom', 'ekf_filter_node_map',
                 'navsat_transform'):
        assert f'\n{node}:\n  ros__parameters:' in rl_params, node
        own = rsd.get(node, {}).get('ros__parameters', {})
        assert set(own) <= {'odometry_slow_topic', 'odometry_slow_rate_hz'}, \
            (node, sorted(own))
        for key in own:
            assert f'\n    {key}:' not in rl_params, (node, key)
    assert '\nekf_filter_node_odom:\n' not in params
    assert '\nnavsat_transform:\n' not in params
    assert "'dual_ekf_navsat_params.yaml'" in robot
    # one override reaches both stacks: robot.launch.py -> mower.launch.py ->
    # dual_ekf_navsat.launch.py for the C++ nodes and mower_localize, and
    # robot.launch.py's own mower_rsd, after mower_rsd.yaml so it wins there
    assert "rl_params_file = LaunchConfiguration('localize_params_file')" in dual
    assert dual.count('parameters=[rl_params_file') == 4
    assert "'localize_params_file',\n" in dual
    assert "'localize_params_file': LaunchConfiguration(" in mower
    assert 'declare_localize_params_file,' in mower
    assert "'localize_params_file': localize_params_file," in robot
    assert "'localize_params_file',\n" in robot
    assert (
        "parameters.append(\n"
        "            LaunchConfiguration('localize_params_file').perform(context))"
    ) in robot
    assert 'localize_params_file:=' in params
    assert "'localize': [" in robot
    assert "'navsat_transform:gps/fix:=' + LaunchConfiguration(" in robot
    for rule in ('ekf_filter_node_odom:odometry/filtered:=odometry/local',
                 'ekf_filter_node_map:odometry/filtered:=odometry/global',
                 'navsat_transform:odometry/filtered:=odometry/global',
                 'navsat_transform:imu:=imu/data'):
        assert f"'{rule}'," in robot, rule
    # navsat stays at 30 Hz: the health gate wants /odometry/gps within 0.30 s
    # and the datum waits 3 s for the map EKF's heading to settle
    navsat = rl_params.split('navsat_transform:', 1)[1]
    assert 'frequency: 30.0' in navsat
    assert 'delay: 3.0' in navsat

    # built and shipped like the other mower_rs binaries
    cmake = MOWER_RS_CMAKE.read_text(encoding='utf-8')
    cargo = MOWER_RS_CARGO.read_text(encoding='utf-8')
    assert 'mower_localize ' in cmake or 'mower_localize)' in cmake
    assert '"crates/mower_localize"' in cargo
    assert '"crates/mower_localize_core"' in cargo

    # compose switch and its documented default (on since 2026-10-08)
    deploy = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    assert 'rust_localize:=${RUST_LOCALIZE:-true}' in deploy
    env_example = (SRC_DIR.parent / 'deploy/.env.example').read_text(
        encoding='utf-8'
    )
    assert 'RUST_LOCALIZE=true' in env_example
    # The filter's settings come from the node's parameters, not compiled in
    # (mower_localize's on-bus tests in src/ekf.rs; mower_localize_core's
    # tests/yaml_config.rs checks the yaml), and its timer skips missed ticks
    # (mower_localize/tests/source_rules.rs).


def test_rust_velocity_guard_keeps_the_same_rules_and_wiring():
    """rust_guards:=true swaps in mower_rs velocity_command_guard: same
    remappings and session requirement. Its limits, stop barriers and clocks
    are velocity_command_guard's own tests (src/core.rs, src/lib.rs,
    tests/source_rules.rs)."""
    mux_launch = TWIST_MUX_LAUNCH.read_text(encoding='utf-8')
    assert "package='mower_rs'" in mux_launch
    assert "condition=_rust_binary(rust_guards, rust_daemon)" in mux_launch
    assert "condition=UnlessCondition(rust_guards)" in mux_launch
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')
    deploy_compose = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    for switch in ('rust_status', 'rust_adapter', 'rust_record', 'rust_nav', 'rust_battery', 'rust_pid_autotune', 'rust_map', 'rust_agent', 'rust_guards', 'rust_imu', 'rust_bridge'):
        assert f"'{switch}': {switch}," in robot_launch
        assert f"{switch}:=${{{switch.upper()}:-false}}" in deploy_compose
    # the coverage planner is Rust-only: no switch left anywhere
    for text in (robot_launch, deploy_compose, MISSION_LAUNCH.read_text(encoding='utf-8'),
                 ENV_EXAMPLE.read_text(encoding='utf-8')):
        assert 'rust_coverage' not in text.lower()
    assert ("parameters=[{'require_command_session': True,\n"
            "                     'max_input_age_s': MANUAL_MAX_INPUT_AGE_S}]") in mux_launch


def test_rust_nav_server_keeps_the_same_safety_rules_and_wiring():
    """rust_nav:=true swaps in mower_rs mower_nav for nav_action_server under
    the same node name. Its actions, services, admission/health rules,
    immutable safety defaults, bounded Nav2 waits and the 20 Hz fail-safe
    heartbeat are mower_nav's own tests (src/state.rs, src/geometry.rs,
    tests/bus.rs, tests/source_rules.rs)."""
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_nav'" in mission_launch
    assert "condition=_rust_binary(rust_nav, rust_daemon)" in mission_launch
    assert "condition=UnlessCondition(rust_nav)" in mission_launch
    assert mission_launch.count("name='nav_action_server'") == 1


def test_rust_pid_autotune_keeps_the_same_maths_and_wiring():
    """rust_pid_autotune:=true swaps in mower_rs mower_pid_autotune for
    pid_autotune_node under the same node name. The parameters, plausibility
    limits and flag bits both must have are the crate's
    tests/rclpy_parity.json: the Python side is checked here, the Rust side
    by the crate (src/lib.rs, tests/source_rules.rs), which also has its
    service, status states and immutable parameters."""
    py_node = PID_AUTOTUNE_NODE.read_text(encoding='utf-8')
    py_tuning = PID_TUNING.read_text(encoding='utf-8')
    parity = json.loads(PID_AUTOTUNE_PARITY.read_text(encoding='utf-8'))
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_pid_autotune'" in mission_launch
    assert "condition=_rust_binary(rust_pid_autotune, rust_daemon)" in mission_launch
    assert "condition=UnlessCondition(rust_pid_autotune)" in mission_launch
    assert mission_launch.count("name='pid_autotune'") == 2
    # every declared parameter with its default, in order and of its type
    declared = [(name, ast.literal_eval(default))
                for name, default in re.findall(r"^\s+p\('(\w+)', ([^)]+)\)", py_node, re.M)]

    def typed(parameters):
        return [(name, type(value).__name__, value) for name, value in parameters]
    assert declared and typed(declared) == typed(parity['parameters'])
    # plausibility limits and flag bits
    for const, value in parity['tuning_limits'].items():
        py_value = ast.literal_eval(
            re.search(rf'^{const} = ([^#\n]+)', py_tuning, re.M).group(1).strip())
        assert py_value == (tuple(value) if isinstance(value, list) else value), const
    for name, value in parity['telemetry_flags'].items():
        assert int(re.search(rf'^{name} = (\S+)', py_node, re.M).group(1), 0) == value, name


def test_rust_map_manager_keeps_the_same_safety_rules_and_wiring():
    """rust_map:=true swaps in mower_rs mower_map for map_manage_node under
    the same node name, with the same 0.75 m safety floor and the same
    user-facing messages (the crate's tests/rclpy_parity.json): the Python
    side is checked here, the Rust side by the crate (tests/bus.rs,
    tests/source_rules.rs), which also has its services, latched topics and
    fail-closed rules."""
    py_node = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_map'" in mission_launch
    assert "condition=_rust_binary(rust_map, rust_daemon)" in mission_launch
    assert "condition=UnlessCondition(rust_map)" in mission_launch
    assert mission_launch.count("name='map_manage'") == 2
    assert 'MIN_SAFE_INFLATE_RADIUS_M = 0.75' in py_node
    # every Chinese user-facing message without a format field
    messages = [text for text in re.findall(r"'([^'\n]*[\u4e00-\u9fff][^'\n]*)'", py_node)
                if '{' not in text and not text.endswith(': ')]
    parity = json.loads(MAP_PARITY.read_text(encoding='utf-8'))
    assert messages and list(dict.fromkeys(messages)) == parity['user_messages']


def test_mower_coverage_keeps_the_same_safety_rules_and_wiring():
    """mower_rs mower_coverage is the only coverage planner (the rclpy
    coverage_node is gone): it always runs, as its own binary or as the
    daemon's 'coverage' module, under the same node name. The services,
    action, topics, parameter defaults, dispatch deadlines, retry cadence and
    user-facing messages the Python node had are mower_coverage's own tests
    (tests/bus.rs, tests/source_rules.rs; the defaults and messages frozen
    from dff480b:src/mower_mission/mower_mission/coverage_node.py), the
    daemon module's switch mower_rsd's tests/module_table.rs."""
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')
    # the Python planner and its entry points are gone
    assert not (SRC_DIR / 'mower_mission/mower_mission/coverage_node.py').exists()
    assert 'mower_mission.coverage_node' not in MISSION_SETUP.read_text(encoding='utf-8')
    assert "executable='coverage_node'" not in mission_launch
    # one Node, off only while mower_rsd hosts it
    assert "executable='mower_coverage'" in mission_launch
    assert "condition=_rust_only_binary(rust_daemon)" in mission_launch
    assert mission_launch.count("name='boustrophedon_coverage'") == 1
    assert "('coverage', None)," in robot_launch
    # the sim graphs start the same binary under its own default node name
    for launch in (SYSTEM_TEST_LAUNCH, SMALL_TEST_LAUNCH):
        text = launch.read_text(encoding='utf-8')
        assert "'ros2', 'run', 'mower_rs', 'mower_coverage'" in text, launch.name
        assert "'coverage_node'" not in text, launch.name
        assert 'coverage_backend' not in text, launch.name
    assert "['zigzag_angle_deg:=', zigzag_angle_deg]" in SYSTEM_TEST_LAUNCH.read_text(
        encoding='utf-8')


def test_coverage_binary_and_daemon_module_are_exact_complements():
    """The coverage planner has no switch, so exactly one copy must run for
    every rust_daemon value: robot.launch.py's _truthy accepts yes/on and
    surrounding blanks, and a plain UnlessCondition would abort the whole
    launch on those."""
    mission_src = MISSION_LAUNCH.read_text(encoding='utf-8')
    robot_src = ROBOT_LAUNCH.read_text(encoding='utf-8')
    assert 'UnlessCondition(rust_daemon)' not in mission_src
    # ROS-free half (runs in the python-tests job): both files accept the
    # same spellings, and the helper strips blanks like _truthy does.
    mission_true = ast.literal_eval(
        re.search(r'^_TRUE = "(.*)"$', mission_src, re.M).group(1))
    robot_true = ast.literal_eval(re.search(
        r"return value\.strip\(\)\.lower\(\) in (\{[^}]*\})",
        robot_src).group(1))
    assert set(mission_true) == robot_true
    helper = mission_src.split('def _rust_only_binary(', 1)[1].split(
        '\ndef ', 1)[0]
    assert '''"'.strip().lower() not in ", _TRUE''' in helper
    # the behavioural half needs ROS launch
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    mission = _load_launch_module(MISSION_LAUNCH)
    robot = _load_launch_module(ROBOT_LAUNCH)
    assert ('coverage', None) in robot._DAEMON_MODULES
    for value in ('true', 'True', ' on', 'ON', 'yes', '1',
                  'false', 'FALSE', '0', 'off', 'no', 'maybe', ''):
        context = LaunchContext()
        daemon_hosts_it = robot._truthy(value)
        assert mission._rust_only_binary(value).evaluate(context) is (
            not daemon_hosts_it), repr(value)


def test_rust_agent_keeps_the_same_backend_protocol_and_wiring():
    """rust_agent:=true swaps in mower_rs mower_agent for the Python fleet
    agent, with the same launch arguments. The API version and the timing
    constants both have are the crate's tests/rclpy_parity.json: the Python
    side is checked here, the Rust side by the crate (src/lib.rs), which also
    has the endpoints, relay framing, control messages and log lines
    (tests/source_rules.rs)."""
    py = AGENT_PY.read_text(encoding='utf-8')
    launch = ROSBRIDGE_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_agent'" in launch and launch.count("name='mower_agent'") == 2
    assert "condition=_rust_binary(rust_agent, rust_daemon)" in launch and "condition=UnlessCondition(rust_agent)" in launch
    assert launch.count("'--rosbridge', 'ws://127.0.0.1:9091'") == 2
    parity = json.loads(AGENT_PARITY.read_text(encoding='utf-8'))
    api = re.search(r'^ROBOT_API_VERSION = (\d+)', VERSION_PY.read_text(encoding='utf-8'), re.M).group(1)
    assert int(api) == parity['ROBOT_API_VERSION']
    for name, value in parity['constants'].items():
        assert f'{name} = {value}' in py, name


def test_rust_battery_node_keeps_the_same_model_and_wiring():
    """rust_battery:=true swaps in mower_rs mower_battery for
    battery_state_node: same node name and launch parameters, and the OCV
    table of the crate's tests/rclpy_parity.json (the Python side is checked
    here, the Rust side by the crate's src/lib.rs). Its topics, defaults and
    BatteryState semantics are mower_battery's own tests (tests/bus.rs,
    src/lib.rs, tests/source_rules.rs)."""
    python_estimator = BATTERY_ESTIMATOR.read_text(encoding='utf-8')
    mission_launch = MISSION_LAUNCH.read_text(encoding='utf-8')
    assert "executable='mower_battery'" in mission_launch
    assert "condition=_rust_binary(rust_battery, rust_daemon)" in mission_launch
    assert "condition=UnlessCondition(rust_battery)" in mission_launch
    assert mission_launch.count("name='battery_state'") == 2
    assert mission_launch.count("'charger_present_min_v': 25.0") == 2
    assert mission_launch.count("'meter_current_wired': False") == 2
    # the OCV table, row for row
    python_rows = [[float(volts), float(charge)] for volts, charge
                   in re.findall(r'\((\d\.\d\d), (\d\.\d\d)\)', python_estimator)]
    parity = json.loads(BATTERY_PARITY.read_text(encoding='utf-8'))
    assert python_rows and python_rows == parity['ocv_table']


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


def test_robot_stack_runs_an_idle_recorder_for_the_app():
    """The app's 錄話題 button calls /mower_recorder/start|stop: a recorder
    service waits for it (autostart off), finalizes its mcap on `docker
    stop`, never flashes the STM32, and is the only owner of
    /mower_recorder/* -- lawan_node keeps record:=false and the
    data-collection helper takes the recorder down for its own run."""
    source = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    recorder = source.split('\n  recorder:\n', 1)[1]
    for line in ('- mower_recorder', '- record.launch.py', '- autostart:=false',
                 '- output_root:=/home/mower/.mower/bags',
                 'stop_signal: SIGINT', 'restart: unless-stopped',
                 '- MOWER_FIRMWARE_SYNC=0', 'network_mode: host', 'ipc: host',
                 '- ${MOWER_STATE_DIR:-/home/cat/.mower}:/home/mower/.mower'):
        assert line in recorder, line
    assert '/dev:/dev' not in recorder
    assert 'record:=false' in source.split('  mediamtx:', 1)[0]
    # front camera: MediaMTX records into <run>/video/, which it must see at
    # the same path as the recorder does
    assert '- video_api_url:=http://127.0.0.1:9997' in recorder
    mediamtx = source.split('\n  mediamtx:\n', 1)[1].split('\n  recorder:\n', 1)[0]
    assert '- ${MOWER_STATE_DIR:-/home/cat/.mower}/bags:/home/mower/.mower/bags' in mediamtx

    helper = DATA_COLLECTION_HELPER.read_text(encoding='utf-8')
    start = helper.split('\n  start)\n', 1)[1].split('\n    ;;\n', 1)[0]
    stop = helper.split('\n  stop)\n', 1)[1].split('\n    ;;\n', 1)[0]
    assert start.index('stack stop recorder') < start.index('compose up -d')
    assert 'stack up -d recorder' in stop
    assert stop.index('compose down') < stop.index('stack up -d recorder')

    install = DEPLOY_INSTALL.read_text(encoding='utf-8')
    assert 'host/mower-data-collection.sh" /opt/mower/host/' in install
    assert 'docker-compose.data-collection.yaml" /opt/mower/' in install


def test_manual_guard_allows_the_relay_round_trip_and_nothing_else_does():
    """App commands carry the robot's own clock, so the manual guard's age
    limit bounds phone <-> relay <-> robot. 0.25 s dropped every command over
    the Cloudflare relay (video unaffected, so the robot looked fine): the
    manual guard -- Python, Rust binary and mower_rsd -- allows 0.8 s, the
    app's _manualCommandClockTimeout; the final guard keeps 0.25 s."""
    mux_launch = TWIST_MUX_LAUNCH.read_text(encoding='utf-8')
    assert 'MANUAL_MAX_INPUT_AGE_S = 0.8\n' in mux_launch
    manual = mux_launch.split('manual_velocity_guard = Node(', 1)[1].split('\n    )\n', 1)[0]
    manual_rs = mux_launch.split('manual_velocity_guard_rs = Node(', 1)[1].split('\n    )\n', 1)[0]
    for block in (manual, manual_rs):
        assert "'max_input_age_s': MANUAL_MAX_INPUT_AGE_S" in block
    for name in ('velocity_guard = Node(', 'velocity_guard_rs = Node('):
        final = mux_launch.split('\n    ' + name, 1)[1].split('\n    )\n', 1)[0]
        assert 'max_input_age_s' not in final, name

    rsd = MOWER_RSD_PARAMS.read_text(encoding='utf-8')
    manual_rsd = rsd.split('\nmanual_velocity_guard:\n', 1)[1].split('\nvelocity_command_guard:\n', 1)[0]
    assert 'max_input_age_s: 0.8' in manual_rsd
    final_rsd = re.split(r'\n\S', rsd.split('\nvelocity_command_guard:\n', 1)[1], maxsplit=1)[0]
    assert 'max_input_age_s' not in final_rsd


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
    # the mower_rs driver (rust_imu:=true) keeps the same fail-closed rules:
    # mower_imu's serial_rules (src/lib.rs) and tests/source_rules.rs


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
    assert 'condition=_rust_binary(enable_gps' in block
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
    binaries = re.search(r'^set\(MOWER_RS_BINARIES (.*)\)$', cmake, re.M)
    assert binaries is not None
    assert 'mower_gps' in binaries.group(1).split()
    assert 'mower_rsd' in binaries.group(1).split()
    cargo = MOWER_RS_CARGO.read_text(encoding='utf-8')
    assert '"crates/mower_ubx"' in cargo and '"crates/mower_gps"' in cargo

    deploy = DEPLOY_COMPOSE.read_text(encoding='utf-8')
    lawan = deploy.split('  mediamtx:', 1)[0]
    assert 'enable_gps:=${GPS:-true}' in lawan
    assert 'profiles:' not in deploy
    assert 'mower_path_planning-gps' not in deploy
    assert 'ublox' not in deploy

    dockerfile = DOCKERFILE.read_text(encoding='utf-8')
    assert 'gps_runtime' not in dockerfile
    workflow = BUILD_WORKFLOW.read_text(encoding='utf-8')
    assert 'gps_runtime' not in workflow
    assert 'image_suffix: -gps' not in workflow
    assert 'install/mower_rs/lib/mower_rs/mower_gps' in workflow


def test_containers_see_the_live_dev_tree_not_static_device_nodes():
    """USB sensors re-enumerate (hub resets, re-plugs); a `devices:` entry
    is a copy of one major:minor made at create time in a container without
    udev, so it only kept working while the kernel handed back the same tty
    number. The host /dev is bind-mounted and access is limited by device
    cgroup rules instead; nothing must be plugged in for the stack to start.
    Verified on the robot 2026-09-20 (symlink gone/back live, driver exit
    0.46 s, restart on the same container)."""
    for compose in (LOCAL_COMPOSE, DEPLOY_COMPOSE):
        source = compose.read_text(encoding='utf-8')
        lawan = source.split('  mediamtx:', 1)[0]
        assert not re.search(r'^\s+devices:\s*$', lawan, re.M), compose
        assert re.search(r'^\s+- /dev:/dev\s*$', lawan, re.M), compose
        rules = lawan.split('device_cgroup_rules:', 1)[1].split('volumes:', 1)[0]
        for rule in ("'c 4:67 rmw'", "'c 188:* rmw'", "'c 166:* rmw'"):
            assert rule in rules, (compose, rule)
        assert 'privileged' not in source, compose
    udev = UDEV_RULES.read_text(encoding='utf-8')
    for name in ('stmcom', 'imu_usb', 'gps_rtk'):
        assert f'SYMLINK+="{name}"' in udev, name
    assert 'ATTR{idProduct}=="0610", TEST=="power/control", ATTR{power/control}="on"' in udev


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


def test_rust_base_and_ros2_control_are_mutually_exclusive():
    """rust_base:=true replaces the whole ros2_control chain -- the
    controller_manager process, mower_hardware::MowerSystem, diff_controller,
    joint_state_broadcaster and their two spawners -- with one mower_rs node.
    They open the same serial port, so exactly one of them may ever start;
    robot_state_publisher is *not* replaced and still needs /joint_states."""
    mower_launch = MOWER_LAUNCH.read_text(encoding='utf-8')
    robot_launch = ROBOT_LAUNCH.read_text(encoding='utf-8')

    # the include that starts ros2_control_node + both spawners
    assert 'controller_test.launch.py' in mower_launch
    controller_block = mower_launch.split('controller_test.launch.py', 1)[1]
    assert 'condition=_rust_off(rust_base)' in controller_block.split(
        'mower_base_node', 1)[0]
    # ... and the node that replaces it, off whenever the daemon runs it
    assert "executable='mower_base'" in mower_launch
    assert ("condition=_rust_binary(rust_base, LaunchConfiguration('rust_daemon'))"
            in mower_launch)
    assert 'respawn_delay=2.0' in mower_launch
    # both are in the launch description, so the conditions are what decides
    for item in ('mower_controller_launch,', 'mower_base_node,'):
        assert item in mower_launch
    # robot_state_publisher is untouched by the switch (only E4's
    # rust_static_tf replaces it)
    rsp = mower_launch.split('robot_state_publisher = Node(', 1)[1].split(
        '\n    )\n', 1)[0]
    assert 'rust_base' not in rsp
    assert ("condition=_unless_in_daemon(LaunchConfiguration('rust_static_tf'),"
            in rsp)

    # the switch itself: declared on (since 2026-10-08), threaded down, and
    # a daemon module
    argument = re.search(
        r"DeclareLaunchArgument\(\s*'rust_base'(.*?)\n\s*\),",
        robot_launch,
        flags=re.DOTALL,
    )
    assert argument is not None
    assert re.search(r"default_value='true'", argument.group(1))
    assert "'rust_base': rust_base," in robot_launch
    assert "('base', 'rust_base')," in robot_launch

    # one module of mower_rsd (its tests/module_table.rs), one parameter
    # section, one installed binary
    params = MOWER_RSD_PARAMS.read_text(encoding='utf-8')
    assert '\nmower_base:\n  ros__parameters:' in params
    assert 'mower_base' in MOWER_RS_CMAKE.read_text(encoding='utf-8').split(
        'set(MOWER_RS_BINARIES', 1)[1].split(')', 1)[0]
    assert '"crates/mower_base"' in MOWER_RS_CARGO.read_text(encoding='utf-8')

    # compose switch, default on; RUST_BASE=false in .env is the roll-back
    assert 'rust_base:=${RUST_BASE:-true}' in DEPLOY_COMPOSE.read_text(
        encoding='utf-8')
    assert 'RUST_BASE=true' in ENV_EXAMPLE.read_text(encoding='utf-8')


def _load_launch_module(path):
    import importlib.util
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rust_base_and_localize_sides_are_exact_complements_for_any_value():
    """RUST_BASE=on / RUST_LOCALIZE=yes: a plain UnlessCondition accepts only
    true/false/1/0 and aborts the whole launch (restart: unless-stopped then
    crash-loops the stack, bridge included), while the Rust side and
    robot.launch.py's module set accept yes/on too. Every value has to start
    exactly one side, and the same side everywhere."""
    assert 'UnlessCondition(rust_base)' not in MOWER_LAUNCH.read_text(
        encoding='utf-8')
    assert 'UnlessCondition(rust_localize)' not in DUAL_EKF_LAUNCH.read_text(
        encoding='utf-8')
    pytest.importorskip('launch')
    pytest.importorskip('xacro')
    pytest.importorskip('ament_index_python')
    from launch import LaunchContext

    mower = _load_launch_module(MOWER_LAUNCH)
    dual = _load_launch_module(DUAL_EKF_LAUNCH)
    robot = _load_launch_module(ROBOT_LAUNCH)
    for value in ('true', 'True', ' on', 'ON', 'yes', '1',
                  'false', 'FALSE', '0', 'off', 'no', 'maybe', ''):
        context = LaunchContext()
        rust = robot._truthy(value)
        assert mower._rust_binary(value, 'false').evaluate(context) is rust
        assert mower._rust_off(value).evaluate(context) is not rust
        assert dual._rust_localize_binary(value, 'false').evaluate(
            context) is rust
        assert dual._cpp_localization(value).evaluate(context) is not rust
        # with the daemon on, the separate binary stays down either way
        assert mower._rust_binary(value, 'on').evaluate(context) is False


def test_rust_base_reproduces_the_launched_controller_parameters():
    """The Rust driver's built-in defaults are the production diff_controller
    yaml and the rest of what mower_hardware read; mower_base's own tests
    hold its side (tests/controller_yaml.rs, tests/rear_light.rs,
    tests/source_rules.rs). Here, mower_hardware's side of the rear light:
    the recorders' topic and the 6 s keep-alive per source that mower_base
    keeps (E9-07)."""
    hw = (SRC_DIR / 'mower_hardware/include/mower_hardware/mower_system.hpp').read_text(encoding='utf-8')
    assert 'rear_light_topic_ = "/mower_base/rear_light"' in hw and 'rear_light_timeout_s_ = 6.0' in hw


def test_rust_source_modules_publish_the_slow_copies_their_throttle_made():
    """/odom_slow and /odometry/global_slow come from a topic_tools throttle
    while the C++ source runs, and from mower_base / mower_localize once they
    publish the source topic. Each throttle is held down by exactly the switch
    that moves its source into mower_rs, so every copy always has one
    publisher, and with both switches false (the default) mission.launch.py
    starts the same nodes as before."""
    mission = MISSION_LAUNCH.read_text(encoding='utf-8')
    for name, switch in (('odom_throttle', 'rust_base'),
                         ('global_odom_throttle', 'rust_localize')):
        block = mission.split(f"name='{name}',", 1)[1].split('\n    )\n', 1)[0]
        assert f'condition=UnlessCondition({switch}),' in block, name
        assert f"'{switch}',\n        default_value='false'," in mission, switch
        assert f'declare_{switch},' in mission, switch
    # what the modules reproduce: the throttles' own arguments
    assert "arguments=['messages', '/odom', '5.0', '/odom_slow']," in mission
    assert "'messages', '/odometry/global', '5.0', '/odometry/global_slow'," \
        in mission

    # robot.launch.py threads both switches into mission.launch.py too, not
    # only into mower.launch.py, which starts the modules
    robot = ROBOT_LAUNCH.read_text(encoding='utf-8')
    mission_include = robot.split("'mission.launch.py')", 1)[1].split(
        '}.items()', 1)[0]
    assert "'rust_base': rust_base," in mission_include
    assert "'rust_localize': rust_localize," in mission_include

    # topic and rate are parameters whose defaults are the throttles' values
    # (mower_base's tests/source_rules.rs, mower_localize's src/ekf.rs),
    # written out in the parameter file both launch paths of mower_base and
    # the daemon read
    params = MOWER_RSD_PARAMS.read_text(encoding='utf-8')
    base = params.split('\nmower_base:\n', 1)[1].split('\n\n', 1)[0]
    assert 'odom_slow_topic: /odom_slow' in base
    assert 'odom_slow_rate_hz: 5.0' in base
    ekf_map = params.split('\nekf_filter_node_map:\n', 1)[1].split(
        '\nnavsat_transform:', 1)[0]
    assert 'odometry_slow_topic: /odometry/global_slow' in ekf_map
    assert 'odometry_slow_rate_hz: 5.0' in ekf_map

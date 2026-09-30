"""Source-level regression checks for snapshot-topic QoS compatibility."""

import ast
import re
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
# The only coverage planner (the rclpy coverage_node.py was removed).
COVERAGE_RS = (
    PACKAGE_DIR.parent / 'mower_rs/crates/mower_coverage/src/lib.rs'
)
MAP_MANAGE_NODE = PACKAGE_DIR / 'mower_mission/map_manage_node.py'
FLUTTER_ADAPTER = (
    PACKAGE_DIR / 'mower_mission/adapters/flutter_adapter_node.py'
)
ROSBRIDGE_CONFIG = (
    PACKAGE_DIR.parent / 'mower_bringup/config/rosbridge_params.yaml'
)
NAV_SERVER = PACKAGE_DIR / 'mower_mission/navigation/nav_action_server.py'
NAV_GUARD = PACKAGE_DIR / 'mower_mission/navigation_guard.py'
PATH_RECORDER = PACKAGE_DIR / 'mower_mission/path_record_node.py'
MISSION_LOCK_SRV = (
    PACKAGE_DIR.parent / 'mower_interface/srv/MissionOperationLock.srv'
)
DISPATCH_CONFIRM_SRV = (
    PACKAGE_DIR.parent
    / 'mower_interface/srv/ConfirmNavigationDispatch.srv'
)
WAYPOINT_ACTION = PACKAGE_DIR.parent / 'mower_interface/action/Waypoint.action'


def _publisher_qos_for(topic: str) -> str:
    """The QoS expression the Rust coverage node creates `topic` with."""
    match = re.search(
        r'create_publisher::<[\w:]+>\(\s*"' + re.escape(topic)
        + r'",\s*([^?]*?)\)\?',
        COVERAGE_RS.read_text(encoding='utf-8'),
    )
    assert match is not None, f'publisher for {topic} not found'
    return match.group(1).strip()


def _rust_fn(source: str, name: str) -> str:
    """Body of the Rust fn `name` (up to the next item at its level)."""
    start = re.search(
        rf'^\s*(?:pub )?(?:async )?fn {name}\b', source, flags=re.MULTILINE
    )
    assert start is not None, f'fn {name} not found'
    rest = source[start.end():]
    first_line_end = rest.index('\n') + 1
    end = re.compile(
        r'^(?:    (?:///|#\[|(?:pub )?(?:async )?fn )|\S)',
        flags=re.MULTILINE,
    ).search(rest, first_line_end)
    return rest[:end.start()] if end else rest


def test_coverage_snapshot_publishers_offer_transient_local_qos():
    """Every current-plan/map snapshot must match late-joining consumers."""
    source = COVERAGE_RS.read_text(encoding='utf-8')
    assert (
        'fn latched() -> QosProfile {\n'
        '    QosProfile::default().keep_last(1).reliable().transient_local()'
    ) in source
    for topic in (
        '/coverage_path',
        '/coverage_path_markers',
        '/coverage_invalid_segments',
        '/coverage_connectors',
        '/free_space_inflated',
        '/risk_map_inflated',
    ):
        assert _publisher_qos_for(topic) == 'latched()', topic


def test_fixed_map_datum_fallback_is_opt_in():
    """A missing GPS fix must not silently publish a plausible site datum."""
    source = FLUTTER_ADAPTER.read_text(encoding='utf-8')
    assert "declare_parameter('enable_map_datum_fallback', False)" in source
    assert "get_parameter('enable_map_datum_fallback')" in source


def test_robot_pose_relay_and_health_gate_reject_stale_source_data():
    adapter = FLUTTER_ADAPTER.read_text(encoding='utf-8')
    server = NAV_SERVER.read_text(encoding='utf-8')
    assert "declare_parameter('robot_pose_max_age_s', 1.0)" in adapter
    assert 'odom.header.frame_id != source_frame' in adapter
    assert 'odom.child_frame_id != child_frame' in adapter
    assert 'not -0.5 <= age_s <= max_age_s' in adapter
    # The relay must read the EKF map pose, never the 77 Hz /tf stream.
    assert 'from tf2_ros import' not in adapter
    assert 'from tf2_ros import' not in PATH_RECORDER.read_text(encoding='utf-8')
    assert "msg.header.frame_id != 'map'" in server
    assert '_source_stamp_block_reason(' in server
    assert 'not -90.0 <= lat <= 90.0' in server
    assert 'not -180.0 <= lon <= 180.0' in server


def test_rosbridge_is_loopback_only_by_default():
    source = ROSBRIDGE_CONFIG.read_text(encoding='utf-8')
    assert 'address: "127.0.0.1"' in source
    assert ("topics_pub_glob: \"['/app_joy_cmd', '/mower_recorder/command', "
            "'/mower_base/servo_command', '/mower_base/blade_command']\"") in source
    assert "'/manual_command_clock'" in source
    assert "'/joy_cmd'" not in source


def test_navigation_and_mutation_admission_share_a_central_lock():
    server = NAV_SERVER.read_text(encoding='utf-8')
    guard = NAV_GUARD.read_text(encoding='utf-8')
    interface = MISSION_LOCK_SRV.read_text(encoding='utf-8')
    assert "'/mission_operation_lock'" in server
    assert 'def _navigation_admission_block_reason_locked' in server
    assert 'if self._mutation_owner is not None' in server
    assert "'/mission_operation_lock'" in guard
    assert 'string owner' in interface
    assert 'bool acquire' in interface
    assert 'self._mutation_callback_group = MutuallyExclusiveCallbackGroup()' in server
    assert 'callback_group=self._mutation_callback_group' in server
    assert "'/navigation_coordinator_lock'" in server
    assert 'autonomy_authorized = (' in server
    assert "self._nav_state == 'running'" in server


def test_navigation_dispatch_requires_a_correlated_confirmation():
    server = NAV_SERVER.read_text(encoding='utf-8')
    coverage = COVERAGE_RS.read_text(encoding='utf-8')
    interface = DISPATCH_CONFIRM_SRV.read_text(encoding='utf-8')
    action = WAYPOINT_ACTION.read_text(encoding='utf-8')
    assert 'string dispatch_id' in interface
    assert 'string dispatch_id' in action
    assert 'dispatch_id != self._pending_dispatch_id' in server
    assert "self._nav_state = 'pending_confirmation'" in server
    assert 'self.track_and_cancel_goal(' in coverage
    assert 'navigation may be active' in coverage
    # the coverage node stamps every goal and confirms that same id
    assert (
        'dispatch_id: dispatch_id.clone(), zone_id, resume_segment_index };'
    ) in coverage
    assert (
        'ConfirmNavigationDispatch::Request { dispatch_id: '
        'dispatch_id.clone() }'
    ) in coverage


def test_mutation_release_failure_is_never_acknowledged_as_success():
    guard = NAV_GUARD.read_text(encoding='utf-8')
    recorder = PATH_RECORDER.read_text(encoding='utf-8')
    map_manager = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    assert 'if not self._release_mission_mutation():' in guard
    assert 'response.success = False' in guard
    assert recorder.count('res.success = release_confirmed') >= 3
    assert 'mutation_lease and not self._release_mission_mutation()' in recorder
    assert 'if not release_confirmed:' in map_manager


def test_navigation_health_pose_message_is_imported():
    """The production health gate must not crash the nav server at import."""
    tree = ast.parse(NAV_SERVER.read_text(encoding='utf-8'))
    geometry_imports = {
        alias.name
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        and node.module == 'geometry_msgs.msg'
        for alias in node.names
    }
    assert 'PoseStamped' in geometry_imports


def test_site_loading_is_proximity_limited_and_work_files_are_atomic():
    source = PATH_RECORDER.read_text(encoding='utf-8')
    assert "declare_parameter('max_site_datum_distance_m', 100.0)" in source
    assert 'datum_distance_m > max_distance_m' in source
    assert 'os.replace(tmp_path, path)' in source
    assert 'res.success = False\n            res.message = f\'部分成功储存列表' in source


def test_app_geometry_edits_advance_physical_recording_ids():
    """An App add must not let the next physical recording reuse its id."""
    tree = ast.parse(PATH_RECORDER.read_text(encoding='utf-8'))
    edit_callback = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'PathRecorder'
        for node in node.body
        if isinstance(node, ast.FunctionDef) and node.name == 'edit_zone_srv'
    )
    calls = [
        node
        for node in ast.walk(edit_callback)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == '_restore_id_counters'
    ]
    assert calls


def test_app_geometry_edit_requires_both_work_and_site_persistence():
    """Do not ACK an edit if its active named-site snapshot was not saved."""
    source = PATH_RECORDER.read_text(encoding='utf-8')
    assert 'if not self._update_active_site():' in source
    assert 'rollback_persisted = bool(save())' in source
    assert "場地檔同步失敗，變更已回復" in source


def test_active_site_manifest_is_restored_and_mutations_fail_closed():
    """A reboot must preserve or explicitly block the active-site identity."""
    source = PATH_RECORDER.read_text(encoding='utf-8')
    assert 'self._restore_startup_persistence()' in source
    assert 'site_store.read_active_site(' in source
    assert 'site_store.write_active_site(' in source
    assert 'site_store.clear_active_site(' in source
    assert 'self._active_manifest_blocked_reason' in source
    assert "請先使用 /site_op load 重新載入場地" in source


def test_recording_end_is_acknowledged_only_after_its_atomic_save():
    """A failed end-save must retain the live recorder for a safe retry."""
    tree = ast.parse(PATH_RECORDER.read_text(encoding='utf-8'))
    recorder = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'PathRecorder'
    )
    cases = (
        ('record_zone_end_srv', '_save_zone_list', 'record_zone_status'),
        ('risk_zone_end_srv', '_save_risk_zone_list', 'risk_zone_status'),
        (
            'chennal_record_end_srv',
            '_save_chennal_path_list',
            'chennal_record_status',
        ),
    )
    for method_name, save_name, status_name in cases:
        method = next(
            node
            for node in recorder.body
            if isinstance(node, ast.FunctionDef) and node.name == method_name
        )
        method_source = ast.unparse(method)
        save_guard = f'if not self.{save_name}():'
        stop_assignment = f'self.{status_name} = False'
        assert save_guard in method_source
        assert method_source.index(save_guard) < method_source.index(
            stop_assignment
        )
        assert '.markers.pop()' in method_source


def test_nav2_uses_separate_local_and_global_safety_maps():
    source = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    assert 'self.risk_map_inflated = risk_map_inflated' in source
    assert "OccupancyGrid, '/map_grid_global'" in source
    assert 'union_free_space_grids(' in source
    assert 'self._create_free_space_inflated(\n                combined_base' in source
    assert 'fuse_navigation_grid(' in source
    assert 'risk_map=self.risk_map' in source
    assert 'risk_map_inflated=self.risk_map_inflated' in source
    assert 'self.nav_global_map_pub.publish(global_map_msg)' in source
    assert "robot_pose_header.frame_id must be map" in source
    assert 'self.nav_base_map_pub.publish(free_map)' not in source


def test_image_coverage_cannot_bypass_production_clearance():
    source = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    assert (
        'zone_map.mask_map_inflated = copy.deepcopy(free_space_inflated)'
        in source
    )
    assert 'erode_free_space_grid(' in source
    assert (
        'zone_map.mask_map_inflated = copy.deepcopy(zone_map.mask_map)'
        not in source
    )


def test_channel_map_service_acknowledges_completed_publication():
    source = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    assert 'if not math.isfinite(next_inflate_radius_m)' in source
    assert "declare_parameter('chennal_width_m', 1.2)" in source
    assert 'len(marker.points) < 2' in source
    assert '拒絕不完整資料' in source
    assert "@guarded_mission_mutation('rebuild the channel map')" in source
    tree = ast.parse(source)
    sync_method = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'MapManage'
        for node in node.body
        if (
            isinstance(node, ast.FunctionDef)
            and node.name == '_create_chennal_map_sync'
        )
    )
    method_source = ast.unparse(sync_method)
    assert 'self._wait_for_future(future, timeout_sec)' in method_source
    assert 'self.chennal_map_inflated_pub.publish' in method_source
    assert 'self._build_navigation_maps' in method_source
    assert 'self._commit_navigation_maps' in method_source
    assert method_source.index('self._build_navigation_maps') < (
        method_source.index('self.chennal_map = chennal_map')
    )
    assert 'return (True, message)' in method_source
    assert '_start_create_chennal_map_async' not in source
    assert 'math.ceil(chennal_width / resolution)' in source


def test_freespace_rebuild_and_image_restore_are_transactional():
    source = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    methods = {
        node.name: ast.unparse(node)
        for class_node in tree.body
        if isinstance(class_node, ast.ClassDef)
        and class_node.name == 'MapManage'
        for node in class_node.body
        if isinstance(node, ast.FunctionDef)
    }
    builder = methods['_create_zone_maps_and_freespace']
    rebuild = methods['_create_free_space_sync']
    restore = methods['restore_free_space_srv']
    assert 'self.zone_map_list = []' not in builder
    assert 'return (masked_map, zone_maps)' in builder
    assert 'self.zone_map_list = zone_maps' in rebuild
    assert 'self.collected_free_space = overall_freespace_map' in rebuild
    assert '沒有採集的自由空間可還原；保留目前圖片任務' in restore
    assert restore.index('self._build_navigation_maps') < restore.index(
        'self.zone_map_list = self._free_zone_backup'
    )


def test_map_inflation_cannot_start_or_change_below_safety_envelope():
    source = MAP_MANAGE_NODE.read_text(encoding='utf-8')
    assert 'MIN_SAFE_INFLATE_RADIUS_M = 0.75' in source
    assert 'initial_inflate_radius_m < MIN_SAFE_INFLATE_RADIUS_M' in source
    assert 'next_inflate_radius_m < MIN_SAFE_INFLATE_RADIUS_M' in source


def test_cancel_tracking_shares_attempts_and_stops_before_teardown():
    """Source-level companion of src/mower_rs/tools/coverage_cancel_check.py
    (the black-box behaviour check of the same tracker, run in CI's mower_rs
    job), which replaced the deleted test_coverage_cancel_tracking.py. One correlated
    cancel attempt per dispatch id, only the current attempt may clear
    itself, no fallback after shutdown, and tracking stops before the node
    stops spinning."""
    source = COVERAGE_RS.read_text(encoding='utf-8')
    fallback = _rust_fn(source, 'request_nav2_cancel_fallback')
    finish = _rust_fn(source, 'finish_attempt')
    assert re.search(r'if tr\.shutdown \{\s*return None;', fallback)
    assert 'tr.attempts.get(dispatch_id).cloned()' in fallback
    assert 'Arc::ptr_eq(a, attempt)' in finish
    assert source.index('tr.shutdown = true;') < source.index(
        'running.store(false, Ordering::Relaxed);'
    )


def test_zone_sequence_stop_tracks_the_exact_goal_outside_the_lock():
    source = COVERAGE_RS.read_text(encoding='utf-8')
    stop = source.split(
        '"/stop_zone_sequence", QosProfile::services_default())', 1
    )[1].split('\n    }\n', 1)[0]
    cancel = _rust_fn(source, 'cancel_sequence_active_goal')
    clear = _rust_fn(source, 'clear_active_goal')
    send = _rust_fn(source, 'send_follow_path')
    sequence = _rust_fn(source, 'run_sequence')
    # stop: flag the sequence first, then cancel its exact active goal
    assert 'ctx.sequence_cancel.store(true, Ordering::SeqCst);' in stop
    assert stop.index('ctx.sequence_cancel.store(true') < stop.index(
        'ctx.cancel_sequence_active_goal('
    )
    # cancel once per goal, and track it only after the lock is released
    assert 'Some(a) if a.cancel_marked => return false,' in cancel
    assert 'a.cancel_marked = true;' in cancel
    assert (
        cancel.index('self.tracking.lock()')
        < cancel.index('};')
        < cancel.index('self.track_and_cancel_goal(')
    )
    # only the matching generation (handle, dispatch id, result) is cleared
    assert (
        'active.handle_id == handle_id && active.dispatch_id == dispatch_id '
        '&& active.slot.id == slot_id'
    ) in clear
    # the result is captured before the dispatch is confirmed
    assert send.index('handles.adopt(self, goal, result)') < send.index(
        'ConfirmNavigationDispatch::Request {'
    )
    # both zone and channel legs of a sequence are sequence-owned goals
    assert sequence.count(', true, true).await') == 2


def test_nav2_dispatch_is_bounded_and_terminal_evidence_is_strict():
    source = NAV_SERVER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    methods = {
        node.name: ast.unparse(node)
        for class_node in tree.body
        if isinstance(class_node, ast.ClassDef)
        and class_node.name == 'NavActionServer'
        for node in class_node.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    guarded = methods['_guarded_execute_callback']
    bounded = methods['_send_nav2_goal_bounded']
    terminal = methods['_task_result_from_future']
    wait = methods['_wait_for_nav_task']
    assert 'self.single_path_execute_callback' in guarded
    assert 'nav2_goal_response_timeout_s' in bounded
    assert 'send_future.add_done_callback' in bounded
    assert 'late_handle.cancel_goal_async()' in bounded
    assert 'GoalStatus.STATUS_SUCCEEDED' in terminal
    assert 'GoalStatus.STATUS_CANCELED' in terminal
    assert 'GoalStatus.STATUS_ABORTED' in terminal
    assert 'return (False, None, True)' in terminal
    assert '_feedback_distance' not in wait
    assert 'navigator.goToPose' not in methods['single_path_execute_callback']
    assert 'navigator.followPath' not in methods['single_path_execute_callback']
    assert 'navigator.getPath' not in source
    assert 'navigator.smoothPath' not in source
    assert 'navigator.followPath' not in source

    terminal_clear_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == '_mark_nav2_dispatch_terminal'
    ]
    assert terminal_clear_calls
    assert all(
        any(
            keyword.arg == 'expected_generation'
            for keyword in call.keywords
        )
        for call in terminal_clear_calls
    )
    recover = methods['_recover_nav2_result_future']
    assert 'goal_handle.get_result_async()' in recover
    assert 'self._nav2_active_generation != expected_generation' in recover


def test_production_sensor_gate_matches_rtk_and_stop_margin():
    source = NAV_SERVER.read_text(encoding='utf-8')
    assert "'max_gps_horizontal_sigma_m',\n            0.015" in source
    assert "'navigation_health_timeout_s',\n            0.30" in source
    assert "'max_imu_orientation_sigma_rad'" in source
    assert 'eigenvalue_upper_bound > max_variance' in source
    assert source.count('horizontal covariance is degenerate') == 2

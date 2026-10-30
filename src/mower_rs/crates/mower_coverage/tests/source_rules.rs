//! Rules of mower_coverage that only its source shows (E9-07, from
//! test_controller_configuration.py and test_qos_configuration.py; see
//! test_support/source_check.rs). The endpoints, the latched snapshots, the
//! parameter defaults, the dispatch and the cancel tracking are run in
//! tests/bus.rs (coverage_cancel_check.py's scenarios on the bus); what is
//! left is the order of a few steps and code no scenario reaches.

#[path = "../../../test_support/source_check.rs"]
mod source_check;

use source_check::Source;

fn lib() -> Source {
    Source::new("mower_coverage/src/lib.rs", include_str!("../src/lib.rs"))
}

/// The sim graphs (system_test / small_test launch) run the binary under its
/// own default node name, the production one.
#[test]
fn the_binary_runs_as_boustrophedon_coverage() {
    Source::new("mower_coverage/src/main.rs", include_str!("../src/main.rs")).has("module_main(\"coverage\", \"boustrophedon_coverage\"");
}

/// Every message coverage_node.py showed the app or wrote to its log (the
/// Chinese ones frozen from the removed node, in
/// test_controller_configuration.py before), and the English ones.
#[test]
fn the_user_facing_messages_are_the_python_node_s() {
    let lib = lib();
    for text in [
        "boustrophedon_coverage 初始化",
        "收到 risk_map 地圖",
        "收到 risk_map_inflated 地圖",
        "缺少 risk_map_inflated 地圖數據",
        "覆蓋路徑生成成功",
        "覆蓋路徑生成失敗",
        "沒有可用的 zone map",
        "已有任務序列執行中，請先呼叫 /stop_zone_sequence",
        "前一序列的導航終止狀態尚未確認，拒絕啟動新序列",
        "zone_ids 不可為空",
        "已立即送出目前導航目標的取消請求；正追蹤至終止狀態",
        "目前導航目標的取消已在追蹤中",
        "已送出取消請求；序列尚未派送或正在步驟間切換",
        "目前無執行中的任務序列",
        "服務呼叫超時",
        "Service 不可用",
        "Navigation action goal accepted",
        "Zone not found",
        "Zone coverage path is empty",
        "Navigation action server unavailable",
        "busy or a safety precondition failed",
        "A previous zone-sequence goal is not terminal; the new accepted goal is being canceled",
        "Zone sequence was canceled before goal dispatch",
        "Navigation action completed unsuccessfully",
        "is a legacy startup-only parameter; coverage always plans with mower_coverage_core",
        "coverage parameters cannot change while navigation or coverage generation is active/unknown",
        "action is still not terminal; retrying correlated cancel",
        "action cancellation acknowledgment timed out",
        "navigation action is now terminal",
    ] {
        lib.says(text);
    }
}

/// A sequence leg's result is waited for its leg timeout (600 s plus the
/// path at 0.05 m/s, the unit test `sequence_leg_timeout_s`'s), the
/// correlated cancel waits at most 0.25 s for its service, and its request
/// timeout is clamped to 0.5..=30 s.
#[test]
fn the_dispatch_waits_are_bounded() {
    let lib = lib();
    lib.body("send_follow_path").has("let timeout_s = sequence_leg_timeout_s(&path);");
    lib.body("request_nav2_cancel_fallback").has("self.clock.timeout(Duration::from_millis(250), self.cancel_client.wait_for_service())");
    lib.has("(0.5..=30.0).contains(&t)");
}

/// Zigzag is the boustrophedon planner, which sweeps at the configured angle
/// and searches one only when asked.
#[test]
fn zigzag_plans_with_the_boustrophedon_planner_at_the_configured_angle() {
    let lib = lib();
    lib.has("plan_boustrophedon_rs(");
    lib.lacks("generate_coverage_zigzag_path_rs");
    lib.has("let angle = if p.zigzag_auto_angle { None } else { Some(zigzag_angle_deg) };");
}

/// The result of a dispatched goal is held before its dispatch is
/// confirmed, so that a stop during the confirmation finds the goal; and a
/// goal's end clears the sequence's active goal only for the same
/// generation (handle, dispatch id and result slot), never a newer one.
#[test]
fn the_active_goal_is_held_before_confirmation_and_cleared_only_by_its_own_end() {
    let lib = lib();
    lib.body("send_follow_path").before("handles.adopt(self, goal, result)", "ConfirmNavigationDispatch::Request {");
    lib.body("clear_active_goal").has("active.handle_id == handle_id && active.dispatch_id == dispatch_id && active.slot.id == slot_id");
}

/// No generic parameter handler: the parameter services are the module's
/// own, which refuse a layout change during navigation.
#[test]
fn the_parameter_services_are_the_module_s_own() {
    lib().lacks("make_parameter_handler");
}

/// The correlated-cancel tracker, whose behaviour tools/coverage_cancel_check.py
/// checks black box in CI (scenarios A-E): every tracker of one goal shares
/// one cancel attempt per dispatch id; only the attempt that is still the
/// current one may clear itself; no fallback cancel goes out once the node
/// shuts down; and the tracking stops before the node stops spinning, so
/// nothing is sent after SIGINT. A source check: the black box sees the
/// requests, not why there is only one (from test_qos_configuration.py).
#[test]
fn the_cancel_tracker_shares_attempts_and_stops_before_the_node() {
    let lib = lib();
    let fallback = lib.body("request_nav2_cancel_fallback");
    fallback.has("if tr.shutdown { return None;");
    fallback.has("tr.attempts.get(dispatch_id).cloned()");
    lib.body("finish_attempt").has("Arc::ptr_eq(a, attempt)");
    lib.in_order(&["tr.shutdown = true;", "running.store(false, Ordering::Relaxed);"]);
}

/// /stop_zone_sequence flags the sequence first, so that no next leg is sent,
/// and then cancels its exact active goal: once per goal (a second stop finds
/// it marked, as the bus test a_second_stop_does_not_cancel_the_leg_again
/// sees), tracked only after the tracking lock is let go. Both the zone and
/// the channel legs are sent as the sequence's goals. A source check for the
/// order and the lock (from test_qos_configuration.py).
#[test]
fn a_sequence_stop_flags_first_and_cancels_its_exact_goal_once_outside_the_lock() {
    let lib = lib();
    lib.in_order(&[
        "m.resolve_service(\"/stop_zone_sequence\")",
        "ctx.sequence_cancel.store(true, Ordering::SeqCst);",
        "ctx.cancel_sequence_active_goal(",
    ]);
    lib.body("cancel_sequence_active_goal").in_order(&[
        "self.tracking.lock()",
        "Some(a) if a.cancel_marked => return false,",
        "a.cancel_marked = true;",
        "};",
        "self.track_and_cancel_goal(",
    ]);
    assert_eq!(lib.body("run_sequence").count(", true, true).await"), 2, "both legs are the sequence's goals");
}

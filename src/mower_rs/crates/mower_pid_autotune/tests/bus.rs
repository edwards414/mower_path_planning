//! pid_autotune on the bus (E2 step 6): the module's real `run_on_bus` with mower_rsd's `-r` rules, and around it on
//! another scope of the same `Bus` what mower_rsd has there: mower_base's telemetry and side channels (a fake board
//! with a first-order wheel, as `tools/fake_base.py` has it), nav's `/mission_operation_lock` and
//! `/nav_operation_active`, and the Studio's calls of `/pid_autotune`.
//!
//! - **r2r:** the module keeps a node for its parameters and logs, on ROS domain 91; no message goes through DDS. One
//!   module at a time: two nodes named `pid_autotune` must not share the process's rosout writer.
//! - **Time:** the session runs on tokio's clock and `Instant`, as before E2, so these tests wait in real time, with
//!   short holds.

use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use mower_bus::{Bus, Client, Qos, Scope, Subscriber};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::mower_interface::srv::{MissionOperationLock, PidAutotune};
use r2r::std_msgs::msg::{Bool, String as Text};
use r2r::{Parameter, ParameterValue};
use serde_json::{json, Value};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "91");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

const BEST_EFFORT_1: Qos = Qos::default().keep_last(1).best_effort();

/// The test's reader of every status document: latched, and deep enough that none is lost while the test is busy.
const STATUS_QUEUE: Qos = Qos::latched().keep_last(100);

/// One start of the module, as mower_rsd starts it; it holds the turn until it is dropped.
struct Autotune {
    m: ModuleCtx,
    shutdown: Shutdown,
    scope: Scope,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
    _turn: MutexGuard<'static, ()>,
}

impl Autotune {
    /// Short holds, so that a session takes a few seconds; the check's hold stays longer than its steady-state
    /// window (0.8 s, `tuning::step_metrics`).
    fn start(bus: &Bus) -> Autotune {
        static TURN: Mutex<()> = Mutex::new(());
        let turn = TURN.lock().unwrap_or_else(PoisonError::into_inner);
        let node = r2r::Node::create(context(), "pid_autotune", "").unwrap();
        for (key, value) in [
            ("hold_s", ParameterValue::Double(0.6)),
            ("verify_hold_s", ParameterValue::Double(2.0)),
            ("review_timeout_s", ParameterValue::Double(10.0)),
        ] {
            node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value));
        }
        let shutdown = Shutdown::new();
        let scope = bus.scope("pid_autotune");
        let m = ModuleCtx::new("pid_autotune", shutdown.clone())
            .with_bus(scope.clone())
            .with_remaps(Arc::new(mower_bridge::production_remaps()));
        let task = tokio::spawn(mower_pid_autotune::run_on_bus(node, m.clone()));
        Autotune { m, shutdown, scope, task: Some(task), _turn: turn }
    }

    async fn stop(mut self) {
        self.shutdown.trigger();
        let task = self.task.take().unwrap();
        let result = tokio::time::timeout(WAIT, task).await.expect("the module did not stop").expect("it panicked");
        result.expect("the module failed");
        self.scope.close();
    }
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

/// The scopes on `/mower_base/telemetry` that read it.
fn telemetry_readers(bus: &Bus) -> Vec<String> {
    bus.topic("/mower_base/telemetry").map(|t| t.subscribers).unwrap_or_default()
}

async fn call(client: &Client<PidAutotune::Request, PidAutotune::Response>, op: &str) -> (bool, String) {
    tokio::time::timeout(WAIT, client.wait_for_service()).await.unwrap().unwrap();
    let response = tokio::time::timeout(WAIT, client.request(&PidAutotune::Request { op: op.to_string() }).unwrap())
        .await
        .expect("no answer")
        .expect("the call failed");
    (response.success, response.message)
}

fn status(msg: &Text) -> Value {
    serde_json::from_str(&msg.data).unwrap()
}

/// What the Studio sees: every status document, in order.
fn record_status(mut sub: Subscriber<Text>) -> Arc<Mutex<Vec<(Instant, Value)>>> {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let out = seen.clone();
    tokio::spawn(async move {
        while let Some(msg) = sub.recv().await {
            out.lock().unwrap().push((Instant::now(), status(&msg)));
        }
    });
    seen
}

fn last_state(seen: &Mutex<Vec<(Instant, Value)>>) -> Option<String> {
    seen.lock().unwrap().last().map(|(_, s)| s["state"].as_str().unwrap_or_default().to_string())
}

// ---------------------------------------------------------------------------
// A fake mower_base on the bus: its telemetry at 20 Hz, its PID and override side channels
// ---------------------------------------------------------------------------

#[derive(Default)]
struct Board {
    gains: [[f64; 3]; 2],
    flags: i64,
    rx_seq: i64,
    seq: i64,
    permille: i64,
    override_until: Option<Instant>,
    rpm: f64,
    pid_requests: Vec<Value>,
    overrides: Vec<Value>,
}

struct FakeBase {
    board: Arc<Mutex<Board>>,
    leds: Arc<Mutex<Vec<Value>>>,
}

const MAX_RPM: f64 = 58.0;
const TAU_S: f64 = 0.15;

fn frame(b: &Board) -> Text {
    let target = b.permille as f64 / 1000.0 * MAX_RPM;
    let wheel = json!({"pid_output": b.permille, "measured_rpm": b.rpm, "target_rpm": target});
    let side = |g: [f64; 3]| json!({"kp": g[0], "ki": g[1], "kd": g[2]});
    let doc = json!({
        "wheel": {"seq": b.seq, "left": wheel, "right": wheel},
        "pid": {"valid": true, "flags": b.flags, "last_rx_seq": b.rx_seq, "flash_diag": 0,
                "left": side(b.gains[0]), "right": side(b.gains[1])},
        "motor": {"valid": true, "flags": 0},
        "power": {"valid": true, "state": 0},
    });
    Text { data: doc.to_string() }
}

impl FakeBase {
    fn start(test: &Scope) -> FakeBase {
        let board = Arc::new(Mutex::new(Board {
            gains: [[2.0, 0.6, 0.0], [2.0, 0.6, 0.0]],
            // CLOSED_LOOP | FLASH_VALID: a board that booted with saved gains
            flags: 0x03,
            ..Board::default()
        }));
        let telemetry = test.publisher::<Text>("/mower_base/telemetry", BEST_EFFORT_1).unwrap();
        let mut pid = test.subscribe::<Text>("/mower_base/pid_command", Qos::default().keep_last(4)).unwrap();
        let mut wheel_override = test.subscribe::<Text>("/mower_base/wheel_override", BEST_EFFORT_1).unwrap();
        let mut led = test.subscribe::<Text>("/mower_base/led_command", Qos::latched()).unwrap();
        let leds = Arc::new(Mutex::new(Vec::new()));
        {
            let board = board.clone();
            tokio::spawn(async move {
                while let Some(msg) = pid.recv().await {
                    let req: Value = serde_json::from_str(&msg.data).unwrap();
                    let mut b = board.lock().unwrap();
                    for (i, side) in ["left", "right"].iter().enumerate() {
                        b.gains[i] = ["kp", "ki", "kd"].map(|k| req[side][k].as_f64().unwrap());
                    }
                    let closed = req["closed_loop"].as_i64() == Some(1);
                    let mut flags = (b.flags & 0x06) | 0x08 | if closed { 0x01 } else { 0 };
                    if req["persist"].as_i64() == Some(1) {
                        flags |= 0x06;
                    }
                    b.flags = flags;
                    b.rx_seq = (b.rx_seq + 1) % 256;
                    b.pid_requests.push(req);
                }
            });
        }
        {
            let board = board.clone();
            tokio::spawn(async move {
                while let Some(msg) = wheel_override.recv().await {
                    let req: Value = serde_json::from_str(&msg.data).unwrap();
                    let mut b = board.lock().unwrap();
                    b.permille = req["left_permille"].as_i64().unwrap();
                    b.override_until = Some(Instant::now() + Duration::from_millis(req["ttl_ms"].as_u64().unwrap()));
                    b.overrides.push(req);
                }
            });
        }
        {
            let leds = leds.clone();
            tokio::spawn(async move {
                while let Some(msg) = led.recv().await {
                    leds.lock().unwrap().push(serde_json::from_str(&msg.data).unwrap());
                }
            });
        }
        {
            let board = board.clone();
            tokio::spawn(async move {
                let mut tick = tokio::time::interval(Duration::from_millis(50));
                loop {
                    tick.tick().await;
                    let msg = {
                        let mut b = board.lock().unwrap();
                        if b.override_until.is_some_and(|until| Instant::now() > until) {
                            b.permille = 0;
                        }
                        let target = b.permille as f64 / 1000.0 * MAX_RPM;
                        b.rpm += (target - b.rpm) * (1.0 - (-0.05 / TAU_S).exp());
                        b.seq = (b.seq + 1) % 256;
                        frame(&b)
                    };
                    if telemetry.publish(&msg).is_err() {
                        return;
                    }
                }
            });
        }
        FakeBase { board, leds }
    }
}

/// nav's lock service: answers `granted`, records every request.
fn lock_server(test: &Scope, granted: bool) -> Arc<Mutex<Vec<MissionOperationLock::Request>>> {
    let mut service = test
        .service::<MissionOperationLock::Request, MissionOperationLock::Response>("/mission_operation_lock")
        .unwrap();
    let seen = Arc::new(Mutex::new(Vec::new()));
    let out = seen.clone();
    tokio::spawn(async move {
        while let Some(req) = service.recv().await {
            out.lock().unwrap().push(req.message.clone());
            let message = if granted { "granted".to_string() } else { "navigation goal active".to_string() };
            let _ = req.respond(MissionOperationLock::Response { success: granted, message });
        }
    });
    seen
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

/// Idle, as on r2r: the latched status `idle`/`ready` for a late joiner, the service's refusals, no reader of the
/// base telemetry, and every endpoint where the manifest has it: `/pid_autotune/status` and `/mower_base/led_command`
/// R TL 1, `/mower_base/pid_command` R V 4, `/mower_base/wheel_override` BE V 1, `/nav_operation_active` read R V 10,
/// the service and the lock's client.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn idle_it_serves_and_shows_its_status_and_reads_no_telemetry() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let autotune = Autotune::start(&bus);
    let client = test.client::<PidAutotune::Request, PidAutotune::Response>("/pid_autotune").unwrap();
    assert_eq!(call(&client, "abort").await, (false, "nothing to abort".to_string()));
    assert_eq!(call(&client, " Apply ").await, (false, "no tuned gains waiting for review".to_string()));
    assert_eq!(call(&client, "Bogus").await, (false, "unknown op 'Bogus' (start|abort|apply|discard)".to_string()));
    assert_eq!(call(&client, "it's").await, (false, "unknown op \"it's\" (start|abort|apply|discard)".to_string()));

    let mut late = test.subscribe::<Text>("/pid_autotune/status", Qos::latched()).unwrap();
    let first = status(&late.try_recv().expect("the latched status"));
    assert_eq!((first["state"].as_str(), first["message"].as_str()), (Some("idle"), Some("ready")));
    assert!(first["error"].is_null() && first["started_at"].is_null());

    mower_bridge::coverage::assert_in_manifest(&autotune.m);
    let topics = bus.topics();
    let of = |name: &str| topics.iter().find(|t| t.name == name).cloned().unwrap();
    assert_eq!(of("/pid_autotune/status").publisher_qos, [Qos::latched()]);
    assert_eq!(of("/mower_base/pid_command").publisher_qos, [Qos::default().keep_last(4)]);
    assert_eq!(of("/mower_base/wheel_override").publisher_qos, [BEST_EFFORT_1]);
    assert_eq!(of("/mower_base/led_command").publisher_qos, [Qos::latched()]);
    assert_eq!(of("/nav_operation_active").subscriber_qos, [Qos::default()]);
    assert!(telemetry_readers(&bus).is_empty(), "a telemetry reader while idle");
    let services = bus.services();
    let lock = services.iter().find(|s| s.name == "/mission_operation_lock").unwrap();
    assert_eq!(lock.clients, ["pid_autotune"]);
    autotune.stop().await;
    assert!(bus.topics().iter().all(|t| !t.publishers.contains(&"pid_autotune".to_string())));
}

/// A session with no base: the telemetry reader is made for it, the precheck gives up after the telemetry timeout
/// plus the first frame's grace (0.5 + 1.0 s), and the reader goes at the first frame after the session.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_session_reads_the_telemetry_only_while_it_runs() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let seen = record_status(test.subscribe::<Text>("/pid_autotune/status", STATUS_QUEUE).unwrap());
    let autotune = Autotune::start(&bus);
    let client = test.client::<PidAutotune::Request, PidAutotune::Response>("/pid_autotune").unwrap();
    let t0 = Instant::now();
    assert_eq!(call(&client, "start").await, (true, "auto-tune started".to_string()));
    assert_eq!(telemetry_readers(&bus), ["pid_autotune"], "the session's reader, at once");
    assert_eq!(call(&client, "start").await.0, false, "a second start while one runs");
    eventually("the session to fail", || last_state(&seen).as_deref() == Some("failed")).await;
    let elapsed = t0.elapsed();
    assert!(elapsed >= Duration::from_millis(1400) && elapsed < Duration::from_millis(3000), "{elapsed:?}");
    let last = seen.lock().unwrap().last().unwrap().1.clone();
    assert_eq!(last["error"], "no /mower_base/telemetry (is the base driver running?)");
    // the reader goes with the next frame
    assert_eq!(telemetry_readers(&bus), ["pid_autotune"]);
    let base = test.publisher::<Text>("/mower_base/telemetry", BEST_EFFORT_1).unwrap();
    base.publish(&Text { data: "{}".to_string() }).unwrap();
    eventually("the reader to go", || telemetry_readers(&bus).is_empty()).await;
    autotune.stop().await;
}

/// The whole session over the bus against a fake board and nav's lock: the lock taken and given back, the busy
/// light and the normal one, the open-loop steps through the override at 10 Hz, the board's acknowledgements, the
/// fit, the closed-loop check, the operator's "apply" and the flash save, every state on the 5 Hz status, and the
/// telemetry reader gone afterwards.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_session_tunes_verifies_and_saves_through_the_bus() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let base = FakeBase::start(&test);
    let locks = lock_server(&test, true);
    let nav = test.publisher::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
    nav.publish(&Bool { data: false }).unwrap();
    let seen = record_status(test.subscribe::<Text>("/pid_autotune/status", STATUS_QUEUE).unwrap());
    let autotune = Autotune::start(&bus);
    let client = test.client::<PidAutotune::Request, PidAutotune::Response>("/pid_autotune").unwrap();
    assert_eq!(call(&client, "start").await.0, true);
    eventually("the review", || matches!(last_state(&seen).as_deref(), Some("review" | "failed"))).await;
    let reviewed = seen.lock().unwrap().last().unwrap().1.clone();
    assert_eq!(reviewed["state"], "review", "{reviewed:#}");
    assert!(reviewed["new_gains"]["left"]["kp"].as_f64().unwrap() > 0.0);
    assert!(reviewed["model"]["left"]["tau"].as_f64().unwrap() > 0.0);
    assert!(!reviewed["samples"]["open_loop"]["left"].as_array().unwrap().is_empty());
    assert_eq!(call(&client, "apply").await, (true, "apply".to_string()));
    eventually("the end", || matches!(last_state(&seen).as_deref(), Some("done" | "failed" | "idle"))).await;
    let done = seen.lock().unwrap().last().unwrap().1.clone();
    assert_eq!((done["state"].as_str(), done["message"].as_str()), (Some("done"), Some("new gains saved to flash")));
    assert!(done["error"].is_null());

    // every state, in order, and the 5 Hz progress in between
    let states: Vec<String> = {
        let mut v: Vec<String> = seen.lock().unwrap().iter().map(|(_, s)| s["state"].as_str().unwrap().to_string()).collect();
        v.dedup();
        v
    };
    assert_eq!(states, ["idle", "precheck", "open_loop", "fitting", "verify", "review", "saving", "done"]);
    let in_open_loop: Vec<Instant> =
        seen.lock().unwrap().iter().filter(|(_, s)| s["state"] == "open_loop").map(|(t, _)| *t).collect();
    let span = in_open_loop.last().unwrap().duration_since(in_open_loop[0]).as_secs_f64();
    assert!(in_open_loop.len() as f64 >= span * 5.0 * 0.8, "{} in {span:.2} s", in_open_loop.len());

    // the lock: taken, then given back, under the node's own name
    let locks = locks.lock().unwrap().clone();
    assert_eq!(locks.iter().map(|r| r.acquire).collect::<Vec<_>>(), [true, false]);
    assert!(locks[0].owner.starts_with("/pid_autotune:") && locks[0].operation == "pid auto-tune", "{:?}", locks[0]);
    // the lights: busy, then normal
    let leds = base.leds.lock().unwrap().clone();
    assert_eq!(leds.iter().map(|l| l["mode"].as_i64().unwrap()).collect::<Vec<_>>(), [6, 1]);
    // the board: open loop with the old gains, the new ones closed loop, then saved
    let b = base.board.lock().unwrap();
    let modes: Vec<(i64, i64)> =
        b.pid_requests.iter().map(|r| (r["closed_loop"].as_i64().unwrap(), r["persist"].as_i64().unwrap())).collect();
    assert_eq!(modes, [(0, 0), (1, 0), (1, 1)]);
    assert_eq!(b.pid_requests[0]["left"]["kp"], 2.0);
    let steps: Vec<i64> = b.overrides.iter().map(|o| o["left_permille"].as_i64().unwrap()).collect();
    for permille in [0, 400, 700, 500] {
        assert!(steps.contains(&permille), "{permille} in {steps:?}");
    }
    assert_eq!(b.overrides.last().unwrap(), &json!({"left_permille": 0, "right_permille": 0, "ttl_ms": 0}));
    drop(b);
    eventually("the telemetry reader to go", || telemetry_readers(&bus).is_empty()).await;
    autotune.stop().await;
}

/// Preconditions, as on r2r: navigation active, or the lock refused, and the session fails with the old gains
/// put back.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_busy_robot_refuses_the_session() {
    let bus = Bus::new();
    let test = bus.scope("test");
    let base = FakeBase::start(&test);
    let _locks = lock_server(&test, false);
    let nav = test.publisher::<Bool>("/nav_operation_active", Qos::latched()).unwrap();
    let seen = record_status(test.subscribe::<Text>("/pid_autotune/status", STATUS_QUEUE).unwrap());
    let autotune = Autotune::start(&bus);
    let client = test.client::<PidAutotune::Request, PidAutotune::Response>("/pid_autotune").unwrap();
    // pid_autotune's reader is volatile: it hears the flag once it is up, not nav's history
    eventually("its nav reader", || bus.topic("/nav_operation_active").is_some_and(|t| !t.subscribers.is_empty())).await;
    nav.publish(&Bool { data: true }).unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(call(&client, "start").await.0, true);
    eventually("the refusal", || last_state(&seen).as_deref() == Some("failed")).await;
    assert_eq!(seen.lock().unwrap().last().unwrap().1["error"], "navigation is active");

    nav.publish(&Bool { data: false }).unwrap();
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(call(&client, "start").await.0, true);
    eventually("the second refusal", || {
        seen.lock().unwrap().last().is_some_and(|(_, s)| s["state"] == "failed" && s["error"] != "navigation is active")
    })
    .await;
    assert_eq!(seen.lock().unwrap().last().unwrap().1["error"], "robot is busy: navigation goal active");
    // the gains were only ever put back: no open loop, no save
    assert!(base.board.lock().unwrap().pid_requests.iter().all(|r| r["closed_loop"] == 1 && r["persist"] == 0));
    autotune.stop().await;
}

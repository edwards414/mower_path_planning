//! robot_status on the bus (step E2 of docs/ROS_FREE_PLAN.md): the module's own `run`, its nine inputs published and
//! its outputs read through another scope of the same bus, the way mower_rsd's other modules and the bus/DDS bridge
//! reach it.
//!
//! - **r2r:** the module keeps a node for its parameters and logs, so the tests need the process's r2r context. It is
//!   on ROS domain 81, set before the context exists, apart from other DDS users of the host; no message goes
//!   through DDS. No parameter is set, so the names are the defaults (production reads `/odom_slow` where these read
//!   `/odom`, through the same single subscription).
//! - **Time:** the module's wall timers run on a manual clock, so a tick is one `advance`. Each test runs on one
//!   thread: the module's tasks run only while the test awaits, in the order they were woken, so an input published
//!   before an `advance` is read before the tick it feeds. Ages, staleness and the busy window still use the real
//!   monotonic clock, as before E2.
//! - **State dir:** the module reads `MOWER_STATE_DIR` once, when it starts. The variable is process-wide, so the
//!   tests take turns (`Status::start`), each with a directory of its own.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, SystemTime};

use futures::StreamExt;
use mower_bus::{Bus, Client, Clock, ManualClock, Message, Qos, Scope, Subscriber, Time};
use mower_rs_common::{read_json_file, unix_time_s, ModuleCtx, ModuleResult, Remaps, Shutdown};
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::{BatteryState, Imu, NavSatFix};
use r2r::std_msgs::msg::{Bool, String as StringMsg};
use r2r::std_srvs::srv::Trigger;
use serde_json::{json, Value};

/// For anything the module has to do. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(10);

/// The process's r2r context, on ROS domain 81.
fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "81");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

/// The latched profile of robot_status's own writers and of the latched topics it reads.
fn latched() -> Qos {
    Qos::default().keep_last(1).reliable().transient_local()
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

fn doc(msg: &Arc<StringMsg>) -> Value {
    serde_json::from_str(&msg.data).expect("JSON")
}

/// The fields `keys` of the JSON object `v` as an object of their own (a missing one as null), to compare several
/// at once.
fn pick(v: &Value, keys: &[&str]) -> Value {
    Value::Object(keys.iter().map(|k| (k.to_string(), v[*k].clone())).collect())
}

async fn next<T: Message>(sub: &mut Subscriber<T>) -> Arc<T> {
    tokio::time::timeout(WAIT, sub.next()).await.expect("nothing arrived").expect("the test's scope closed")
}

type TriggerClient = Client<Trigger::Request, Trigger::Response>;

async fn call(client: &TriggerClient) -> Trigger::Response {
    let pending = client.request(&Trigger::Request::default()).expect("no server");
    tokio::time::timeout(WAIT, pending).await.expect("no answer").expect("the call failed")
}

fn host_request(dir: &Path) -> Option<Value> {
    read_json_file(dir.join("host.request"))
}

/// One start of robot_status: on a bus of its own (`test` is the scope of everything else in the process), a manual
/// clock and a state dir of its own, with the remap rules `rules` (`-r` arguments).
struct Status {
    bus: Bus,
    test: Scope,
    clock: ManualClock,
    dir: PathBuf,
    shutdown: Shutdown,
    scope: Scope,
    task: tokio::task::JoinHandle<ModuleResult>,
    _turn: MutexGuard<'static, ()>,
}

impl Status {
    /// The module only runs once the test awaits: the test makes its readers first, so that it sees the module's
    /// first publishes (`/robot/info` and the lights come at once).
    fn start(name: &str, rules: &[&str]) -> Status {
        static TURN: Mutex<()> = Mutex::new(());
        let turn = TURN.lock().unwrap_or_else(PoisonError::into_inner);
        let dir = std::env::temp_dir().join(format!("robot_status_bus_{}_{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::env::set_var("MOWER_STATE_DIR", &dir);
        let argv: Vec<String> = std::iter::once("--ros-args")
            .chain(rules.iter().flat_map(|rule| ["-r", *rule]))
            .map(str::to_string)
            .collect();
        let remaps = Arc::new(Remaps::from_args(&argv).unwrap());
        let (clock, manual) = Clock::manual(Time::from_system(SystemTime::now()));
        let bus = Bus::new();
        let test = bus.scope("test");
        let scope = bus.scope("robot_status");
        let shutdown = Shutdown::new();
        let m = ModuleCtx::new("robot_status", shutdown.clone())
            .with_bus(scope.clone())
            .with_clock(clock)
            .with_remaps(remaps);
        let task = tokio::spawn(robot_status::run(context(), m));
        Status { bus, test, clock: manual, dir, shutdown, scope, task, _turn: turn }
    }

    /// Wait until the module has made its endpoints and timers: its first `/robot/info` is out.
    async fn up(&self) {
        let mut info = self.test.subscribe::<StringMsg>("/robot/info", latched()).unwrap();
        next(&mut info).await;
    }

    /// Let every task that can run, run: on one thread they run while the test waits.
    async fn settle(&self) {
        tokio::time::sleep(ms(20)).await;
    }

    fn client(&self, name: &str) -> TriggerClient {
        self.test.client::<Trigger::Request, Trigger::Response>(name).unwrap()
    }

    /// What mower_rsd does: the module stops when asked, then its scope is closed.
    async fn stop(self) {
        self.shutdown.trigger();
        let result = tokio::time::timeout(WAIT, self.task).await.expect("the module did not stop");
        result.expect("the module panicked").expect("the module failed");
        self.scope.close();
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

#[tokio::test(flavor = "current_thread")]
async fn online_follows_the_odometry_at_2_hz_and_replays_to_a_late_reader() {
    let s = Status::start("online", &[]);
    let mut online = s.test.subscribe::<Bool>("/robot/online", latched()).unwrap();
    let odom = s.test.publisher::<Odometry>("/odom", Qos::default()).unwrap();
    s.up().await;

    // An rcl wall timer: the first tick one period (0.5 s) after the module made it.
    s.clock.advance(ms(499));
    s.settle().await;
    assert!(online.try_recv().is_err(), "a tick before its period");
    s.clock.advance(ms(1));
    assert!(!next(&mut online).await.data, "no odometry yet");

    odom.publish(&Odometry::default()).unwrap();
    s.clock.advance(ms(500));
    assert!(next(&mut online).await.data);
    // Every tick publishes, not only the changes.
    s.clock.advance(ms(500));
    assert!(next(&mut online).await.data);
    s.settle().await;
    assert!(online.try_recv().is_err(), "one message per tick");

    // Latched: a reader that joins now gets the last value at once; a volatile one gets nothing.
    let mut late = s.test.subscribe::<Bool>("/robot/online", latched()).unwrap();
    assert!(late.try_recv().unwrap().data);
    let mut volatile = s.test.subscribe::<Bool>("/robot/online", Qos::default()).unwrap();
    assert!(volatile.try_recv().is_err());

    // heartbeat_stale_timeout_s (2 s) of real time without odometry: offline.
    tokio::time::sleep(ms(2100)).await;
    s.clock.advance(ms(500));
    assert!(!next(&mut online).await.data);
    s.stop().await;
}

#[tokio::test(flavor = "current_thread")]
async fn telemetry_carries_every_input_at_10_hz() {
    let s = Status::start("telemetry", &[]);
    // Readers deep enough to count what a second brings.
    let mut telemetry = s.test.subscribe::<StringMsg>("/robot/telemetry", Qos::default()).unwrap();
    let mut online = s.test.subscribe::<Bool>("/robot/online", Qos::default()).unwrap();
    let mut info = s.test.subscribe::<StringMsg>("/robot/info", Qos::default()).unwrap();
    // Each input with its producer's profile (docs/BUS_TOPICS.md 2.2-2.7).
    let fix = s.test.publisher::<NavSatFix>("/fix", Qos::default()).unwrap();
    let filtered = s.test.publisher::<NavSatFix>("/gps/filtered", Qos::default()).unwrap();
    let imu = s.test.publisher::<Imu>("/imu/data", Qos::default()).unwrap();
    let odom = s.test.publisher::<Odometry>("/odom", Qos::default()).unwrap();
    let base_qos = Qos::default().keep_last(1).best_effort();
    let base = s.test.publisher::<StringMsg>("/mower_base/telemetry", base_qos).unwrap();
    let battery = s.test.publisher::<BatteryState>("/battery_state", Qos::default()).unwrap();
    let aon = s.test.publisher::<BatteryState>("/aon_battery_state", Qos::default()).unwrap();
    std::fs::write(s.dir.join("link_status.json"), r#"{"iface":"wwan0","sinr_db":3}"#).unwrap();
    next(&mut info).await;

    let mut f = NavSatFix::default();
    f.header.frame_id = "gps_link".into();
    (f.latitude, f.longitude, f.altitude) = (25.0, 121.5, 10.0);
    f.status.status = 2;
    f.status.service = 1;
    f.position_covariance = vec![0.0004, 0.0, 0.0, 0.0, 0.0009, 0.0, 0.0, 0.0, 0.0016];
    f.position_covariance_type = 2;
    fix.publish(&f).unwrap();
    let mut g = NavSatFix::default();
    (g.latitude, g.longitude, g.altitude) = (25.1, 121.6, 11.0);
    filtered.publish(&g).unwrap();
    let mut i = Imu::default();
    i.header.frame_id = "imu_link".into();
    let half = 45f64.to_radians();
    (i.orientation.z, i.orientation.w) = (half.sin(), half.cos());
    i.orientation_covariance = vec![0.01; 9];
    i.angular_velocity.z = 0.5;
    i.linear_acceleration.z = 9.81;
    imu.publish(&i).unwrap();
    let mut o = Odometry::default();
    o.header.frame_id = "odom".into();
    (o.pose.pose.position.x, o.pose.pose.position.y) = (1.5, -2.25);
    o.pose.pose.orientation.w = 1.0;
    o.twist.twist.linear.x = 0.01;
    odom.publish(&o).unwrap();
    base.publish(&StringMsg { data: r#"{"feedback_age_s":0.04,"estop":false}"#.into() }).unwrap();
    let pack = BatteryState {
        present: true, percentage: 0.5, voltage: 22.68, current: f32::NAN, power_supply_status: 2,
        ..Default::default()
    };
    battery.publish(&pack).unwrap();
    let cell = BatteryState {
        present: true, percentage: 0.9, voltage: 4.1, current: f32::NAN, power_supply_status: 4,
        ..Default::default()
    };
    aon.publish(&cell).unwrap();

    // An rcl wall timer at 10 Hz: the first tick 0.1 s after the module made it.
    s.clock.advance(ms(100));
    let d = doc(&next(&mut telemetry).await);
    assert_eq!(d["seq"], json!(1));
    assert!(d["robot_id"].is_string() && d["time"].as_f64().unwrap() > 1.7e9);
    let gps = &d["gps"];
    assert_eq!(
        pick(gps, &["valid", "lat", "lon", "alt", "status", "status_text", "service", "frame_id"]),
        json!({"valid": true, "lat": 25.0, "lon": 121.5, "alt": 10.0, "status": 2, "status_text": "RTK FIXED",
               "service": 1, "frame_id": "gps_link"})
    );
    let accuracy = pick(gps, &["h_acc_m", "v_acc_m", "cov_type"]);
    assert_eq!(accuracy, json!({"h_acc_m": 0.03, "v_acc_m": 0.04, "cov_type": 2}));
    assert_eq!(pick(&gps["filtered"], &["lat", "lon", "alt"]), json!({"lat": 25.1, "lon": 121.6, "alt": 11.0}));
    assert!(gps["age_s"].as_f64().unwrap() < 1.0);
    assert_eq!(
        pick(&d["imu"], &["valid", "frame_id", "roll_deg", "yaw_deg", "gyro_dps", "accel_mps2", "has_orientation"]),
        json!({"valid": true, "frame_id": "imu_link", "roll_deg": 0.0, "yaw_deg": 90.0, "gyro_dps": [0.0, 0.0, 28.648],
               "accel_mps2": [0.0, 0.0, 9.81], "has_orientation": true})
    );
    assert_eq!(
        pick(&d["odom"], &["valid", "x", "y", "yaw_deg", "vx", "frame_id"]),
        json!({"valid": true, "x": 1.5, "y": -2.25, "yaw_deg": 0.0, "vx": 0.01, "frame_id": "odom"})
    );
    assert!(d["odom"]["age_s"].as_f64().unwrap() < 1.0);
    let base_block = pick(&d["base"], &["valid", "feedback_age_s", "estop"]);
    assert_eq!(base_block, json!({"valid": true, "feedback_age_s": 0.04, "estop": false}));
    assert!(d["base"]["age_s"].as_f64().unwrap() < 1.0);
    assert_eq!(
        pick(&d["battery"], &["valid", "present", "pct", "voltage_v", "current_a", "status"]),
        json!({"valid": true, "present": true, "pct": 0.5, "voltage_v": 22.68, "current_a": null,
               "status": "discharging"})
    );
    assert_eq!(pick(&d["battery"]["aon"], &["pct", "status"]), json!({"pct": 0.9, "status": "full"}));
    assert_eq!(pick(&d["link"], &["valid", "iface"]), json!({"valid": true, "iface": "wwan0"}));
    assert!(d["host"]["uptime_s"].is_number());
    assert_eq!(d["info"]["api_version"], json!(2), "the last /robot/info");

    // Ten ticks a second, one document each; in the same second the heartbeat ticks twice and the info once.
    for seq in 2..=10 {
        s.clock.advance(ms(100));
        assert_eq!(doc(&next(&mut telemetry).await)["seq"], json!(seq));
    }
    s.settle().await;
    assert!(telemetry.try_recv().is_err());
    let beats = std::iter::from_fn(|| online.try_recv().ok()).count();
    let infos = std::iter::from_fn(|| info.try_recv().ok()).count();
    assert_eq!((beats, infos), (2, 1));

    // Volatile, as before: a reader that joins late gets no history.
    let mut late = s.test.subscribe::<StringMsg>("/robot/telemetry", latched()).unwrap();
    assert!(late.try_recv().is_err());
    s.stop().await;
}

#[tokio::test(flavor = "current_thread")]
async fn info_follows_the_firmware_and_navigation_and_drives_the_lights() {
    let s = Status::start("info", &[]);
    let mut info = s.test.subscribe::<StringMsg>("/robot/info", latched()).unwrap();
    let mut led = s.test.subscribe::<StringMsg>("/mower_base/led_command", latched()).unwrap();
    // mower_base's and nav's writers are latched; so are robot_status's readers.
    let firmware = s.test.publisher::<StringMsg>("/mower_base/firmware_info", latched()).unwrap();
    let nav = s.test.publisher::<Bool>("/nav_operation_active", latched()).unwrap();

    // The first document comes at once, before any tick, and sets the lights to normal.
    let first = doc(&next(&mut info).await);
    let head = pick(&first, &["api_version", "busy", "update"]);
    assert_eq!(head, json!({"api_version": 2, "busy": false, "update": null}));
    assert_eq!(first["firmware"]["running"], Value::Null);
    let lights = doc(&next(&mut led).await);
    assert_eq!(lights, json!({"mode": 1, "r": 110, "g": 110, "b": 110, "period_ms": 0}));
    let mut late = s.test.subscribe::<StringMsg>("/robot/info", latched()).unwrap();
    assert_eq!(doc(&late.try_recv().unwrap()), first, "latched");

    // New firmware info is published at once, without waiting for the 1 Hz tick.
    let running =
        json!({"version": "1.2.0", "semver": "1.2.0", "git_sha": "01234567", "build_unix": 100, "dirty": false});
    firmware.publish(&StringMsg { data: running.to_string() }).unwrap();
    assert_eq!(doc(&next(&mut info).await)["firmware"]["running"], running);
    // The same document again changes nothing.
    firmware.publish(&StringMsg { data: running.to_string() }).unwrap();
    s.settle().await;
    assert!(info.try_recv().is_err());

    // Navigating: busy at the next tick, in the document and in robot_status.json for the host.
    nav.publish(&Bool { data: true }).unwrap();
    s.clock.advance(ms(999));
    s.settle().await;
    assert!(info.try_recv().is_err(), "a tick before its period");
    s.clock.advance(ms(1));
    assert_eq!(doc(&next(&mut info).await)["busy"], json!(true));
    let status = read_json_file(s.dir.join("robot_status.json")).unwrap();
    let status = pick(&status, &["busy", "nav_running", "moving"]);
    assert_eq!(status, json!({"busy": true, "nav_running": true, "moving": false}));

    // The host starts an update: the amber orbit, latched for the base.
    std::fs::write(s.dir.join("update_status.json"), r#"{"state":"pulling","message":"m","time":5}"#).unwrap();
    s.clock.advance(ms(1000));
    assert_eq!(doc(&next(&mut info).await)["update"]["state"], json!("pulling"));
    let orbit = json!({"mode": 6, "r": 255, "g": 180, "b": 0, "period_ms": 1600});
    assert_eq!(doc(&next(&mut led).await), orbit);
    let mut late = s.test.subscribe::<StringMsg>("/mower_base/led_command", latched()).unwrap();
    assert_eq!(doc(&late.try_recv().unwrap()), orbit, "latched, the newest only");
    s.stop().await;
}

#[tokio::test(flavor = "current_thread")]
async fn the_system_services_write_the_host_request_and_answer_as_before() {
    let s = Status::start("services", &[]);
    let nav = s.test.publisher::<Bool>("/nav_operation_active", latched()).unwrap();
    let odom = s.test.publisher::<Odometry>("/odom", Qos::default()).unwrap();
    let update = s.client("/system/update");
    let restart = s.client("/system/restart");
    let check = s.client("/system/check_update");
    s.up().await;

    for (client, action) in [(&update, "update"), (&restart, "restart"), (&check, "check")] {
        let answer = call(client).await;
        assert!(answer.success, "{action}: {}", answer.message);
        assert_eq!(answer.message, format!("{action} requested; watch /robot/info update.state"));
        let request = host_request(&s.dir).unwrap();
        assert_eq!(pick(&request, &["action", "requested_by"]), json!({"action": action, "requested_by": "app"}));
        assert!((request["time"].as_f64().unwrap() - unix_time_s()).abs() < 5.0);
    }

    // Navigating: update and restart are refused and write nothing; the registry check is still allowed.
    std::fs::remove_file(s.dir.join("host.request")).unwrap();
    nav.publish(&Bool { data: true }).unwrap();
    for client in [&update, &restart] {
        let answer = call(client).await;
        assert!(!answer.success);
        assert_eq!(answer.message, "robot is busy (moving or navigating); stop it first");
    }
    assert!(host_request(&s.dir).is_none());
    assert!(call(&check).await.success);
    nav.publish(&Bool { data: false }).unwrap();
    assert!(call(&update).await.success);

    // Moving (faster than moving_speed_threshold within busy_timeout_s) is busy too.
    let mut moving = Odometry::default();
    moving.twist.twist.linear.x = 0.3;
    odom.publish(&moving).unwrap();
    assert!(!call(&restart).await.success);

    // A request file that cannot be written is answered, with the reason.
    std::fs::remove_file(s.dir.join("host.request")).unwrap();
    std::fs::create_dir_all(s.dir.join("host.request").join("x")).unwrap();
    let answer = call(&check).await;
    assert!(!answer.success);
    assert!(answer.message.starts_with("could not write host request: "), "{}", answer.message);
    s.stop().await;
}

/// The endpoints as mower_rsd's other modules and the bridge find them: the ROS names and types, one writer or
/// reader each (the heartbeat shares the odometry reader), and nothing left once the scope has closed.
#[tokio::test(flavor = "current_thread")]
async fn the_endpoints_are_on_the_bus_under_their_ros_names() {
    let s = Status::start("endpoints", &[]);
    s.up().await;
    let writes = [
        ("/robot/online", "std_msgs/msg/Bool"),
        ("/robot/info", "std_msgs/msg/String"),
        ("/robot/telemetry", "std_msgs/msg/String"),
        ("/mower_base/led_command", "std_msgs/msg/String"),
    ];
    let reads = [
        ("/odom", "nav_msgs/msg/Odometry"),
        ("/fix", "sensor_msgs/msg/NavSatFix"),
        ("/gps/filtered", "sensor_msgs/msg/NavSatFix"),
        ("/imu/data", "sensor_msgs/msg/Imu"),
        ("/mower_base/telemetry", "std_msgs/msg/String"),
        ("/battery_state", "sensor_msgs/msg/BatteryState"),
        ("/aon_battery_state", "sensor_msgs/msg/BatteryState"),
        ("/mower_base/firmware_info", "std_msgs/msg/String"),
        ("/nav_operation_active", "std_msgs/msg/Bool"),
    ];
    let topics = s.bus.topics();
    let mine = |names: &[String]| names.iter().filter(|n| *n == "robot_status").count();
    // (ROS type, writers of robot_status, readers of robot_status)
    let seen = |name: &str| {
        let t = topics.iter().find(|t| t.name == name).unwrap_or_else(|| panic!("{name}"));
        (t.ros_type.clone(), mine(&t.publishers), mine(&t.subscribers))
    };
    for (name, ros_type) in writes {
        assert_eq!(seen(name), (Some(ros_type.to_string()), 1, 0), "{name}");
    }
    for (name, ros_type) in reads {
        assert_eq!(seen(name), (Some(ros_type.to_string()), 0, 1), "{name}");
    }
    let involved = topics.iter().filter(|t| mine(&t.publishers) + mine(&t.subscribers) > 0).count();
    assert_eq!(involved, writes.len() + reads.len());
    let services = s.bus.services();
    for name in ["/system/update", "/system/restart", "/system/check_update"] {
        let srv = services.iter().find(|srv| srv.name == name).unwrap_or_else(|| panic!("{name}"));
        assert_eq!(srv.ros_type.as_deref(), Some("std_srvs/srv/Trigger"), "{name}");
        assert_eq!(srv.server.as_deref(), Some("robot_status"), "{name}");
    }
    assert_eq!(services.len(), 3);

    let bus = s.bus.clone();
    s.stop().await;
    assert!(bus.topics().iter().all(|t| mine(&t.publishers) + mine(&t.subscribers) == 0));
    assert!(bus.services().iter().all(|srv| srv.server.is_none()));
}

/// Every name goes through the process's remap rules for the node, the absolute ones and those from parameters
/// alike; a topic rule does not rename a service, and a rule for another node changes nothing.
#[tokio::test(flavor = "current_thread")]
async fn every_name_follows_the_remap_rules_of_the_node() {
    let s = Status::start(
        "remaps",
        &[
            "robot_status:/robot/online:=/status/online",
            "robot_status:/fix:=/gnss/fix",
            "robot_status:rosservice:///system/update:=/host/update",
            "robot_status:rostopic:///system/restart:=/nowhere",
            "battery_state:/robot/info:=/elsewhere",
        ],
    );
    let mut online = s.test.subscribe::<Bool>("/status/online", latched()).unwrap();
    let fix = s.test.publisher::<NavSatFix>("/gnss/fix", Qos::default()).unwrap();
    let mut telemetry = s.test.subscribe::<StringMsg>("/robot/telemetry", Qos::default()).unwrap();
    let update = s.client("/host/update");
    s.up().await;

    fix.publish(&NavSatFix { latitude: 25.0, ..Default::default() }).unwrap();
    // Half a second at once: the heartbeat's first tick, and one (late) telemetry tick.
    s.clock.advance(ms(500));
    assert!(!next(&mut online).await.data);
    assert_eq!(doc(&next(&mut telemetry).await)["gps"]["lat"], json!(25.0));
    assert!(call(&update).await.success);

    let topics = s.bus.topics();
    let used = |name: &str| {
        topics.iter().any(|t| t.name == name && t.publishers.iter().chain(&t.subscribers).any(|n| n == "robot_status"))
    };
    assert!(!used("/robot/online") && !used("/fix") && used("/robot/info") && !used("/elsewhere"));
    let services = s.bus.services();
    let served =
        |name: &str| services.iter().any(|srv| srv.name == name && srv.server.as_deref() == Some("robot_status"));
    assert!(!served("/system/update") && served("/system/restart") && !served("/nowhere"));
    s.stop().await;
}

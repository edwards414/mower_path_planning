//! mower_base on the bus (E2 step 6): the module's real `run_on_bus` with mower_rsd's `-r` rules, its STM32 a fake
//! board on a pty (the frames of `tools/fake_base.py`: 0x85 and 0x81 every 20 ms, a reply to 0x06), and its readers
//! and writers another scope of the same `Bus`, as the final guard, pid_autotune, robot_status and the bridge are in
//! mower_rsd.
//!
//! - **r2r:** the module keeps a node for its parameters and logs, on ROS domain 92; no message goes through DDS. One
//!   module at a time: two nodes named `mower_base` must not share the process's rosout writer.
//! - **Time:** the 25 Hz serial cycle runs on CLOCK_MONOTONIC and the system clock, as it does in production (the
//!   module doc, "Clocks"), so these tests wait in real time, with a short publisher settle and release window.

use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};
use std::time::{Duration, Instant, SystemTime};

use mower_base_core::protocol::{self as proto, FrameParser};
use mower_bus::{Bus, Publisher, Qos, Scope, Subscriber, Time};
use mower_rs_common::{ModuleCtx, ModuleResult, Shutdown};
use r2r::geometry_msgs::msg::TwistStamped;
use r2r::nav_msgs::msg::Odometry;
use r2r::sensor_msgs::msg::JointState;
use r2r::std_msgs::msg::String as Text;
use r2r::{Parameter, ParameterValue};

/// For anything that waits in real time. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

const BEST_EFFORT_1: Qos = Qos::default().keep_last(1).best_effort();

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "92");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

fn ms(n: u64) -> Duration {
    Duration::from_millis(n)
}

// ---------------------------------------------------------------------------
// The fake board
// ---------------------------------------------------------------------------

/// A raw pty, its slave linked at `link` for the driver to open, its master non-blocking (as mower_sim's `pty.rs`,
/// with `ptsname_r`, which tests running side by side can call at once).
struct Pty {
    master: OwnedFd,
    _slave: OwnedFd,
}

impl Pty {
    fn open(link: &std::path::Path) -> Pty {
        let (mut m, mut s) = (-1, -1);
        // SAFETY: openpty writes two descriptors; the other pointers may be null.
        let r = unsafe { libc::openpty(&mut m, &mut s, std::ptr::null_mut(), std::ptr::null(), std::ptr::null()) };
        assert_eq!(r, 0, "openpty: {}", std::io::Error::last_os_error());
        // SAFETY: both were just opened and are owned by nobody else.
        let (master, slave) = unsafe { (OwnedFd::from_raw_fd(m), OwnedFd::from_raw_fd(s)) };
        let mut name = [0u8; 128];
        // SAFETY: termios calls on descriptors we own; ptsname_r into a buffer we own.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            assert_eq!(libc::tcgetattr(slave.as_raw_fd(), &mut t), 0);
            libc::cfmakeraw(&mut t);
            assert_eq!(libc::tcsetattr(slave.as_raw_fd(), libc::TCSANOW, &t), 0);
            let flags = libc::fcntl(master.as_raw_fd(), libc::F_GETFL);
            libc::fcntl(master.as_raw_fd(), libc::F_SETFL, flags | libc::O_NONBLOCK);
            assert_eq!(libc::ptsname_r(master.as_raw_fd(), name.as_mut_ptr() as *mut libc::c_char, name.len()), 0);
        }
        let end = name.iter().position(|b| *b == 0).unwrap();
        let name = std::str::from_utf8(&name[..end]).unwrap().to_string();
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(&name, link).unwrap();
        Pty { master, _slave: slave }
    }

    fn read(&self, buf: &mut [u8]) -> usize {
        // SAFETY: read into a buffer we own, on a non-blocking descriptor.
        let n = unsafe { libc::read(self.master.as_raw_fd(), buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        n.max(0) as usize
    }

    fn write(&self, data: &[u8]) {
        let mut done = 0;
        while done < data.len() {
            // SAFETY: write from a slice we own, on a non-blocking descriptor.
            let n = unsafe { libc::write(self.master.as_raw_fd(), data[done..].as_ptr() as *const libc::c_void, data.len() - done) };
            if n <= 0 {
                break;
            }
            done += n as usize;
        }
    }
}

/// What the board was sent, frame by frame, with when.
#[derive(Default)]
struct Received {
    frames: Vec<(Instant, u8, Vec<u8>)>,
}

impl Received {
    /// Every 0x01's `(left, right)` permille since `since`.
    fn wheels_since(&self, since: Instant) -> Vec<(i16, i16)> {
        self.frames
            .iter()
            .filter(|(t, kind, p)| *t >= since && *kind == proto::WHEEL_SPEED_COMMAND && p.len() == 8)
            .map(|(_, _, p)| (i16::from_le_bytes([p[0], p[1]]), i16::from_le_bytes([p[2], p[3]])))
            .collect()
    }

    /// Every 0x02's permille since `since`.
    fn blade_since(&self, since: Instant) -> Vec<i16> {
        self.frames
            .iter()
            .filter(|(t, kind, p)| *t >= since && *kind == proto::LAWER_MOTOR_COMMAND && p.len() == 8)
            .map(|(_, _, p)| i16::from_le_bytes([p[0], p[1]]))
            .collect()
    }
}

/// A board that answers like the real one, enough for mower_base to arm: wheel feedback and motor status every
/// 20 ms (COMMAND_VALID once a 0x01 came, COMMAND_TIMEOUT when the last is older than 300 ms), firmware info on
/// 0x06.
struct Board {
    link: PathBuf,
    received: Arc<Mutex<Received>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Board {
    fn start(dir: &std::path::Path) -> Board {
        let link = dir.join("stmcom");
        let pty = Pty::open(&link);
        let received = Arc::new(Mutex::new(Received::default()));
        let running = Arc::new(AtomicBool::new(true));
        let thread = {
            let (received, running) = (received.clone(), running.clone());
            std::thread::spawn(move || {
                let mut parser = FrameParser::new();
                let mut seq = 0u8;
                let mut cmd = (0i16, 0i16);
                let mut last_command: Option<Instant> = None;
                let mut last_rx_seq = 0u8;
                let mut next_status = Instant::now();
                let mut buf = [0u8; 512];
                let send = |kind: u8, payload: &[u8], seq: &mut u8| {
                    pty.write(&proto::build_frame(kind, *seq, payload));
                    *seq = seq.wrapping_add(1);
                };
                while running.load(Ordering::Relaxed) {
                    let n = pty.read(&mut buf);
                    let now = Instant::now();
                    let mut frames = Vec::new();
                    parser.feed(&buf[..n], |kind, seq, payload| frames.push((kind, seq, payload.to_vec())));
                    for (kind, frame_seq, payload) in frames {
                        if kind == proto::WHEEL_SPEED_COMMAND && payload.len() == 8 {
                            cmd = (i16::from_le_bytes([payload[0], payload[1]]), i16::from_le_bytes([payload[2], payload[3]]));
                            last_command = Some(now);
                            last_rx_seq = frame_seq;
                        }
                        if kind == proto::INFO_REQUEST {
                            let mut p = vec![1, 2, 3, proto::PROTOCOL_VERSION];
                            p.extend_from_slice(&0xdead_beefu32.to_le_bytes());
                            p.extend_from_slice(&1_790_000_000u32.to_le_bytes());
                            p.extend_from_slice(&[0, 0, 0, 0]);
                            send(proto::FIRMWARE_INFO, &p, &mut seq);
                        }
                        received.lock().unwrap().frames.push((now, kind, payload));
                    }
                    if now >= next_status {
                        next_status = now + ms(20);
                        let mut p = Vec::with_capacity(24);
                        for v in [0i16, 0, 0, 0, cmd.0, cmd.1] {
                            p.extend_from_slice(&v.to_le_bytes());
                        }
                        p.extend_from_slice(&0i32.to_le_bytes());
                        p.extend_from_slice(&0i32.to_le_bytes());
                        p.extend_from_slice(&[proto::WHEEL_FLAG_OUTPUT_ENABLED, 0, 0, 0]);
                        send(proto::WHEEL_FEEDBACK_STATUS, &p, &mut seq);
                        let age_ms = last_command.map(|t| now.duration_since(t).as_millis() as u64);
                        let timeout = age_ms.map_or(true, |a| a > 300);
                        let flags = if age_ms.is_some() { proto::STATUS_FLAG_COMMAND_VALID } else { 0 }
                            | if timeout { proto::STATUS_FLAG_COMMAND_TIMEOUT } else { 0 };
                        let applied = if timeout { (0, 0) } else { cmd };
                        let mut p = Vec::with_capacity(12);
                        for v in [cmd.0, cmd.1, applied.0.saturating_mul(2), applied.1.saturating_mul(2)] {
                            p.extend_from_slice(&v.to_le_bytes());
                        }
                        p.extend_from_slice(&(age_ms.unwrap_or(u64::MAX).min(65_535) as u16).to_le_bytes());
                        p.push(flags);
                        p.push(last_rx_seq);
                        send(proto::MOTOR_STATUS, &p, &mut seq);
                    }
                    std::thread::sleep(ms(2));
                }
            })
        };
        Board { link, received, running, thread: Some(thread) }
    }
}

impl Drop for Board {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

// ---------------------------------------------------------------------------
// The module
// ---------------------------------------------------------------------------

struct Base {
    m: ModuleCtx,
    shutdown: Shutdown,
    scope: Scope,
    task: Option<tokio::task::JoinHandle<ModuleResult>>,
    _turn: MutexGuard<'static, ()>,
}

impl Base {
    /// mower_base on `bus`, its port the board's link: a 0.2 s publisher settle and release window, so that a
    /// release takes well under a second.
    fn start(bus: &Bus, board: &Board) -> Base {
        static TURN: Mutex<()> = Mutex::new(());
        let turn = TURN.lock().unwrap_or_else(PoisonError::into_inner);
        let node = r2r::Node::create(context(), "mower_base", "").unwrap();
        for (key, value) in [
            ("device", ParameterValue::String(board.link.display().to_string())),
            ("publisher_settle_s", ParameterValue::Double(0.2)),
            ("latch_release_s", ParameterValue::Double(0.2)),
            ("shutdown_command", ParameterValue::String(String::new())),
        ] {
            node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value));
        }
        let shutdown = Shutdown::new();
        let scope = bus.scope("mower_base");
        let m = ModuleCtx::new("mower_base", shutdown.clone())
            .with_bus(scope.clone())
            .with_remaps(Arc::new(mower_bridge::production_remaps()));
        let task = tokio::spawn(mower_base::run_on_bus(node, m.clone()));
        Base { m, shutdown, scope, task: Some(task), _turn: turn }
    }

    async fn stop(mut self) {
        self.shutdown.trigger();
        let task = self.task.take().unwrap();
        let result = tokio::time::timeout(WAIT, task).await.expect("the base did not stop").expect("it panicked");
        result.expect("the base failed");
        self.scope.close();
    }
}

fn tempdir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("mower_base_bus_{tag}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(ms(10)).await;
    }
}

/// Count what arrives on `sub` for `d`.
async fn count_for<T: mower_bus::Message>(sub: &mut Subscriber<T>, d: Duration) -> usize {
    let end = tokio::time::Instant::now() + d;
    let mut n = 0;
    while let Ok(Some(_)) = tokio::time::timeout_at(end, sub.recv()).await {
        n += 1;
    }
    n
}

fn twist(linear_x: f64) -> TwistStamped {
    let mut msg = TwistStamped::default();
    let (sec, nanosec) = Time::from_system(SystemTime::now()).to_parts();
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.twist.linear.x = linear_x;
    msg
}

/// Publish `linear_x` every 50 ms for `d`, as the final guard does at twist_mux's 20 Hz.
async fn stream(guard: &Publisher<TwistStamped>, linear_x: f64, d: Duration) {
    let end = Instant::now() + d;
    while Instant::now() < end {
        guard.publish(&twist(linear_x)).unwrap();
        tokio::time::sleep(ms(50)).await;
    }
}

fn moving(wheels: &[(i16, i16)]) -> usize {
    wheels.iter().filter(|(l, r)| *l != 0 || *r != 0).count()
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

/// What the base shows the bus, as it shows DDS on r2r: `/odom` and `/joint_states` at the cycle's 25 Hz
/// (SystemDefaults, R V 1), the slow copy at no more than 5 Hz (R V 10), the telemetry once per 0x85 (BE V 1), the
/// board's firmware latched for a late joiner (R TL 1); every endpoint in the manifest under its production name.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_base_publishes_on_the_bus_what_it_published_on_dds() {
    let dir = tempdir("outputs");
    let board = Board::start(&dir);
    let bus = Bus::new();
    let test = bus.scope("test");
    let mut odom = test.subscribe::<Odometry>("/odom", Qos::default()).unwrap();
    let mut odom_slow = test.subscribe::<Odometry>("/odom_slow", Qos::default()).unwrap();
    let mut joints = test.subscribe::<JointState>("/joint_states", Qos::sensor_data()).unwrap();
    let mut telemetry = test.subscribe::<Text>("/mower_base/telemetry", BEST_EFFORT_1).unwrap();
    let base = Base::start(&bus, &board);
    tokio::time::timeout(WAIT, odom.recv()).await.expect("no /odom").unwrap();

    let (n_odom, n_joints) = (count_for(&mut odom, ms(1000)).await, count_for(&mut joints, ms(1)).await);
    assert!((20..=27).contains(&n_odom), "/odom at {n_odom} Hz");
    assert!(n_joints >= 1, "/joint_states");
    let _ = odom_slow.latest();
    let n_slow = count_for(&mut odom_slow, ms(1000)).await;
    assert!((3..=6).contains(&n_slow), "/odom_slow at {n_slow} Hz");
    let frame = tokio::time::timeout(WAIT, telemetry.recv()).await.unwrap().unwrap();
    let doc: serde_json::Value = serde_json::from_str(&frame.data).unwrap();
    assert!(doc.get("wheel").is_some(), "{doc}");
    let joint = tokio::time::timeout(WAIT, joints.recv()).await.unwrap().unwrap();
    assert_eq!(joint.name, ["left_wheel_joint", "right_wheel_joint"]);
    assert_eq!(joint.header.frame_id, "base_link");
    let o = tokio::time::timeout(WAIT, odom.recv()).await.unwrap().unwrap();
    assert_eq!((o.header.frame_id.as_str(), o.child_frame_id.as_str()), ("odom", "base_footprint"));

    // the firmware the board told it, latched
    eventually("the firmware info", || {
        let mut late = test.subscribe::<Text>("/mower_base/firmware_info", Qos::latched()).unwrap();
        late.try_recv().is_ok()
    })
    .await;

    mower_bridge::coverage::assert_in_manifest(&base.m);
    let topics = bus.topics();
    let writers = |name: &str| topics.iter().find(|t| t.name == name).unwrap().publisher_qos.clone();
    let readers = |name: &str| topics.iter().find(|t| t.name == name).unwrap().subscriber_qos.clone();
    assert_eq!(writers("/odom"), [Qos::system_default()]);
    assert_eq!(writers("/joint_states"), [Qos::system_default()]);
    assert_eq!(writers("/tf"), [Qos::system_default()]);
    assert_eq!(writers("/odom_slow"), [Qos::default()]);
    assert_eq!(writers("/mower_base/telemetry"), [BEST_EFFORT_1]);
    assert_eq!(writers("/mower_base/firmware_info"), [Qos::latched()]);
    assert_eq!(readers("/drivetrain_guarded_cmd_vel"), [Qos::system_default()]);
    assert_eq!(readers("/mower_base/led_command"), [Qos::latched()]);
    assert_eq!(readers("/mower_base/pid_command"), [Qos::default().keep_last(4)]);
    for name in ["/mower_base/wheel_override", "/mower_base/servo_command", "/mower_base/blade_command"] {
        assert_eq!(readers(name), [BEST_EFFORT_1], "{name}");
    }
    base.stop().await;
    drop(board);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The arm latch with the bus's writers in place of the graph's (BUS_TOPICS.md 5.5 f). With no writer of
/// `/drivetrain_guarded_cmd_vel` its silence is no stop edge, so the wheels stay held however long it lasts; a writer
/// that comes with a live stream (a restart under nav2 or teleop) is held until it carries an explicit stop for the
/// release window, and only then drives. A bridge writer that counted all along would have let the first command of
/// that stream through (mower_bus_ros README, `from_ros_mirrored`).
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_arm_latch_counts_the_bus_s_writers_of_the_commands() {
    let dir = tempdir("latch");
    let board = Board::start(&dir);
    let bus = Bus::new();
    let test = bus.scope("test");
    let base = Base::start(&bus, &board);
    let started = Instant::now();
    eventually("the base's command reader", || {
        bus.topic("/drivetrain_guarded_cmd_vel").is_some_and(|t| t.subscribers == ["mower_base"])
    })
    .await;
    // no writer, longer than settle + silence window
    tokio::time::sleep(ms(1500)).await;
    assert_eq!(moving(&board.received.lock().unwrap().wheels_since(started)), 0);

    // the final guard comes, already streaming 0.2 m/s: held
    let guard = test.publisher::<TwistStamped>("/drivetrain_guarded_cmd_vel", Qos::default().keep_last(1)).unwrap();
    let held = Instant::now();
    stream(&guard, 0.2, ms(1200)).await;
    let wheels = board.received.lock().unwrap().wheels_since(held);
    assert!(wheels.len() >= 20, "{} wheel frames", wheels.len());
    assert_eq!(moving(&wheels), 0, "the latch let a live stream through: {wheels:?}");

    // an explicit stop for the release window, then the stick: it drives
    stream(&guard, 0.0, ms(400)).await;
    let released = Instant::now();
    stream(&guard, 0.2, ms(600)).await;
    let wheels = board.received.lock().unwrap().wheels_since(released);
    assert!(moving(&wheels) >= 5, "it did not follow the stick after the release: {wheels:?}");

    // silence: cmd_vel_timeout (0.25 s) brakes it
    tokio::time::sleep(ms(500)).await;
    let quiet = Instant::now();
    tokio::time::sleep(ms(300)).await;
    let wheels = board.received.lock().unwrap().wheels_since(quiet);
    assert_eq!(moving(&wheels), 0, "{wheels:?}");
    drop(guard);
    base.stop().await;
    drop(board);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The Studio's blade dead-man from the bus: 0x02 at the permille asked for while it is refreshed within its ttl,
/// and the blade stopped once it is not.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_blade_runs_while_its_dead_man_is_held() {
    let dir = tempdir("blade");
    let board = Board::start(&dir);
    let bus = Bus::new();
    let test = bus.scope("test");
    let blade = test.publisher::<Text>("/mower_base/blade_command", Qos::default()).unwrap();
    let base = Base::start(&bus, &board);
    eventually("the base's blade reader", || blade.subscription_count() == 1).await;
    tokio::time::sleep(ms(500)).await;
    let held = Instant::now();
    for _ in 0..10 {
        blade.publish(&Text { data: r#"{"permille":300,"ttl_ms":500}"#.to_string() }).unwrap();
        tokio::time::sleep(ms(200)).await;
    }
    let running = board.received.lock().unwrap().blade_since(held);
    assert!(running.iter().filter(|p| **p == 300).count() >= 5, "{running:?}");
    let let_go = Instant::now();
    tokio::time::sleep(ms(1000)).await;
    // the stop goes out once the last refresh's ttl (500 ms) has run out
    let after = board.received.lock().unwrap().blade_since(let_go);
    assert_eq!(after.last(), Some(&0), "the blade after its ttl: {after:?}");
    let stopped = board.received.lock().unwrap().frames.iter().rev()
        .find(|(_, kind, p)| *kind == proto::LAWER_MOTOR_COMMAND && p.len() == 8 && p[0] == 0 && p[1] == 0)
        .map(|(t, _, _)| *t)
        .unwrap();
    let after_s = stopped.duration_since(let_go).as_secs_f64();
    assert!((0.2..0.7).contains(&after_s), "the stop {after_s:.3} s after the last refresh");
    base.stop().await;
    drop(board);
    let _ = std::fs::remove_dir_all(&dir);
}

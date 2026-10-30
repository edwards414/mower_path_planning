//! The rear light end to end: `/mower_base/rear_light` on DDS, as path_record_node (through the bus bridge) and
//! mower_recorder write it, to the 0x03 frame mower_base writes the STM32, its overlay byte the `REAR_RECORDING`
//! bit while a recorder records, as `mower_hardware::MowerSystem` set it.
//!
//! - **The board** is a pty that only records what mower_base writes: the LED path needs no feedback.
//! - **DDS:** the recorders are a node of the test's own, on ROS domain 93, in the process's one context, so mower_base's
//!   reader matches them as it matches the recorders on the robot.

use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use mower_base_core::protocol::{self as proto, FrameParser};
use mower_rs_common::{ModuleCtx, Shutdown};
use r2r::std_msgs::msg::String as Text;
use r2r::{Parameter, ParameterValue, QosProfile};

const WAIT: Duration = Duration::from_secs(20);

fn context() -> r2r::Context {
    static CONTEXT: OnceLock<r2r::Context> = OnceLock::new();
    CONTEXT
        .get_or_init(|| {
            // rcl reads it when the context is created, and only then.
            std::env::set_var("ROS_DOMAIN_ID", "93");
            r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)")
        })
        .clone()
}

/// A raw pty, its slave linked at `link` for the driver to open; a thread keeps every 0x03 payload the driver
/// writes, with when.
struct Board {
    leds: Arc<Mutex<Vec<(Instant, Vec<u8>)>>>,
    running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Board {
    fn start(link: &Path) -> Board {
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
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(std::str::from_utf8(&name[..end]).unwrap(), link).unwrap();
        let leds = Arc::new(Mutex::new(Vec::new()));
        let running = Arc::new(AtomicBool::new(true));
        let thread = {
            let (leds, running) = (leds.clone(), running.clone());
            std::thread::spawn(move || {
                let _slave = slave;
                let mut parser = FrameParser::new();
                let mut buf = [0u8; 512];
                while running.load(Ordering::Relaxed) {
                    // SAFETY: read into a buffer we own, on a non-blocking descriptor.
                    let n = unsafe { libc::read(master.as_raw_fd(), buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
                    let now = Instant::now();
                    parser.feed(&buf[..n.max(0) as usize], |kind, _, payload| {
                        if kind == proto::WS2812_COMMAND {
                            leds.lock().unwrap().push((now, payload.to_vec()));
                        }
                    });
                    std::thread::sleep(Duration::from_millis(2));
                }
            })
        };
        Board { leds, running, thread: Some(thread) }
    }

    fn since(&self, t: Instant) -> Vec<Vec<u8>> {
        self.leds.lock().unwrap().iter().filter(|(at, _)| *at >= t).map(|(_, p)| p.clone()).collect()
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

async fn eventually(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + WAIT;
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn request(effect: &str, source: &str) -> Text {
    Text { data: format!(r#"{{"effect":"{effect}","source":"{source}"}}"#) }
}

/// Both recorders on DDS: the app's zone recording (path_record) and a bag (mower_recorder), each on its own
/// writer, latched as they write it. The overlay comes on with the first "recording", stays while either records,
/// and goes when the last one says "off"; one recorder that dies without its "off" leaves it on for
/// `rear_light_timeout_s` (here 1.5 s) after its last refresh, no longer.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn the_recorders_turn_the_rear_overlay_on_and_off() {
    let dir: PathBuf = std::env::temp_dir().join(format!("mower_base_rear_light_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let link = dir.join("stmcom");
    let board = Board::start(&link);

    let ctx = context();
    let node = r2r::Node::create(ctx.clone(), "mower_base", "").unwrap();
    for (key, value) in [
        ("device", ParameterValue::String(link.display().to_string())),
        ("rear_light_timeout_s", ParameterValue::Double(1.5)),
        ("shutdown_command", ParameterValue::String(String::new())),
    ] {
        node.params.lock().unwrap().insert(key.to_string(), Parameter::new(value));
    }
    let shutdown = Shutdown::new();
    let base = tokio::spawn(mower_base::run_node(node, ModuleCtx::new("mower_base", shutdown.clone())));

    // the recorders: a node of their own, R TL 1 writers as path_record_node's bridge and recorder_manager have
    let recorders = Arc::new(Mutex::new(r2r::Node::create(ctx, "rear_light_recorders", "").unwrap()));
    let latched = QosProfile::default().keep_last(1).reliable().transient_local();
    let (path_record, bag) = {
        let mut n = recorders.lock().unwrap();
        (
            n.create_publisher::<Text>("/mower_base/rear_light", latched.clone()).unwrap(),
            n.create_publisher::<Text>("/mower_base/rear_light", latched).unwrap(),
        )
    };
    let spinning = Arc::new(AtomicBool::new(true));
    let spinner = {
        let (recorders, spinning) = (recorders.clone(), spinning.clone());
        std::thread::spawn(move || {
            while spinning.load(Ordering::Relaxed) {
                recorders.lock().unwrap().spin_once(Duration::from_millis(5));
                std::thread::sleep(Duration::from_millis(1));
            }
        })
    };
    let readers = || recorders.lock().unwrap().get_subscriptions_info_by_topic("/mower_base/rear_light", false).unwrap().len();
    eventually("mower_base's reader", || readers() == 1).await;
    // the driver writes no 0x03 while nothing asked for a light
    tokio::time::sleep(Duration::from_millis(500)).await;
    assert!(board.since(Instant::now() - Duration::from_secs(10)).is_empty());

    let on = Instant::now();
    path_record.publish(&request("recording", "path_record")).unwrap();
    eventually("the overlay", || !board.since(on).is_empty()).await;
    assert_eq!(board.since(on)[0], [proto::LED_SHOW, 0, 0, 0, 0, 0, proto::LED_OVERLAY_REAR_RECORDING, 0]);

    // the bag records too; the app's recording stops: still on
    bag.publish(&request("recording", "bag")).unwrap();
    path_record.publish(&request("off", "path_record")).unwrap();
    tokio::time::sleep(Duration::from_millis(500)).await;
    assert!(board.since(on).iter().all(|p| p[6] == proto::LED_OVERLAY_REAR_RECORDING), "{:?}", board.since(on));
    // the bag stops: off at once
    let off = Instant::now();
    bag.publish(&request("off", "bag")).unwrap();
    eventually("the overlay to go", || board.since(off).iter().any(|p| p[6] == 0)).await;
    assert!(off.elapsed() < Duration::from_millis(500), "{:?}", off.elapsed());
    assert_eq!(board.since(off).last().unwrap(), &[proto::LED_SHOW, 0, 0, 0, 0, 0, 0, 0]);

    // a recorder that dies without its "off": the overlay holds 1.5 s past its last "recording", no longer
    let refresh = Instant::now();
    bag.publish(&request("recording", "bag")).unwrap();
    eventually("the overlay again", || board.since(refresh).iter().any(|p| p[6] == 1)).await;
    eventually("the timeout", || board.since(refresh).last().is_some_and(|p| p[6] == 0)).await;
    let held = refresh.elapsed();
    assert!(held >= Duration::from_millis(1400) && held < Duration::from_millis(2500), "{held:?}");

    spinning.store(false, Ordering::Relaxed);
    spinner.join().unwrap();
    shutdown.trigger();
    tokio::time::timeout(WAIT, base).await.unwrap().unwrap().unwrap();
    drop(board);
    let _ = std::fs::remove_dir_all(&dir);
}

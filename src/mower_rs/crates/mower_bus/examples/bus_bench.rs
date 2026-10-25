//! What a topic costs the daemon (README.md, "Topics"), for the loads it
//! carries: ~100 B messages at up to 100 Hz (IMU), a 210 KB OccupancyGrid at
//! 1 Hz, up to ~10 readers per topic, readers on tokio and on std threads.
//!
//! ```text
//! cargo run --release -p mower_bus --example bus_bench
//! ```
//!
//! - **publish cost:** a tight loop with readers that are present but not
//!   waiting. Their KEEP_LAST queues stay full, so every publish also pushes
//!   the oldest message out of each one.
//! - **wake-ups and latency:** one publish every 200 µs (paced, so the
//!   readers are always waiting, as they are at 100 Hz) to readers on a
//!   multi-thread tokio runtime or blocked in `recv_timeout` on std threads.
//!   The publish call then includes waking them.

use std::sync::{Arc, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_bus::{Bus, Message, Publisher, Qos};
use serde::{Deserialize, Serialize};

const GRID_BYTES: usize = 210_000;
const PERIOD: Duration = Duration::from_micros(200);
const SAMPLES: usize = 5_000;
/// Paced publishes left out of the statistics while threads and the
/// runtime settle.
const WARMUP: usize = 200;

/// sensor_msgs/Imu without the covariances: a stamp, a frame id and three
/// vectors, ~100 B.
#[derive(Clone, Serialize, Deserialize)]
struct Imu {
    sent_ns: u64,
    frame_id: String,
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
}

/// An OccupancyGrid-sized message.
#[derive(Clone, Serialize, Deserialize)]
struct Grid {
    sent_ns: u64,
    data: Vec<i8>,
}

/// A message that carries the time it was published, for the latency runs.
trait Stamped: Message {
    fn stamp(&mut self, ns: u64);
    fn sent_ns(&self) -> u64;
}

impl Stamped for Imu {
    fn stamp(&mut self, ns: u64) {
        self.sent_ns = ns;
    }
    fn sent_ns(&self) -> u64 {
        self.sent_ns
    }
}

impl Stamped for Grid {
    fn stamp(&mut self, ns: u64) {
        self.sent_ns = ns;
    }
    fn sent_ns(&self) -> u64 {
        self.sent_ns
    }
}

fn imu() -> Imu {
    Imu {
        sent_ns: 0,
        frame_id: "imu_link".into(),
        orientation: [0.0, 0.0, 0.38, 0.92],
        angular_velocity: [0.01, -0.02, 0.3],
        linear_acceleration: [0.1, 0.05, 9.81],
    }
}

fn grid() -> Grid {
    Grid { sent_ns: 0, data: vec![0; GRID_BYTES] }
}

/// Nanoseconds since the first call: a clock both sides of a latency run
/// read.
fn now_ns() -> u64 {
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    ORIGIN.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

fn main() {
    let cpus = thread::available_parallelism().map_or(0, |n| n.get());
    let build = if cfg!(debug_assertions) { "debug build: run with --release" } else { "release build" };
    println!("mower_bus topic bench, {build}, {} with {cpus} CPUs", std::env::consts::ARCH);
    println!(
        "Imu: {} B plus a {} B frame id; Grid: {} B of cells",
        std::mem::size_of::<Imu>(),
        imu().frame_id.len(),
        GRID_BYTES
    );

    println!();
    println!("publish cost, ns per message (median of 7 runs; readers not waiting, queues full)");
    println!("{:>8}  {:>14}  {:>18}  {:>27}", "readers", "publish(&Imu)", "publish_arc(Grid)", "publish(&Grid), one copy");
    let small = imu();
    let big = Arc::new(grid());
    for readers in [0, 1, 4] {
        let imu_ns = publish_cost::<Imu>(readers, 200_000, |p| p.publish(&small).unwrap());
        let arc_ns = publish_cost::<Grid>(readers, 200_000, |p| p.publish_arc(Arc::clone(&big)).unwrap());
        let copy_ns = publish_cost::<Grid>(readers, 2_000, |p| p.publish(&big).unwrap());
        println!("{readers:>8}  {imu_ns:>14.0}  {arc_ns:>18.0}  {copy_ns:>27.0}");
    }

    println!();
    println!(
        "waiting readers: one publish every {} µs, {SAMPLES} samples (latency over all readers)",
        PERIOD.as_micros()
    );
    println!("{:<34}  {:>24}  {:>36}", "", "publish call p50 / p99", "publish -> receive p50 / p99 / max");
    let runs: [(&str, Readers, bool); 6] = [
        ("Imu,  1 tokio task", Readers::Tokio(1), false),
        ("Imu,  4 tokio tasks", Readers::Tokio(4), false),
        ("Imu,  1 std thread (recv_timeout)", Readers::Threads(1), false),
        ("Imu,  4 std threads (recv_timeout)", Readers::Threads(4), false),
        ("Grid, 1 tokio task", Readers::Tokio(1), true),
        ("Grid, 1 std thread (recv_timeout)", Readers::Threads(1), true),
    ];
    for (label, readers, is_grid) in runs {
        let (mut calls, mut latency) = if is_grid { paced(readers, grid) } else { paced(readers, imu) };
        let (c50, c99) = (percentile(&mut calls, 0.50), percentile(&mut calls, 0.99));
        let (l50, l99) = (percentile(&mut latency, 0.50), percentile(&mut latency, 0.99));
        let lmax = percentile(&mut latency, 1.0);
        println!(
            "{label:<34}  {:>24}  {:>36}",
            format!("{c50} ns / {c99} ns"),
            format!("{:.1} / {:.1} / {:.1} µs", us(l50), us(l99), us(lmax))
        );
    }
}

/// Median over 7 runs of `iterations` publishes, in ns per publish.
fn publish_cost<T: Message>(readers: usize, iterations: u32, mut publish: impl FnMut(&Publisher<T>)) -> f64 {
    let bus = Bus::new();
    let scope = bus.scope("bench");
    let p = scope.publisher::<T>("/bench", Qos::default()).unwrap();
    let _readers: Vec<_> =
        (0..readers).map(|_| scope.subscribe::<T>("/bench", Qos::sensor_data()).unwrap()).collect();
    for _ in 0..iterations.min(10_000) {
        publish(&p); // fill the queues, warm the allocator
    }
    let mut runs: Vec<f64> = (0..7)
        .map(|_| {
            let t = Instant::now();
            for _ in 0..iterations {
                publish(&p);
            }
            t.elapsed().as_nanos() as f64 / f64::from(iterations)
        })
        .collect();
    runs.sort_by(f64::total_cmp);
    runs[runs.len() / 2]
}

#[derive(Clone, Copy)]
enum Readers {
    Tokio(usize),
    Threads(usize),
}

enum Reader {
    Task(tokio::task::JoinHandle<Vec<u64>>),
    Thread(thread::JoinHandle<Vec<u64>>),
}

/// Publish `SAMPLES + WARMUP` messages from this thread, one per `PERIOD`,
/// to `readers` that wait for them. Each message is built first (the Grid's
/// 210 KB copy is not timed), then stamped and moved in with
/// `publish_owned`. Returns the publish calls' durations and the readers'
/// publish -> receive latencies, in ns, warm-up left out.
///
/// The readers' queues hold every message: at this rate the default depth
/// of 10 is 2 ms of slack, and a reader the OS leaves off-CPU for longer
/// would lose messages (counted in `dropped()`, as it should) and then
/// wait here for them.
fn paced<T: Stamped>(readers: Readers, build: fn() -> T) -> (Vec<u64>, Vec<u64>) {
    let total = SAMPLES + WARMUP;
    let qos = Qos::default().keep_last(total);
    let bus = Bus::new();
    let scope = bus.scope("bench");
    let p = scope.publisher::<T>("/bench", Qos::default()).unwrap();
    let rt = tokio::runtime::Builder::new_multi_thread().worker_threads(4).enable_time().build().unwrap();
    let receivers: Vec<Reader> = match readers {
        Readers::Tokio(n) => (0..n)
            .map(|_| {
                let mut sub = scope.subscribe::<T>("/bench", qos).unwrap();
                Reader::Task(rt.spawn(async move {
                    let mut latency = Vec::with_capacity(total);
                    while latency.len() < total {
                        let msg = tokio::time::timeout(Duration::from_secs(5), sub.next()).await;
                        let msg = msg.expect("a message within 5 s").expect("open");
                        latency.push(now_ns() - msg.sent_ns());
                    }
                    latency
                }))
            })
            .collect(),
        Readers::Threads(n) => (0..n)
            .map(|_| {
                let mut sub = scope.subscribe::<T>("/bench", qos).unwrap();
                Reader::Thread(thread::spawn(move || {
                    let mut latency = Vec::with_capacity(total);
                    while latency.len() < total {
                        let msg = sub.recv_timeout(Duration::from_secs(5)).expect("a message within 5 s");
                        latency.push(now_ns() - msg.sent_ns());
                    }
                    latency
                }))
            })
            .collect(),
    };
    thread::sleep(Duration::from_millis(50)); // let the readers park

    let mut calls = Vec::with_capacity(total);
    let start = Instant::now();
    for i in 0..total {
        let mut msg = build();
        if let Some(wait) = (start + PERIOD * i as u32).checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        }
        let t = Instant::now();
        msg.stamp(now_ns());
        p.publish_owned(msg).unwrap();
        calls.push(t.elapsed().as_nanos() as u64);
    }

    let mut latency = Vec::new();
    for reader in receivers {
        let got = match reader {
            Reader::Task(task) => rt.block_on(task).unwrap(),
            Reader::Thread(thread) => thread.join().unwrap(),
        };
        latency.extend_from_slice(&got[WARMUP..]);
    }
    calls.drain(..WARMUP);
    (calls, latency)
}

fn percentile(values: &mut [u64], p: f64) -> u64 {
    values.sort_unstable();
    values[((values.len() - 1) as f64 * p).round() as usize]
}

fn us(ns: u64) -> f64 {
    ns as f64 / 1_000.0
}

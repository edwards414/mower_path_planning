//! The `host` block of `/robot/telemetry`: what `top`, `free` and `df` show,
//! sampled every two seconds from /proc and /sys.
//!
//! The container runs with `pid: host`, so /proc lists every process on the
//! robot, and /proc/stat, /proc/meminfo, /proc/diskstats and /sys are the
//! host's anyway. Disk capacity is statvfs on paths the container can see: the
//! state dir (bind-mounted from the host), its `bags/` directory (where an
//! extra drive would be mounted) and `/` (the overlay, which reports the
//! filesystem under /var/lib/containerd) when neither exists.
//!
//! Rates (CPU %, per-process %, disk throughput) are differences between two
//! samples, so they are `null` in the first one. Per-process CPU is in % of
//! one core like `top` (4 cores = 400 %), measured against the jiffies that
//! /proc/stat says elapsed, so USER_HZ never has to be known. Scanning
//! ~200 processes costs about 2 ms every two seconds on the LubanCat.

use std::collections::HashMap;
use std::ffi::CString;
use std::fs;
use std::path::Path;
use std::time::Instant;

use mower_rs_common::{json_number, round_to};
use serde_json::{json, Value};

/// How often the block is refreshed.
pub const PERIOD_S: f64 = 2.0;
/// Rows in `procs`, busiest first.
pub const TOP_PROCESSES: usize = 10;

// ---- /proc/stat --------------------------------------------------------------

/// One `cpu` line of /proc/stat, in jiffies.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CpuTimes {
    pub user: u64,
    pub nice: u64,
    pub system: u64,
    pub idle: u64,
    pub iowait: u64,
    pub irq: u64,
    pub softirq: u64,
    pub steal: u64,
}

impl CpuTimes {
    /// guest and guest_nice are already inside user and nice.
    pub fn total(&self) -> u64 {
        self.user + self.nice + self.system + self.idle + self.iowait + self.irq + self.softirq + self.steal
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Stat {
    pub all: CpuTimes,
    /// (cpu index, times); an offline core has no line.
    pub cores: Vec<(usize, CpuTimes)>,
    pub procs_running: Option<u64>,
}

fn parse_cpu_times(fields: &[&str]) -> CpuTimes {
    let f = |i: usize| fields.get(i).and_then(|v| v.parse::<u64>().ok()).unwrap_or(0);
    CpuTimes {
        user: f(0),
        nice: f(1),
        system: f(2),
        idle: f(3),
        iowait: f(4),
        irq: f(5),
        softirq: f(6),
        steal: f(7),
    }
}

pub fn parse_stat(text: &str) -> Stat {
    let mut st = Stat::default();
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let Some(key) = parts.next() else { continue };
        let rest: Vec<&str> = parts.collect();
        if key == "cpu" {
            st.all = parse_cpu_times(&rest);
        } else if let Some(idx) = key.strip_prefix("cpu").and_then(|n| n.parse::<usize>().ok()) {
            st.cores.push((idx, parse_cpu_times(&rest)));
        } else if key == "procs_running" {
            st.procs_running = rest.first().and_then(|v| v.parse().ok());
        }
    }
    st
}

/// `top`'s %Cpu(s) line between two samples. `busy` leaves out idle and
/// iowait (a core waiting on the disk is free to run something else).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CpuPct {
    pub busy: f64,
    pub user: f64,
    pub system: f64,
    pub iowait: f64,
    pub irq: f64,
    pub steal: f64,
}

pub fn cpu_pct(prev: &CpuTimes, cur: &CpuTimes) -> Option<CpuPct> {
    let dt = cur.total().saturating_sub(prev.total());
    if dt == 0 {
        return None;
    }
    let pct = |a: u64, b: u64| 100.0 * b.saturating_sub(a) as f64 / dt as f64;
    let idle = pct(prev.idle, cur.idle);
    let iowait = pct(prev.iowait, cur.iowait);
    Some(CpuPct {
        busy: (100.0 - idle - iowait).clamp(0.0, 100.0),
        user: pct(prev.user + prev.nice, cur.user + cur.nice),
        system: pct(prev.system, cur.system),
        iowait,
        irq: pct(prev.irq + prev.softirq, cur.irq + cur.softirq),
        steal: pct(prev.steal, cur.steal),
    })
}

// ---- /proc/meminfo, /proc/loadavg -------------------------------------------

/// /proc/meminfo in kB, keyed by field name.
pub fn parse_meminfo(text: &str) -> HashMap<String, u64> {
    let mut m = HashMap::new();
    for line in text.lines() {
        let Some((k, v)) = line.split_once(':') else { continue };
        if let Some(n) = v.split_whitespace().next().and_then(|n| n.parse::<u64>().ok()) {
            m.insert(k.trim().to_string(), n);
        }
    }
    m
}

/// The `mem` block the way `free` computes it: used = total - free -
/// buffers - cache, where cache includes reclaimable slab.
pub fn mem_block(m: &HashMap<String, u64>) -> Value {
    let get = |k: &str| m.get(k).copied();
    let Some(total) = get("MemTotal") else { return Value::Null };
    let free = get("MemFree").unwrap_or(0);
    let buffers = get("Buffers").unwrap_or(0);
    let cache = get("Cached").unwrap_or(0) + get("SReclaimable").unwrap_or(0);
    let used = total.saturating_sub(free + buffers + cache);
    let avail = get("MemAvailable").unwrap_or(free);
    let swap_total = get("SwapTotal").unwrap_or(0);
    let swap_used = swap_total.saturating_sub(get("SwapFree").unwrap_or(0));
    let mb = |kb: u64| json_number(round_to(kb as f64 / 1024.0, 1));
    json!({
        "total_mb": mb(total),
        "used_mb": mb(used),
        "cache_mb": mb(buffers + cache),
        "free_mb": mb(free),
        "avail_mb": mb(avail),
        "swap_total_mb": mb(swap_total),
        "swap_used_mb": mb(swap_used),
    })
}

/// (load1, load5, load15, threads); the fourth field is `running/total`
/// scheduling entities, i.e. threads.
pub fn parse_loadavg(text: &str) -> (Option<f64>, Option<f64>, Option<f64>, Option<u64>) {
    let f: Vec<&str> = text.split_whitespace().collect();
    let num = |i: usize| f.get(i).and_then(|v| v.parse::<f64>().ok());
    let threads = f.get(3).and_then(|v| v.split('/').nth(1)).and_then(|v| v.parse().ok());
    (num(0), num(1), num(2), threads)
}

// ---- processes ----------------------------------------------------------------

/// The fields of /proc/<pid>/stat this block uses.
#[derive(Clone, Debug, PartialEq)]
pub struct ProcStat {
    pub comm: String,
    pub state: char,
    /// utime + stime, jiffies.
    pub ticks: u64,
    pub threads: u64,
    /// Jiffies after boot; with the pid it tells a reused pid apart.
    pub start: u64,
}

pub fn parse_proc_stat(text: &str) -> Option<ProcStat> {
    // comm is in parentheses and may itself contain spaces and ')'.
    let open = text.find('(')?;
    let close = text.rfind(')')?;
    let comm = text.get(open + 1..close)?.to_string();
    let rest: Vec<&str> = text.get(close + 1..)?.split_whitespace().collect();
    // rest[0] is field 3 (state); field n is rest[n - 3].
    let field = |n: usize| rest.get(n - 3).and_then(|v| v.parse::<u64>().ok());
    Some(ProcStat {
        comm,
        state: rest.first()?.chars().next()?,
        ticks: field(14)? + field(15)?,
        threads: field(20).unwrap_or(0),
        start: field(22).unwrap_or(0),
    })
}

fn basename(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

/// A readable process name plus its ROS node name, if any. comm is cut at 15
/// characters and is `python3` for every rclpy node, so the command line
/// decides where it can: `__node:=` for the node, the script for Python, the
/// untruncated binary name otherwise. Kernel threads have no command line.
pub fn process_label(comm: &str, cmdline: &[u8]) -> (String, Option<String>) {
    let args: Vec<String> = cmdline
        .split(|&b| b == 0)
        .filter(|a| !a.is_empty())
        .map(|a| String::from_utf8_lossy(a).into_owned())
        .collect();
    let node = args
        .iter()
        .find_map(|a| a.strip_prefix("__node:=").map(str::to_string))
        .filter(|n| !n.is_empty());
    let Some(argv0) = args.first() else { return (comm.to_string(), node) };
    let exe = basename(argv0);
    let name = if exe.starts_with("python") {
        let mut it = args.iter().skip(1);
        let mut script = None;
        while let Some(a) = it.next() {
            if a == "-m" {
                script = it.next().map(|m| m.to_string());
                break;
            }
            if a.starts_with('-') {
                continue;
            }
            script = Some(basename(a).trim_end_matches(".py").to_string());
            break;
        }
        script.unwrap_or_else(|| comm.to_string())
    } else if !comm.is_empty() && exe.starts_with(comm) {
        exe.to_string()
    } else {
        comm.to_string()
    };
    (name, node)
}

/// VmRSS in kB from /proc/<pid>/status (0 for kernel threads).
pub fn parse_status_rss_kb(text: &str) -> u64 {
    text.lines()
        .find_map(|l| l.strip_prefix("VmRSS:"))
        .and_then(|v| v.split_whitespace().next())
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

// ---- disks --------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DiskCounters {
    pub read_sectors: u64,
    pub write_sectors: u64,
    /// Milliseconds with at least one request in flight.
    pub io_ms: u64,
}

pub fn parse_diskstats(text: &str) -> Vec<(String, DiskCounters)> {
    let mut out = Vec::new();
    for line in text.lines() {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() < 14 {
            continue;
        }
        let n = |i: usize| f[i].parse::<u64>().unwrap_or(0);
        out.push((
            f[2].to_string(),
            DiskCounters { read_sectors: n(5), write_sectors: n(9), io_ms: n(12) },
        ));
    }
    out
}

/// Whole drives worth showing: not partitions (those have no /sys/block
/// entry), loop/ram/zram devices or the eMMC boot and RPMB areas.
pub fn is_shown_disk(name: &str) -> bool {
    !(name.starts_with("loop")
        || name.starts_with("ram")
        || name.starts_with("zram")
        || (name.starts_with("mmcblk") && (name.contains("boot") || name.contains("rpmb"))))
}

/// One /proc/self/mountinfo line: (mount point, fstype, source).
pub fn parse_mountinfo(text: &str) -> Vec<(String, String, String)> {
    let unescape = |s: &str| s.replace("\\040", " ").replace("\\011", "\t").replace("\\012", "\n").replace("\\134", "\\");
    let mut out = Vec::new();
    for line in text.lines() {
        let Some((left, right)) = line.split_once(" - ") else { continue };
        let l: Vec<&str> = left.split_whitespace().collect();
        let r: Vec<&str> = right.split_whitespace().collect();
        if l.len() < 5 || r.len() < 2 {
            continue;
        }
        out.push((unescape(l[4]), r[0].to_string(), unescape(r[1])));
    }
    out
}

/// The mount a path lives on: the longest mount point that is a path prefix.
pub fn mount_for<'a>(mounts: &'a [(String, String, String)], path: &str) -> Option<&'a (String, String, String)> {
    mounts
        .iter()
        .filter(|(mp, _, _)| {
            mp == "/" || path == mp || path.strip_prefix(mp.as_str()).is_some_and(|r| r.starts_with('/'))
        })
        .max_by_key(|(mp, _, _)| mp.len())
}

/// statvfs(path) -> (fragment size, blocks, blocks free, blocks available).
fn statvfs(path: &str) -> Option<(u64, u64, u64, u64)> {
    let c = CString::new(path).ok()?;
    // SAFETY: statvfs only writes into the zeroed struct we own.
    let mut s: libc::statvfs = unsafe { std::mem::zeroed() };
    if unsafe { libc::statvfs(c.as_ptr(), &mut s) } != 0 {
        return None;
    }
    Some((s.f_frsize as u64, s.f_blocks as u64, s.f_bfree as u64, s.f_bavail as u64))
}

/// `df` for one filesystem. used % is used / (used + available) as df has it
/// (root's reserved blocks count as neither).
pub fn disk_entry(role: &str, path: &str, frsize: u64, blocks: u64, bfree: u64, bavail: u64, fstype: &str, source: &str) -> Value {
    let gb = |b: u64| json_number(round_to((b as f64) * frsize as f64 / 1e9, 2));
    let used = blocks.saturating_sub(bfree);
    let denom = used + bavail;
    let used_pct = if denom > 0 { Some(round_to(100.0 * used as f64 / denom as f64, 1)) } else { None };
    json!({
        "role": role,
        "path": path,
        "dev": basename(source),
        "fstype": fstype,
        "total_gb": gb(blocks),
        "used_gb": gb(used),
        "avail_gb": gb(bavail),
        "used_pct": used_pct.map(json_number).unwrap_or(Value::Null),
    })
}

fn read(path: impl AsRef<Path>) -> Option<String> {
    fs::read_to_string(path).ok()
}

fn read_trim(path: impl AsRef<Path>) -> Option<String> {
    read(path).map(|s| s.trim().to_string()).filter(|s| !s.is_empty())
}

fn parse_hex(s: &str) -> Option<u64> {
    u64::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok()
}

/// cpu list like "0-3" or "0 1 2 3" or "0-1,4".
pub fn parse_cpu_list(s: &str) -> Vec<u64> {
    let mut out = Vec::new();
    for part in s.split([',', ' ', '\n']).filter(|p| !p.is_empty()) {
        match part.split_once('-') {
            Some((a, b)) => {
                if let (Ok(a), Ok(b)) = (a.parse::<u64>(), b.parse::<u64>()) {
                    out.extend(a..=b);
                }
            }
            None => {
                if let Ok(v) = part.parse() {
                    out.push(v);
                }
            }
        }
    }
    out
}

// ---- sampler --------------------------------------------------------------------

struct Prev {
    t: Instant,
    stat: Stat,
    /// (pid, start) -> ticks
    procs: HashMap<(u32, u64), u64>,
    disks: HashMap<String, DiskCounters>,
}

/// Keeps the previous sample for the rates; owned by the telemetry task.
pub struct Sampler {
    state_dir: String,
    last: Option<Instant>,
    prev: Option<Prev>,
}

impl Sampler {
    pub fn new(state_dir: &str) -> Self {
        Self { state_dir: state_dir.to_string(), last: None, prev: None }
    }

    /// A fresh `host` block when [`PERIOD_S`] has passed since the last one.
    pub fn poll(&mut self, now: Instant) -> Option<Value> {
        if let Some(t) = self.last {
            if now.duration_since(t).as_secs_f64() < PERIOD_S {
                return None;
            }
        }
        self.last = Some(now);
        Some(self.sample(now))
    }

    fn sample(&mut self, now: Instant) -> Value {
        let stat = read("/proc/stat").map(|s| parse_stat(&s)).unwrap_or_default();
        let meminfo = read("/proc/meminfo").map(|s| parse_meminfo(&s)).unwrap_or_default();
        let (load1, load5, load15, threads) =
            read("/proc/loadavg").map(|s| parse_loadavg(&s)).unwrap_or((None, None, None, None));
        let mem_total_kb = meminfo.get("MemTotal").copied();
        let mem_used_pct = match (mem_total_kb, meminfo.get("MemAvailable")) {
            (Some(t), Some(a)) if t > 0 => Some(round_to(100.0 * t.saturating_sub(*a) as f64 / t as f64, 1)),
            _ => None,
        };
        let cpu_temp_c = read_trim("/sys/class/thermal/thermal_zone0/temp")
            .and_then(|s| s.parse::<f64>().ok())
            .map(|m| round_to(m / 1000.0, 1));
        let boot_uptime_s = read("/proc/uptime")
            .and_then(|s| s.split_whitespace().next().and_then(|v| v.parse::<f64>().ok()));

        let prev = self.prev.as_ref();
        let dt_s = prev.map(|p| now.duration_since(p.t).as_secs_f64());

        // CPU
        let total = prev.and_then(|p| cpu_pct(&p.stat.all, &stat.all));
        let cores: Vec<Value> = stat
            .cores
            .iter()
            .map(|(i, cur)| {
                prev.and_then(|p| p.stat.cores.iter().find(|(j, _)| j == i))
                    .and_then(|(_, old)| cpu_pct(old, cur))
                    .map(|c| json_number(round_to(c.busy, 1)))
                    .unwrap_or(Value::Null)
            })
            .collect();
        let r1 = |v: f64| json_number(round_to(v, 1));
        let cpu = json!({
            "pct": total.map(|c| r1(c.busy)).unwrap_or(Value::Null),
            "user": total.map(|c| r1(c.user)).unwrap_or(Value::Null),
            "system": total.map(|c| r1(c.system)).unwrap_or(Value::Null),
            "iowait": total.map(|c| r1(c.iowait)).unwrap_or(Value::Null),
            "irq": total.map(|c| r1(c.irq)).unwrap_or(Value::Null),
            "steal": total.map(|c| r1(c.steal)).unwrap_or(Value::Null),
            "cores": cores,
            "freq": freq_policies(),
            "temps_c": thermal_zones(),
        });

        // Processes: per-CPU jiffies that elapsed between the two samples.
        let ncpu = stat.cores.len().max(1) as f64;
        let elapsed_jiffies = prev
            .map(|p| stat.all.total().saturating_sub(p.stat.all.total()) as f64 / ncpu)
            .filter(|j| *j > 0.0);
        let mut procs_now: HashMap<(u32, u64), u64> = HashMap::new();
        let mut rows: Vec<(f64, u32, ProcStat)> = Vec::new();
        if let Ok(dir) = fs::read_dir("/proc") {
            for e in dir.flatten() {
                let Some(pid) = e.file_name().to_str().and_then(|n| n.parse::<u32>().ok()) else { continue };
                let Some(ps) = read(format!("/proc/{pid}/stat")).and_then(|s| parse_proc_stat(&s)) else { continue };
                let key = (pid, ps.start);
                procs_now.insert(key, ps.ticks);
                let pct = match (prev.and_then(|p| p.procs.get(&key)), elapsed_jiffies) {
                    (Some(old), Some(j)) => 100.0 * ps.ticks.saturating_sub(*old) as f64 / j,
                    _ => 0.0,
                };
                rows.push((pct, pid, ps));
            }
        }
        let process_count = rows.len();
        rows.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.2.ticks.cmp(&a.2.ticks)));
        let procs: Vec<Value> = rows
            .iter()
            .take(TOP_PROCESSES)
            .map(|(pct, pid, ps)| {
                let rss_kb = read(format!("/proc/{pid}/status")).map(|s| parse_status_rss_kb(&s)).unwrap_or(0);
                let cmdline = fs::read(format!("/proc/{pid}/cmdline")).unwrap_or_default();
                let (name, node) = process_label(&ps.comm, &cmdline);
                let mem_pct = mem_total_kb.filter(|t| *t > 0).map(|t| round_to(100.0 * rss_kb as f64 / t as f64, 1));
                json!({
                    "pid": pid,
                    "name": name,
                    "node": node,
                    "state": ps.state.to_string(),
                    "cpu": if elapsed_jiffies.is_some() { json_number(round_to(*pct, 1)) } else { Value::Null },
                    "mem": mem_pct.map(json_number).unwrap_or(Value::Null),
                    "rss_mb": json_number(round_to(rss_kb as f64 / 1024.0, 1)),
                    "threads": ps.threads,
                })
            })
            .collect();

        // Disks: capacity of what the container can see, throughput per drive.
        let mounts = read("/proc/self/mountinfo").map(|s| parse_mountinfo(&s)).unwrap_or_default();
        let mut disks = Vec::new();
        let mut seen: Vec<(u64, u64)> = Vec::new();
        let bags = format!("{}/bags", self.state_dir.trim_end_matches('/'));
        for (role, path) in [("state", self.state_dir.as_str()), ("bags", bags.as_str()), ("root", "/")] {
            let Some((frsize, blocks, bfree, bavail)) = statvfs(path) else { continue };
            if blocks == 0 || seen.contains(&(frsize, blocks)) {
                continue;
            }
            seen.push((frsize, blocks));
            let (fstype, source) = mount_for(&mounts, path)
                .map(|(_, t, s)| (t.as_str(), s.as_str()))
                .unwrap_or(("", ""));
            disks.push(disk_entry(role, path, frsize, blocks, bfree, bavail, fstype, source));
        }
        let counters: HashMap<String, DiskCounters> = read("/proc/diskstats")
            .map(|s| parse_diskstats(&s))
            .unwrap_or_default()
            .into_iter()
            .filter(|(name, _)| is_shown_disk(name) && Path::new("/sys/block").join(name).exists())
            .collect();
        let mut names: Vec<&String> = counters.keys().collect();
        names.sort();
        let io: Vec<Value> = names
            .into_iter()
            .map(|name| {
                let cur = counters[name];
                let rate = prev.and_then(|p| p.disks.get(name)).zip(dt_s).filter(|(_, dt)| *dt > 0.0);
                let kbs = |a: u64, b: u64, dt: f64| json_number(round_to(b.saturating_sub(a) as f64 * 512.0 / 1024.0 / dt, 1));
                let sys = Path::new("/sys/block").join(name);
                let size_gb = read_trim(sys.join("size"))
                    .and_then(|s| s.parse::<u64>().ok())
                    .map(|sectors| json_number(round_to(sectors as f64 * 512.0 / 1e9, 1)))
                    .unwrap_or(Value::Null);
                let model = read_trim(sys.join("device/model")).or_else(|| read_trim(sys.join("device/name")));
                // eMMC wear estimate (JEDEC): 0x01 = 0-10 % used ... 0x0B = exceeded.
                let life: Vec<u64> = read(sys.join("device/life_time"))
                    .map(|s| s.split_whitespace().filter_map(parse_hex).collect())
                    .unwrap_or_default();
                let pre_eol = read_trim(sys.join("device/pre_eol_info")).and_then(|s| parse_hex(&s));
                json!({
                    "dev": name,
                    "model": model,
                    "size_gb": size_gb,
                    "read_kbs": rate.map(|(old, dt)| kbs(old.read_sectors, cur.read_sectors, dt)).unwrap_or(Value::Null),
                    "write_kbs": rate.map(|(old, dt)| kbs(old.write_sectors, cur.write_sectors, dt)).unwrap_or(Value::Null),
                    "util_pct": rate
                        .map(|(old, dt)| json_number(round_to((100.0 * cur.io_ms.saturating_sub(old.io_ms) as f64 / (dt * 1000.0)).min(100.0), 1)))
                        .unwrap_or(Value::Null),
                    "life_time": if life.is_empty() { Value::Null } else { json!(life) },
                    "pre_eol": pre_eol,
                })
            })
            .collect();

        let opt = |v: Option<f64>| v.map(json_number).unwrap_or(Value::Null);
        let doc = json!({
            "load1": opt(load1),
            "load5": opt(load5),
            "load15": opt(load15),
            "mem_used_pct": opt(mem_used_pct),
            "cpu_temp_c": opt(cpu_temp_c),
            "boot_uptime_s": opt(boot_uptime_s.map(|v| round_to(v, 0))),
            "sample_s": opt(dt_s.map(|v| round_to(v, 2))),
            "cpu": cpu,
            "mem": mem_block(&meminfo),
            "disks": disks,
            "io": io,
            "tasks": {
                "procs": process_count,
                "threads": threads,
                "running": stat.procs_running,
            },
            "procs": procs,
        });
        self.prev = Some(Prev { t: now, stat, procs: procs_now, disks: counters });
        doc
    }
}

/// cpufreq policies: current, hardware maximum and the current limit (the
/// thermal governor caps `scaling_max_freq` when the SoC is hot).
fn freq_policies() -> Vec<Value> {
    let Ok(dir) = fs::read_dir("/sys/devices/system/cpu/cpufreq") else { return Vec::new() };
    let mut policies: Vec<_> = dir
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("policy"))
        .map(|e| e.path())
        .collect();
    policies.sort();
    let mhz = |p: &Path, f: &str| {
        read_trim(p.join(f))
            .and_then(|s| s.parse::<f64>().ok())
            .map(|khz| json_number((khz / 1000.0).round()))
            .unwrap_or(Value::Null)
    };
    policies
        .iter()
        .map(|p| {
            json!({
                "cpus": read_trim(p.join("related_cpus")).map(|s| parse_cpu_list(&s)).unwrap_or_default(),
                "cur_mhz": mhz(p, "scaling_cur_freq"),
                "max_mhz": mhz(p, "cpuinfo_max_freq"),
                "limit_mhz": mhz(p, "scaling_max_freq"),
            })
        })
        .collect()
}

/// Every thermal zone by its type (`soc-thermal`, `gpu-thermal` on the RK3568).
fn thermal_zones() -> Value {
    let mut out = serde_json::Map::new();
    let Ok(dir) = fs::read_dir("/sys/class/thermal") else { return Value::Object(out) };
    let mut zones: Vec<_> = dir
        .flatten()
        .filter(|e| e.file_name().to_string_lossy().starts_with("thermal_zone"))
        .map(|e| e.path())
        .collect();
    zones.sort();
    for z in zones {
        let (Some(kind), Some(temp)) = (read_trim(z.join("type")), read_trim(z.join("temp")).and_then(|s| s.parse::<f64>().ok())) else {
            continue;
        };
        out.insert(kind, json_number(round_to(temp / 1000.0, 1)));
    }
    Value::Object(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    const STAT_A: &str = "cpu  100 0 50 800 10 0 5 0 0 0\n\
cpu0 25 0 12 200 3 0 1 0 0 0\n\
cpu1 25 0 13 200 2 0 1 0 0 0\n\
intr 1 2 3\n\
procs_running 3\n";
    const STAT_B: &str = "cpu  160 0 70 900 20 0 10 0 0 0\n\
cpu0 55 0 22 250 8 0 5 0 0 0\n\
cpu1 55 0 23 250 7 0 1 0 0 0\n\
procs_running 2\n";

    #[test]
    fn stat_lines_and_top_percentages() {
        let a = parse_stat(STAT_A);
        let b = parse_stat(STAT_B);
        assert_eq!(a.cores.len(), 2);
        assert_eq!(a.procs_running, Some(3));
        assert_eq!(a.all.total(), 965);
        // delta: user 60, system 20, idle 100, iowait 10, softirq 5 = 195
        let p = cpu_pct(&a.all, &b.all).unwrap();
        assert!((p.user - 100.0 * 60.0 / 195.0).abs() < 1e-9);
        assert!((p.busy - 100.0 * 85.0 / 195.0).abs() < 1e-9);
        assert!((p.iowait - 100.0 * 10.0 / 195.0).abs() < 1e-9);
        assert!((p.irq - 100.0 * 5.0 / 195.0).abs() < 1e-9);
        assert_eq!(cpu_pct(&a.all, &a.all), None);
    }

    #[test]
    fn meminfo_like_free() {
        let m = parse_meminfo(
            "MemTotal:        3988480 kB\nMemFree:         2444288 kB\nMemAvailable:    3362816 kB\n\
Buffers:           40960 kB\nCached:           880640 kB\nSwapCached:            0 kB\n\
SReclaimable:      51200 kB\nSwapTotal:             0 kB\nSwapFree:              0 kB\n",
        );
        let b = mem_block(&m);
        assert_eq!(b["total_mb"], json!(3895.0));
        // 3988480 - 2444288 - 40960 - 931840 = 571392 kB
        assert_eq!(b["used_mb"], json!(558.0));
        assert_eq!(b["cache_mb"], json!(950.0));
        assert_eq!(b["avail_mb"], json!(3284.0));
        assert_eq!(b["swap_total_mb"], json!(0.0));
        assert_eq!(mem_block(&HashMap::new()), Value::Null);
    }

    #[test]
    fn loadavg_fields() {
        assert_eq!(parse_loadavg("2.53 2.65 2.23 3/412 12345\n"), (Some(2.53), Some(2.65), Some(2.23), Some(412)));
        assert_eq!(parse_loadavg(""), (None, None, None, None));
    }

    #[test]
    fn proc_stat_with_awkward_comm() {
        let line = "1234 (a b) c)) S 1 1234 1234 0 -1 4194560 100 0 0 0 250 50 0 0 20 0 7 0 4242 123456 789 1844674 0";
        let ps = parse_proc_stat(line).unwrap();
        assert_eq!(ps.comm, "a b) c)");
        assert_eq!(ps.state, 'S');
        assert_eq!(ps.ticks, 300);
        assert_eq!(ps.threads, 7);
        assert_eq!(ps.start, 4242);
        assert!(parse_proc_stat("garbage").is_none());
    }

    #[test]
    fn labels_prefer_node_script_and_full_binary_name() {
        let cmd = |args: &[&str]| args.join("\0").into_bytes();
        assert_eq!(
            process_label("ros2_control_no", &cmd(&["/opt/ros/jazzy/lib/controller_manager/ros2_control_node", "--ros-args"])),
            ("ros2_control_node".to_string(), None)
        );
        assert_eq!(
            process_label("ekf_node", &cmd(&["/opt/ros/jazzy/lib/robot_localization/ekf_node", "--ros-args", "-r", "__node:=ekf_filter_node_odom"])),
            ("ekf_node".to_string(), Some("ekf_filter_node_odom".to_string()))
        );
        assert_eq!(
            process_label("python3", &cmd(&["/usr/bin/python3", "-u", "/ws/install/lib/mower_mission/telemetry_node.py", "--ros-args"])),
            ("telemetry_node".to_string(), None)
        );
        assert_eq!(process_label("python3", &cmd(&["python3", "-m", "http.server"])), ("http.server".to_string(), None));
        assert_eq!(process_label("kworker/0:1", b""), ("kworker/0:1".to_string(), None));
        // sshd rewrites argv[0]; comm is the better name
        assert_eq!(process_label("sshd", &cmd(&["sshd: cat@pts/0"])), ("sshd".to_string(), None));
    }

    #[test]
    fn status_rss() {
        assert_eq!(parse_status_rss_kb("Name:\tx\nVmRSS:\t   51200 kB\nThreads:\t3\n"), 51200);
        assert_eq!(parse_status_rss_kb("Name:\tkthreadd\n"), 0);
    }

    #[test]
    fn diskstats_and_filter() {
        let d = parse_diskstats(
            " 179       0 mmcblk0 33002 8399 1561004 100336 6290 12544 194456 66661 0 18780 172692 1548 138 43822680 5474 429 220\n\
 179       1 mmcblk0p1 10 0 80 1 0 0 0 0 0 1 1\n",
        );
        assert_eq!(d.len(), 2);
        assert_eq!(d[0].0, "mmcblk0");
        assert_eq!(d[0].1, DiskCounters { read_sectors: 1561004, write_sectors: 194456, io_ms: 18780 });
        assert!(is_shown_disk("mmcblk0"));
        assert!(is_shown_disk("nvme0n1"));
        assert!(!is_shown_disk("mmcblk0boot0"));
        assert!(!is_shown_disk("mmcblk0rpmb"));
        assert!(!is_shown_disk("loop3"));
        assert!(!is_shown_disk("zram0"));
    }

    #[test]
    fn mountinfo_longest_prefix() {
        let m = parse_mountinfo(
            "600 500 0:52 / / rw,relatime - overlay overlay rw,lowerdir=/x\n\
601 600 179:3 /home/cat/.mower /home/mower/.mower rw,relatime - ext4 /dev/mmcblk0p3 rw\n\
602 601 259:1 / /home/mower/.mower/bags rw,relatime - ext4 /dev/nvme0n1p1 rw\n\
603 600 0:60 / /mnt/my\\040disk rw - vfat /dev/sda1 rw\n",
        );
        assert_eq!(m.len(), 4);
        assert_eq!(mount_for(&m, "/home/mower/.mower").unwrap().2, "/dev/mmcblk0p3");
        assert_eq!(mount_for(&m, "/home/mower/.mower/bags").unwrap().2, "/dev/nvme0n1p1");
        assert_eq!(mount_for(&m, "/home/mower/.mowerx").unwrap().1, "overlay");
        assert_eq!(mount_for(&m, "/").unwrap().1, "overlay");
        assert_eq!(mount_for(&m, "/mnt/my disk/a").unwrap().2, "/dev/sda1");
    }

    #[test]
    fn df_entry() {
        // 29 GB ext4 with 5 % reserved: used / (used + avail)
        let e = disk_entry("state", "/home/mower/.mower", 4096, 7_500_000, 5_600_000, 5_225_000, "ext4", "/dev/mmcblk0p3");
        assert_eq!(e["dev"], json!("mmcblk0p3"));
        assert_eq!(e["total_gb"], json!(30.72));
        assert_eq!(e["used_gb"], json!(7.78));
        assert_eq!(e["used_pct"], json!(26.7));
    }

    #[test]
    fn cpu_lists() {
        assert_eq!(parse_cpu_list("0 1 2 3"), vec![0, 1, 2, 3]);
        assert_eq!(parse_cpu_list("0-3"), vec![0, 1, 2, 3]);
        assert_eq!(parse_cpu_list("0-1,4\n"), vec![0, 1, 4]);
    }

    /// On the machine running the tests: the second sample has every rate.
    #[test]
    fn live_sample_has_the_whole_shape() {
        let dir = std::env::temp_dir();
        let mut s = Sampler::new(dir.to_str().unwrap());
        let t0 = Instant::now();
        let first = s.poll(t0).unwrap();
        assert!(first["cpu"]["pct"].is_null());
        assert!(s.poll(t0 + Duration::from_millis(500)).is_none());
        // burn a little CPU so the busiest process has ticks
        let spin = Instant::now();
        let mut x = 0u64;
        while spin.elapsed() < Duration::from_millis(300) {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1);
        }
        std::hint::black_box(x);
        let doc = s.poll(t0 + Duration::from_secs_f64(PERIOD_S)).unwrap();
        let pct = doc["cpu"]["pct"].as_f64().unwrap();
        assert!((0.0..=100.0).contains(&pct));
        assert!(!doc["cpu"]["cores"].as_array().unwrap().is_empty());
        assert!(doc["mem"]["total_mb"].as_f64().unwrap() > 0.0);
        assert!(doc["mem_used_pct"].as_f64().is_some());
        assert!(doc["load1"].as_f64().is_some());
        assert!(!doc["disks"].as_array().unwrap().is_empty());
        let procs = doc["procs"].as_array().unwrap();
        assert!(!procs.is_empty() && procs.len() <= TOP_PROCESSES);
        assert!(procs[0]["cpu"].as_f64().is_some());
        assert!(doc["tasks"]["procs"].as_u64().unwrap() >= procs.len() as u64);
    }
}

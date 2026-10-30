//! `FrameReader`, `Bootloader`, `read_app_info`, `do_flash` and `do_sync` of
//! `firmware/tools/mower_flash.py`, statement for statement: the same frames,
//! seqs, timeouts, sleeps, retries, read windows, prints and outcomes.

use std::collections::VecDeque;

use mower_base_core::boot::{
    self, decode_bl_ack, decode_bl_info, erase_payload, verify_payload, write_payload, BL_ACK, BL_ERASE, BL_INFO,
    BL_PING, BL_RUN_APP, BL_VERIFY, BL_WRITE, BOOT_REQUEST_MAGIC, ENTER_BOOTLOADER, ENTER_BOOTLOADER_ACK,
    WRITE_CHUNK_MAX,
};
use mower_base_core::protocol::{build_frame, FrameParser, FIRMWARE_INFO, INFO_REQUEST};

use crate::image::{load_image, load_manifest, open_error};
use crate::link::{Clock, Link};
use crate::pyjson::{self, Py};
use crate::{Failure, Out, Trace};

/// `APP_STATUS_TYPES | {T_FIRMWARE_INFO}`.
const APP_FRAMES: &[u8] = &[0x81, 0x82, 0x83, 0x84, 0x85, 0x86, FIRMWARE_INFO];

pub const NO_BOOTLOADER: &str = "bootloader did not answer. Is the port right, is the bootloader flashed at \
                                 0x08000000, and is nothing else using the port?";

/// One CRC-valid frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub ftype: u8,
    pub seq: u8,
    pub payload: Vec<u8>,
}

/// `FrameReader`: `_pop_frame` is `protocol::FrameParser` (the same frames
/// after every read, `mower_base_core/tests/boot_vectors.rs`); what a `read`
/// did not want stays queued for the next one, as in Python's buffer.
#[derive(Default)]
pub struct FrameReader {
    parser: FrameParser,
    queue: VecDeque<Frame>,
}

impl FrameReader {
    pub fn feed(&mut self, data: &[u8]) -> Vec<Frame> {
        let mut got = Vec::new();
        self.parser.feed(data, |ftype, seq, payload| got.push(Frame { ftype, seq, payload: payload.to_vec() }));
        self.queue.extend(got.iter().cloned());
        got
    }
    pub fn pop(&mut self) -> Option<Frame> {
        self.queue.pop_front()
    }
}

/// `parse_info`: the decoded 0x90.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlInfo(pub boot::BlInfo);

/// `parse_info(payload)`: `struct.unpack("<IIHBBI", ...)`, which raises on
/// any other length.
pub fn parse_info(payload: &[u8]) -> Result<BlInfo, Failure> {
    decode_bl_info(payload)
        .map(BlInfo)
        .ok_or_else(|| Failure::Uncaught("struct.error: unpack requires a buffer of 16 bytes".into()))
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

/// `print_info(info)`.
pub fn describe_bl_info(info: &BlInfo) -> String {
    let i = &info.0;
    format!(
        "bootloader v{}: app @ 0x{:08X}, max {} KB, chunk {} B, app_valid={} by_request={} erased={}",
        i.bootloader_version,
        i.app_start_address,
        i.app_max_size / 1024,
        i.write_chunk_max,
        py_bool(i.app_valid()),
        py_bool(i.entered_by_request()),
        py_bool(i.erased())
    )
}

/// `parse_firmware_info`: the decoded 0x87.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwInfo {
    pub semver: [u8; 3],
    pub protocol_version: u8,
    pub git_sha32: u32,
    pub build_unix: u32,
    pub dirty: bool,
    pub unversioned: bool,
}

pub fn parse_firmware_info(p: &[u8]) -> FwInfo {
    FwInfo {
        semver: [p[0], p[1], p[2]],
        protocol_version: p[3],
        git_sha32: u32::from_le_bytes([p[4], p[5], p[6], p[7]]),
        build_unix: u32::from_le_bytes([p[8], p[9], p[10], p[11]]),
        dirty: p[12] & 0x01 != 0,
        unversioned: p[12] & 0x02 != 0,
    }
}

impl FwInfo {
    fn version(&self) -> String {
        format!("{}.{}.{}", self.semver[0], self.semver[1], self.semver[2])
    }

    /// The dict `parse_firmware_info` returns, in its key order.
    pub fn to_py(&self) -> Py {
        Py::Dict(vec![
            ("semver".into(), Py::List(self.semver.iter().map(|&v| Py::int(v as i64)).collect())),
            ("version".into(), Py::Str(self.version())),
            ("protocol_version".into(), Py::int(self.protocol_version as i64)),
            ("git_sha32".into(), Py::int(self.git_sha32 as i64)),
            ("git_sha".into(), Py::Str(format!("{:08x}", self.git_sha32))),
            ("build_unix".into(), Py::int(self.build_unix as i64)),
            ("dirty".into(), Py::Bool(self.dirty)),
            ("unversioned".into(), Py::Bool(self.unversioned)),
        ])
    }

    /// `describe_firmware_info(info)`; `built=` is local time, like
    /// `time.localtime` (TZ applies).
    pub fn describe(&self) -> String {
        let when = if self.build_unix != 0 { local_time(self.build_unix as i64) } else { "unknown".into() };
        let mut tags = String::new();
        if self.dirty {
            tags.push_str(" dirty");
        }
        if self.unversioned {
            tags.push_str(" unversioned");
        }
        format!("v{} sha={:08x} built={when}{tags}", self.version(), self.git_sha32)
    }
}

extern "C" {
    fn tzset();
}

/// `time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t))`.
pub fn local_time(t: i64) -> String {
    let tt = t as libc::time_t;
    let mut tm: libc::tm = unsafe { std::mem::zeroed() };
    unsafe {
        tzset();
        libc::localtime_r(&tt, &mut tm);
    }
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        tm.tm_year + 1900,
        tm.tm_mon + 1,
        tm.tm_mday,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec
    )
}

/// `firmware_matches(running, bundled)`, with Python's `==`, `list()` and
/// `bool()`, its short-circuits, and the KeyError / TypeError it raises on a
/// manifest that lacks what it reads.
pub fn firmware_matches(running: &FwInfo, bundled: &Py) -> Result<bool, Failure> {
    if running.unversioned {
        return Ok(false);
    }
    let item = |key: &str| pyjson::getitem(bundled, key).map_err(Failure::Uncaught);
    let semver = pyjson::to_list(item("semver")?).map_err(Failure::Uncaught)?;
    let semver_ok =
        semver.len() == 3 && semver.iter().zip(running.semver).all(|(v, r)| pyjson::eq_int(r as i64, v));
    if !semver_ok {
        return Ok(false);
    }
    if !pyjson::eq_int(running.git_sha32 as i64, item("git_sha32")?) {
        return Ok(false);
    }
    if !pyjson::eq_int(running.build_unix as i64, item("build_unix")?) {
        return Ok(false);
    }
    let dirty = bundled.get("dirty").map(pyjson::truthy).unwrap_or(false);
    Ok(running.dirty == dirty)
}

/// `check(status, what, value)`.
pub fn check(status: u8, what: &str, value: u32) -> Result<(), Failure> {
    if status != 0 {
        let name = boot::status_name(status).map(str::to_string).unwrap_or_else(|| status.to_string());
        return Err(Failure::Runtime(format!("{what} failed: {name} (value=0x{value:08X})")));
    }
    Ok(())
}

/// The serial session: `Bootloader` (its seq and its `FrameReader`) plus
/// what the module-level functions print.
pub struct Session<L: Link, C: Clock> {
    pub link: L,
    pub clock: C,
    pub reader: FrameReader,
    pub seq: u8,
    pub out: Out,
    pub trace: Option<Trace>,
}

impl<L: Link, C: Clock> Session<L, C> {
    pub fn new(link: L, clock: C, out: Out, trace: Option<Trace>) -> Self {
        Session { link, clock, reader: FrameReader::default(), seq: 0, out, trace }
    }

    fn next_seq(&mut self) -> u8 {
        self.seq = self.seq.wrapping_add(1);
        self.seq
    }

    /// `Bootloader.send`.
    pub fn send(&mut self, ftype: u8, payload: &[u8]) -> Result<u8, Failure> {
        let seq = self.next_seq();
        if let Some(t) = self.trace.as_mut() {
            t.frame(self.clock.monotonic(), "tx", ftype, seq, payload);
        }
        self.link.write(&build_frame(ftype, seq, payload))?;
        Ok(seq)
    }

    /// `FrameReader.read(want_type, timeout)`.
    pub fn read(&mut self, want: &[u8], timeout: f64) -> Result<Option<Frame>, Failure> {
        let deadline = self.clock.monotonic() + timeout;
        loop {
            while let Some(frame) = self.reader.pop() {
                if want.contains(&frame.ftype) {
                    return Ok(Some(frame));
                }
            }
            let remaining = deadline - self.clock.monotonic();
            if remaining <= 0.0 {
                return Ok(None);
            }
            let chunk = self.link.read(256, remaining.min(0.05))?;
            if !chunk.is_empty() {
                let frames = self.reader.feed(&chunk);
                if let Some(t) = self.trace.as_mut() {
                    let now = self.clock.monotonic();
                    for f in &frames {
                        t.frame(now, "rx", f.ftype, f.seq, &f.payload);
                    }
                }
            }
        }
    }

    /// `Bootloader.ping(timeout)`.
    pub fn ping(&mut self, timeout: f64) -> Result<Option<BlInfo>, Failure> {
        let seq = self.send(BL_PING, &[])?;
        let deadline = self.clock.monotonic() + timeout;
        loop {
            let remaining = deadline - self.clock.monotonic();
            if remaining <= 0.0 {
                return Ok(None);
            }
            let Some(frame) = self.read(&[BL_INFO], remaining)? else {
                return Ok(None);
            };
            if frame.seq == seq {
                return parse_info(&frame.payload).map(Some);
            }
        }
    }

    /// `Bootloader.command(ftype, payload, timeout)` -> (status, value).
    pub fn command(&mut self, ftype: u8, payload: &[u8], timeout: f64) -> Result<(u8, u32), Failure> {
        let seq = self.send(ftype, payload)?;
        let deadline = self.clock.monotonic() + timeout;
        let no_ack = || Failure::Timeout(format!("no ack for command 0x{ftype:02X}"));
        loop {
            let remaining = deadline - self.clock.monotonic();
            if remaining <= 0.0 {
                return Err(no_ack());
            }
            let Some(frame) = self.read(&[BL_ACK], remaining)? else {
                return Err(no_ack());
            };
            // struct.unpack("<BBI", frame[2]) comes before the cmd / seq test
            let ack = decode_bl_ack(&frame.payload)
                .ok_or_else(|| Failure::Uncaught("struct.error: unpack requires a buffer of 6 bytes".into()))?;
            if ack.command == ftype && frame.seq == seq {
                return Ok((ack.status, ack.value));
            }
        }
    }

    /// `Bootloader.request_enter_from_app()`.
    pub fn request_enter_from_app(&mut self) -> Result<bool, Failure> {
        self.send(ENTER_BOOTLOADER, &boot::enter_payload(BOOT_REQUEST_MAGIC))?;
        Ok(self.read(&[ENTER_BOOTLOADER_ACK], 0.5)?.is_some())
    }

    /// `Bootloader.ensure_bootloader(attempts=25)`.
    pub fn ensure_bootloader(&mut self, attempts: u32) -> Result<BlInfo, Failure> {
        if let Some(info) = self.ping(0.3)? {
            return Ok(info);
        }
        self.out.println("application is running, asking it to reboot into the bootloader");
        let acked = self.request_enter_from_app()?;
        self.out.println(if acked { "  app acknowledged" } else { "  no ack from app (continuing anyway)" });
        self.clock.sleep(0.2);
        for _ in 0..attempts {
            if let Some(info) = self.ping(0.2)? {
                return Ok(info);
            }
        }
        Err(Failure::Runtime(NO_BOOTLOADER.into()))
    }

    /// `read_app_info(bl, timeout)` -> (info, saw_app).
    pub fn read_app_info(&mut self, timeout: f64) -> Result<(Option<FwInfo>, bool), Failure> {
        self.send(INFO_REQUEST, &[])?;
        let deadline = self.clock.monotonic() + timeout;
        let mut saw_app = false;
        loop {
            let remaining = deadline - self.clock.monotonic();
            if remaining <= 0.0 {
                return Ok((None, saw_app));
            }
            let Some(frame) = self.read(APP_FRAMES, remaining)? else {
                return Ok((None, saw_app));
            };
            if frame.ftype == FIRMWARE_INFO && frame.payload.len() == 16 {
                return Ok((Some(parse_firmware_info(&frame.payload)), true));
            }
            saw_app = true;
        }
    }

    /// `do_flash(bl, path, run_after)`.
    pub fn do_flash(&mut self, path: &str, run_after: bool) -> Result<(), Failure> {
        let image = load_image(path)?;
        let crc = boot::crc32_ieee(&image);
        let info = self.ensure_bootloader(25)?;
        self.out.println(&describe_bl_info(&info));
        let chunk_size = (WRITE_CHUNK_MAX.min(info.0.write_chunk_max as usize)) & !3;

        self.out.println(&format!("erasing for {} bytes ...", image.len()));
        let (status, value) = self.command(BL_ERASE, &erase_payload(image.len() as u32), 20.0)?;
        check(status, "erase", value)?;

        // the vector table (offset 0) last: an interrupted update leaves the
        // reset vector blank and the bootloader refuses to jump into it
        if chunk_size == 0 {
            return Err(Failure::Uncaught("ValueError: range() arg 3 must not be zero".into()));
        }
        let mut offsets: Vec<usize> = (chunk_size..image.len()).step_by(chunk_size).collect();
        offsets.push(0);
        let t0 = self.clock.monotonic();
        let total = offsets.len();
        for (i, &off) in offsets.iter().enumerate() {
            let n = i + 1;
            let data = &image[off..(off + chunk_size).min(image.len())];
            let payload = write_payload(off as u32, data);
            for attempt in 0..3 {
                match self.command(BL_WRITE, &payload, 2.0) {
                    Ok((status, value)) => {
                        check(status, &format!("write @0x{off:X}"), value)?;
                        break;
                    }
                    Err(Failure::Timeout(msg)) => {
                        if attempt == 2 {
                            return Err(Failure::Timeout(msg));
                        }
                        self.out.println(&format!("  retry write @0x{off:X}"));
                    }
                    Err(other) => return Err(other),
                }
            }
            if n % 32 == 0 || n == total {
                let done = n * chunk_size;
                let pct = (100 * done / image.len()).min(100);
                self.out.write(&format!("\r  {pct:3}%  {}/{} bytes", done.min(image.len()), image.len()));
                self.out.flush();
            }
        }
        self.out.println(&format!("\n  written in {:.1} s", self.clock.monotonic() - t0));

        self.out.println("verifying ...");
        let (status, value) = self.command(BL_VERIFY, &verify_payload(image.len() as u32, crc), 5.0)?;
        check(status, "verify", value)?;
        self.out.println(&format!("  crc32 0x{crc:08X} ok"));

        if run_after {
            self.out.println("starting application");
            let (status, value) = self.command(BL_RUN_APP, &[], 2.0)?;
            check(status, "run", value)?;
        } else {
            self.out.println("staying in bootloader (use `run` to start the app)");
        }
        Ok(())
    }

    /// `do_sync(bl, image_path, manifest_path, report_path, force)` -> exit
    /// code: 0 up to date / flashed, 1 failed, 2 no STM32.
    pub fn do_sync(
        &mut self,
        image_path: &str,
        manifest_path: Option<&str>,
        report_path: Option<&str>,
        force: bool,
    ) -> Result<i32, Failure> {
        let bundled = load_manifest(image_path, manifest_path)?;
        if !matches!(bundled, Py::Dict(_)) {
            return Err(Failure::Uncaught(format!(
                "AttributeError: '{}' object has no attribute 'get'",
                bundled.type_name()
            )));
        }
        let mut report = Report {
            time: self.clock.time().floor() as i64,
            bundled: Py::Dict(
                ["version", "semver", "git_sha", "git_sha32", "build_unix", "dirty", "crc32", "size"]
                    .iter()
                    .map(|k| (k.to_string(), bundled.get(k).cloned().unwrap_or(Py::None)))
                    .collect(),
            ),
            running_before: None,
            running_after: None,
        };

        let item = |key: &str| pyjson::getitem(&bundled, key).cloned().map_err(Failure::Uncaught);
        let version = pyjson::py_str(&item("version")?);
        let sha = pyjson::py_str(&pyjson::head8(&item("git_sha")?).map_err(Failure::Uncaught)?);
        let size = pyjson::py_str(&item("size")?);
        self.out.println(&format!("bundled: {version} sha={sha} {size} bytes"));

        let (running, saw_app) = self.read_app_info(1.5)?;
        report.running_before = running;
        let why;
        if let Some(r) = running {
            self.out.println(&format!("running: {}", r.describe()));
            if firmware_matches(&r, &bundled)? && !force {
                self.out.println("firmware is up to date");
                report.running_after = Some(r);
                return report.finish(report_path, "up_to_date", 0, None);
            }
            why = if force { "forced" } else { "different build" };
        } else if saw_app {
            why = "running firmware predates 0x87 (no build identity)";
            self.out.println(&format!("running: unknown ({why})"));
        } else {
            if self.ping(0.5)?.is_none() {
                self.out.println("no application status and no bootloader answer on this port");
                return report.finish(report_path, "no_device", 2, Some("no STM32 detected"));
            }
            why = "board is in the bootloader (no valid app?)";
            self.out.println(&format!("running: none ({why})"));
        }

        self.out.println(&format!("flashing: {why}"));
        match self.do_flash(image_path, true) {
            Ok(()) => {}
            Err(Failure::Runtime(msg)) | Err(Failure::Timeout(msg)) => {
                self.out.println(&format!("flash failed: {msg}"));
                return report.finish(report_path, "failed", 1, Some(&msg));
            }
            Err(other) => return Err(other),
        }

        // the app restarts and sends 0x87 within its first second
        self.clock.sleep(0.3);
        let (after, _) = self.read_app_info(3.0)?;
        report.running_after = after;
        let Some(after) = after else {
            return report.finish(report_path, "failed", 1, Some("application did not report 0x87 after flashing"));
        };
        self.out.println(&format!("now running: {}", after.describe()));
        if !firmware_matches(&after, &bundled)? {
            return report.finish(report_path, "failed", 1, Some("running build still differs from the bundled one"));
        }
        report.finish(report_path, "flashed", 0, None)
    }
}

/// `do_sync`'s `report` dict.
pub struct Report {
    pub time: i64,
    pub bundled: Py,
    pub running_before: Option<FwInfo>,
    pub running_after: Option<FwInfo>,
}

impl Report {
    pub fn to_py(&self, action: &str, error: Option<&str>) -> Py {
        let info = |i: &Option<FwInfo>| i.map(|i| i.to_py()).unwrap_or(Py::None);
        Py::Dict(vec![
            ("time".into(), Py::int(self.time)),
            ("bundled".into(), self.bundled.clone()),
            ("running_before".into(), info(&self.running_before)),
            ("running_after".into(), info(&self.running_after)),
            ("action".into(), Py::str(action)),
            ("error".into(), error.map(Py::str).unwrap_or(Py::None)),
        ])
    }

    /// `finish(action, code, error)`: `json.dump(report, f, indent=2)` to
    /// `REPORT.tmp`, then `os.replace` onto REPORT.
    pub fn finish(&self, report_path: Option<&str>, action: &str, code: i32, error: Option<&str>) -> Result<i32, Failure> {
        // `if report_path:`, so `--report ""` writes nothing
        if let Some(path) = report_path.filter(|p| !p.is_empty()) {
            let tmp = format!("{path}.tmp");
            let text = pyjson::dumps(&self.to_py(action, error), Some(2));
            std::fs::write(&tmp, text).map_err(|e| open_error(&e, &tmp))?;
            std::fs::rename(&tmp, path).map_err(|e| {
                let errno = e.raw_os_error().unwrap_or(0);
                Failure::Uncaught(format!(
                    "OSError: {}: {} -> {}",
                    crate::link::os_error(errno),
                    pyjson::str_repr(&tmp),
                    pyjson::str_repr(path)
                ))
            })?;
        }
        Ok(code)
    }
}

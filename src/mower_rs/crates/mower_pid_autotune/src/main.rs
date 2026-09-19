//! mower_pid_autotune: wheel-speed PID auto-tune for the STM32 base (port of
//! `mower_mission/pid_autotune_node.py`, node name `pid_autotune`, same
//! parameters, service, topics and status JSON).
//!
//! Drives both wheels open loop through two PWM steps (0 -> low -> high),
//! reads the speed response the STM32 reports in 0x85 (via the mower_hardware
//! `/mower_base/telemetry` JSON), fits a first-order-plus-dead-time model per
//! wheel, turns it into SIMC PI gains ([`tuning`]), verifies them with a
//! closed-loop step and then waits for the operator to write them to flash or
//! throw them away. Nothing is persisted without an explicit "apply".
//!
//! The wheels spin at up to `step_high_permille` duty while this runs: the
//! vehicle has to be jacked up with both wheels free. The dashboard asks the
//! operator to confirm that before it calls "start"; this node cannot check it.
//!
//! Interface
//! ---------
//! service `/pid_autotune` (mower_interface/srv/PidAutotune):
//!     op = "start" | "abort" | "apply" | "discard"
//! topic  `/pid_autotune/status` (std_msgs/String JSON, latched, 5 Hz while active):
//!     {"state": "idle|precheck|open_loop|fitting|verify|review|saving|done|failed|aborted",
//!      "progress": 0.0-1.0, "message": "...", "error": null|"...",
//!      "started_at": unix_s, "updated_at": unix_s,
//!      "old_gains": {"left": {"kp","ki","kd"}, "right": {...}} | null,
//!      "new_gains": {...} | null,
//!      "model": {"left": {"gain","tau","delay",...}, "right": {...}} | null,
//!      "verify": {"left": {"target","overshoot_pct","settle_s","ss_error","rise_s"}, "right": {...}} | null,
//!      "samples": {"open_loop": {"left": [[t, pwm, rpm], ...], "right": [...]},
//!                  "verify":    {"left": [...], "right": [...]}},
//!      "marks": {"open_loop_low": t, "open_loop_high": t, "verify": t}}
//!
//! `t` in samples/marks is seconds since `started_at`.
//!
//! Uses the mower_hardware side channels `/mower_base/pid_command` (0x04) and
//! `/mower_base/wheel_override` (raw permille, bypasses diff_drive_controller's
//! acceleration limits so a step is a step).
//!
//! Intentional difference from the Python node: the telemetry subscription
//! is permanent (rclpy paid ~6 ms per 17 Hz frame, r2r pays microseconds);
//! staleness is judged by the frame's arrival time exactly as before.

mod tuning;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use futures::StreamExt;
use mower_rs_common::{params, round_to, unix_time_s};
use r2r::mower_interface::srv::{MissionOperationLock, PidAutotune};
use r2r::std_msgs::msg::{Bool, String as StringMsg};
use r2r::QosProfile;
use serde::Serialize;
use serde_json::Value;

use crate::tuning::{FopdtModel, Gains, Sample, SimcOptions, StepMetrics, TuningError};

// telemetry pid.flags (UART 0x84)
const PID_FLAG_CLOSED_LOOP: i64 = 0x01;
const PID_FLAG_FLASH_VALID: i64 = 0x02;
const PID_FLAG_LAST_APPLY_OK: i64 = 0x08;
// telemetry motor.flags (0x81)
const MOTOR_FLAG_DRIVER_ALARM: i64 = 0x04;
// telemetry power.state (0x86)
const POWER_STATE_RUNNING: i64 = 0;

const WHEELS: [&str; 2] = ["left", "right"];

// ---------------------------------------------------------------- status JSON

#[derive(Clone, Serialize, Default, PartialEq, Debug)]
struct Wheels<T> {
    left: T,
    right: T,
}

impl<T> Wheels<T> {
    fn get(&self, w: &str) -> &T {
        if w == "left" { &self.left } else { &self.right }
    }
    fn get_mut(&mut self, w: &str) -> &mut T {
        if w == "left" { &mut self.left } else { &mut self.right }
    }
}

type WheelGains = Wheels<Gains>;

#[derive(Clone, Serialize, Default)]
struct Samples {
    open_loop: Wheels<Vec<[f64; 3]>>,
    verify: Wheels<Vec<[f64; 3]>>,
}

#[derive(Clone, Serialize, Default)]
struct Marks {
    #[serde(skip_serializing_if = "Option::is_none")]
    open_loop_low: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    open_loop_high: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    verify: Option<f64>,
}

#[derive(Clone, Serialize)]
struct Status {
    state: String,
    progress: f64,
    message: String,
    error: Option<String>,
    started_at: Option<f64>,
    updated_at: f64,
    old_gains: Option<WheelGains>,
    new_gains: Option<WheelGains>,
    model: Option<Wheels<FopdtModel>>,
    verify: Option<Wheels<StepMetrics>>,
    samples: Samples,
    marks: Marks,
}

impl Status {
    fn blank(state: &str, message: &str) -> Status {
        Status {
            state: state.to_string(),
            progress: 0.0,
            message: message.to_string(),
            error: None,
            started_at: None,
            updated_at: unix_time_s(),
            old_gains: None,
            new_gains: None,
            model: None,
            verify: None,
            samples: Samples::default(),
            marks: Marks::default(),
        }
    }
}

// ------------------------------------------------------------ base messages

#[derive(Serialize)]
struct PidRequest<'a> {
    left: &'a Gains,
    right: &'a Gains,
    persist: i64,
    closed_loop: i64,
}

#[derive(Serialize)]
struct OverrideRequest {
    left_permille: i64,
    right_permille: i64,
    ttl_ms: i64,
}

#[derive(Serialize)]
struct LedRequest {
    mode: i64,
    r: i64,
    g: i64,
    b: i64,
    period_ms: i64,
}

// ------------------------------------------------------------------- errors

enum RunError {
    Aborted,
    Precondition(String),
    Tuning(String),
}

impl From<TuningError> for RunError {
    fn from(e: TuningError) -> Self {
        RunError::Tuning(e.0)
    }
}

fn precondition<T>(msg: impl Into<String>) -> Result<T, RunError> {
    Err(RunError::Precondition(msg.into()))
}

// -------------------------------------------------------------- json helpers

fn obj<'a>(v: &'a Value, key: &str) -> Option<&'a serde_json::Map<String, Value>> {
    // `d.get(key) or {}` -- anything that is not an object counts as empty
    v.get(key).and_then(Value::as_object)
}

fn truthy(v: Option<&Value>) -> bool {
    match v {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(Value::Number(n)) => n.as_f64().map(|f| f != 0.0).unwrap_or(false),
        Some(Value::String(s)) => !s.is_empty(),
        Some(Value::Array(a)) => !a.is_empty(),
        Some(Value::Object(o)) => !o.is_empty(),
    }
}

/// `int(obj.get(key, 0))` for the flag / state fields.
fn int_of(m: &serde_json::Map<String, Value>, key: &str) -> i64 {
    match m.get(key) {
        Some(Value::Number(n)) => n.as_i64().or_else(|| n.as_f64().map(|f| f as i64)).unwrap_or(0),
        Some(Value::Bool(b)) => *b as i64,
        Some(Value::String(s)) => s.trim().parse().unwrap_or(0),
        _ => 0,
    }
}

/// `float(x)` of a JSON value, None where Python would raise.
fn float_of(v: Option<&Value>) -> Option<f64> {
    match v {
        Some(Value::Number(n)) => n.as_f64(),
        Some(Value::Bool(b)) => Some(*b as i64 as f64),
        Some(Value::String(s)) => s.trim().parse().ok(),
        _ => None,
    }
}

fn gains_of(pid: &serde_json::Map<String, Value>) -> Result<WheelGains, RunError> {
    let one = |w: &str| -> Result<Gains, RunError> {
        let side = pid.get(w).and_then(Value::as_object);
        let get = |k: &str| side.and_then(|s| float_of(s.get(k)));
        match (get("kp"), get("ki"), get("kd")) {
            (Some(kp), Some(ki), Some(kd)) => Ok(Gains { kp, ki, kd }),
            _ => precondition(format!("base PID settings for the {w} wheel are malformed")),
        }
    };
    Ok(Wheels { left: one("left")?, right: one("right")? })
}

fn gains_close(a: &WheelGains, b: &WheelGains) -> bool {
    let tol = 1e-3;
    WHEELS.iter().all(|w| {
        let (x, y) = (a.get(w), b.get(w));
        (x.kp - y.kp).abs() <= tol && (x.ki - y.ki).abs() <= tol && (x.kd - y.kd).abs() <= tol
    })
}

// ------------------------------------------------------------------ settings

struct Settings {
    step_low_permille: i64,
    step_high_permille: i64,
    hold_s: f64,
    verify_permille: i64,
    verify_hold_s: f64,
    tau_c_factor: f64,
    max_overshoot_pct: f64,
    max_settle_s: f64,
    max_ss_error_rpm: f64,
    telemetry_timeout_s: f64,
    review_timeout_s: f64,
    led_busy_rgb: Vec<i64>,
    led_busy_period_ms: i64,
    led_normal_rgb: Vec<i64>,
}

// --------------------------------------------------------------------- node

struct Telemetry {
    latest: Option<(Value, Instant)>,
    seq: Option<Value>,
    recording: Option<Wheels<Vec<Sample>>>,
}

struct Ctx {
    logger: String,
    settings: Settings,
    status: Mutex<Status>,
    tel: Mutex<Telemetry>,
    nav_active: AtomicBool,
    active: AtomicBool,
    abort: AtomicBool,
    alarm_logged: AtomicBool,
    lock_held: AtomicBool,
    decision: Mutex<Option<String>>,
    started_mono: Mutex<Instant>,
    lock_owner: String,
    status_pub: Mutex<r2r::Publisher<StringMsg>>,
    pid_pub: Mutex<r2r::Publisher<StringMsg>>,
    override_pub: Mutex<r2r::Publisher<StringMsg>>,
    led_pub: Option<Mutex<r2r::Publisher<StringMsg>>>,
    lock_client: r2r::Client<MissionOperationLock::Service>,
}

impl Ctx {
    // ------------------------------------------------------------- status
    fn publish_status(&self) {
        let data = serde_json::to_string(&*self.status.lock().unwrap()).unwrap_or_default();
        let _ = self.status_pub.lock().unwrap().publish(&StringMsg { data });
    }

    fn set(&self, state: Option<&str>, progress: Option<f64>, message: Option<&str>) {
        self.set_with(state, progress, message, |_| {});
    }

    /// `_set(state, progress, message, **fields)`: the closure applies the
    /// extra fields under the same lock.
    fn set_with(&self, state: Option<&str>, progress: Option<f64>, message: Option<&str>, fields: impl FnOnce(&mut Status)) {
        let line = {
            let mut st = self.status.lock().unwrap();
            if let Some(s) = state {
                st.state = s.to_string();
            }
            if let Some(p) = progress {
                st.progress = p;
            }
            if let Some(m) = message {
                st.message = m.to_string();
            }
            fields(&mut st);
            st.updated_at = unix_time_s();
            if state.is_some() || message.is_some() { Some(format!("{}: {}", st.state, st.message)) } else { None }
        };
        if let Some(line) = line {
            r2r::log_info!(&self.logger, "{}", line);
        }
        self.publish_status();
    }

    fn rel(&self, mono: Instant) -> f64 {
        let started = *self.started_mono.lock().unwrap();
        let dt = if mono >= started { (mono - started).as_secs_f64() } else { -((started - mono).as_secs_f64()) };
        round_to(dt, 3)
    }

    // ------------------------------------------------------------ callbacks
    fn on_telemetry(&self, text: &str) {
        let Ok(d) = serde_json::from_str::<Value>(text) else { return };
        let now = Instant::now();
        let mut tel = self.tel.lock().unwrap();
        let wheel = obj(&d, "wheel").cloned();
        tel.latest = Some((d, now));
        if tel.recording.is_none() {
            return;
        }
        let seq = wheel.as_ref().and_then(|w| w.get("seq")).cloned();
        if seq == tel.seq {
            return; // driver republished the same 0x85
        }
        tel.seq = seq;
        let rec = tel.recording.as_mut().unwrap();
        for w in WHEELS {
            let side = wheel.as_ref().and_then(|m| m.get(w)).and_then(Value::as_object);
            let pwm = side.and_then(|s| float_of(s.get("pid_output")));
            let rpm = side.and_then(|s| float_of(s.get("measured_rpm")));
            if let (Some(pwm), Some(rpm)) = (pwm, rpm) {
                rec.get_mut(w).push(Sample { t: now_secs(now), pwm, rpm });
            }
        }
    }

    // --------------------------------------------------------- base helpers
    /// Latest base telemetry, or an error when the base has gone quiet.
    async fn telemetry(&self, timeout_s: Option<f64>) -> Result<Value, RunError> {
        if self.abort.load(Ordering::Relaxed) {
            return Err(RunError::Aborted);
        }
        let timeout_s = timeout_s.unwrap_or(self.settings.telemetry_timeout_s);
        // The first frame of a session may still be in flight: give the base
        // one timeout window.
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s.max(0.0));
        while self.tel.lock().unwrap().latest.is_none() && Instant::now() < deadline {
            if self.abort.load(Ordering::Relaxed) {
                return Err(RunError::Aborted);
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        let latest = self.tel.lock().unwrap().latest.clone();
        let tel = match latest {
            Some((tel, at)) if at.elapsed().as_secs_f64() <= timeout_s => tel,
            _ => return precondition("no /mower_base/telemetry (is the base driver running?)"),
        };
        // motor.flags DRIVER_ALARM is the BTS7960 IS pin read as a GPIO: an
        // analog current-sense output that goes high with normal drive current
        // (firmware motor.hpp MOTOR_ALARM_DISABLES_OUTPUT 0). Advisory only, so
        // it is logged, never a reason to stop; the BTS7960 protects itself.
        if let Some(motor) = obj(&tel, "motor") {
            if truthy(motor.get("valid")) && int_of(motor, "flags") & MOTOR_FLAG_DRIVER_ALARM != 0 && !self.alarm_logged.swap(true, Ordering::Relaxed) {
                r2r::log_info!(&self.logger, "driver alarm flag set (BTS7960 IS current sense); ignored");
            }
        }
        if let Some(power) = obj(&tel, "power") {
            if truthy(power.get("valid")) && int_of(power, "state") != POWER_STATE_RUNNING {
                return precondition(format!("power state {} is not RUNNING", power.get("state").map(json_repr).unwrap_or_else(|| "None".into())));
            }
        }
        Ok(tel)
    }

    async fn pid_status(&self, timeout_s: Option<f64>) -> Result<serde_json::Map<String, Value>, RunError> {
        let tel = self.telemetry(timeout_s).await?;
        match obj(&tel, "pid") {
            Some(pid) if truthy(pid.get("valid")) => Ok(pid.clone()),
            _ => precondition("base has not reported its PID settings yet"),
        }
    }

    /// Publish a 0x04 request; returns the 0x84 last_rx_seq seen *before* it
    /// went out, so wait_pid can tell a fresh acknowledgement from the old state.
    fn send_pid(&self, gains: &WheelGains, closed_loop: bool, persist: bool) -> Option<Value> {
        let seq0 = self.tel.lock().unwrap().latest.as_ref().and_then(|(t, _)| obj(t, "pid").and_then(|p| p.get("last_rx_seq")).cloned());
        let req = PidRequest { left: &gains.left, right: &gains.right, persist: persist as i64, closed_loop: closed_loop as i64 };
        let _ = self.pid_pub.lock().unwrap().publish(&StringMsg { data: serde_json::to_string(&req).unwrap_or_default() });
        seq0
    }

    /// Wait until the STM32 reports the requested settings (0x84 in telemetry).
    async fn wait_pid(&self, gains: &WheelGains, seq0: Option<Value>, closed_loop: bool, timeout_s: f64, persisted: bool) -> Result<serde_json::Map<String, Value>, RunError> {
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let tel_timeout = if persisted { Some(timeout_s) } else { None };
        while Instant::now() < deadline {
            // a flash sector erase stalls the STM32 (and its status frames) for
            // up to a couple of seconds; do not call that a dead base
            let pid = self.pid_status(tel_timeout).await?;
            let flags = int_of(&pid, "flags");
            let seen = seq0.is_none() || pid.get("last_rx_seq") != seq0.as_ref();
            if seen
                && gains_close(&gains_of(&pid)?, gains)
                && ((flags & PID_FLAG_CLOSED_LOOP != 0) == closed_loop)
                && flags & PID_FLAG_LAST_APPLY_OK != 0
                && (!persisted || flags & PID_FLAG_FLASH_VALID != 0)
            {
                return Ok(pid);
            }
            self.sleep(0.05).await?;
        }
        // last_rx_seq is a uint8 of the driver's frame counter, so once in 256
        // runs the acknowledgement is indistinguishable from the previous one;
        // matching gains and mode are then the best evidence there is.
        let pid = self.pid_status(tel_timeout).await?;
        let flags = int_of(&pid, "flags");
        if gains_close(&gains_of(&pid)?, gains) && ((flags & PID_FLAG_CLOSED_LOOP != 0) == closed_loop) && flags & PID_FLAG_LAST_APPLY_OK != 0 {
            r2r::log_warn!(&self.logger, "PID settings match but no fresh 0x84 seq was seen; accepting");
            return Ok(pid);
        }
        precondition("base did not acknowledge the PID settings")
    }

    /// 0x04 with persist=1, confirmed by FLASH_VALID + LAST_APPLY_OK. The
    /// sector erase stalls the STM32 for ~1 s. One retry: firmware before
    /// cef0d1e fails the first save after a bootloader jump because of a
    /// stale FLASH_SR error bit, and the failed attempt clears it.
    async fn persist(&self, gains: &WheelGains) -> Result<(), RunError> {
        let mut last = String::new();
        for attempt in 1..=2 {
            let seq0 = self.send_pid(gains, true, true);
            match self.wait_pid(gains, seq0, true, 4.0, true).await {
                Ok(_) => return Ok(()),
                Err(RunError::Precondition(e)) => {
                    let diag = self
                        .tel
                        .lock()
                        .unwrap()
                        .latest
                        .as_ref()
                        .and_then(|(t, _)| obj(t, "pid").map(|p| int_of(p, "flash_diag")))
                        .unwrap_or(0);
                    last = format!("{e} (flash save attempt {attempt}, flash_diag=0x{diag:04x})");
                    r2r::log_warn!(&self.logger, "{}", last);
                }
                Err(other) => return Err(other),
            }
        }
        Err(RunError::Precondition(last))
    }

    fn override_wheels(&self, left: i64, right: i64, ttl_ms: i64) {
        let req = OverrideRequest { left_permille: left, right_permille: right, ttl_ms };
        let _ = self.override_pub.lock().unwrap().publish(&StringMsg { data: serde_json::to_string(&req).unwrap_or_default() });
    }

    /// Keep both wheels at `permille` for `seconds`, watching the base.
    async fn hold(&self, permille: i64, seconds: f64) -> Result<(), RunError> {
        let end = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
        loop {
            self.telemetry(None).await?;
            self.override_wheels(permille, permille, 300);
            if Instant::now() >= end {
                return Ok(());
            }
            self.sleep(0.1).await?;
        }
    }

    /// `self._abort.wait(seconds)` -> Aborted when the flag is raised meanwhile.
    async fn sleep(&self, seconds: f64) -> Result<(), RunError> {
        let end = Instant::now() + Duration::from_secs_f64(seconds.max(0.0));
        loop {
            if self.abort.load(Ordering::Relaxed) {
                return Err(RunError::Aborted);
            }
            let now = Instant::now();
            if now >= end {
                return Ok(());
            }
            tokio::time::sleep((end - now).min(Duration::from_millis(20))).await;
        }
    }

    fn record(&self, on: bool) -> Option<Wheels<Vec<Sample>>> {
        let mut tel = self.tel.lock().unwrap();
        if on {
            tel.seq = None;
            tel.recording = Some(Wheels::default());
            return None;
        }
        tel.recording.take()
    }

    fn samples_json(&self, rec: &Wheels<Vec<Sample>>) -> Wheels<Vec<[f64; 3]>> {
        let conv = |v: &Vec<Sample>| v.iter().map(|s| [self.rel(instant_of(s.t)), round_to(s.pwm, 1), round_to(s.rpm, 2)]).collect();
        Wheels { left: conv(&rec.left), right: conv(&rec.right) }
    }

    fn lights(&self, busy: bool) {
        let Some(led) = &self.led_pub else { return };
        let rgb = |v: &Vec<i64>| (v.first().copied().unwrap_or(0), v.get(1).copied().unwrap_or(0), v.get(2).copied().unwrap_or(0));
        let req = if busy {
            let (r, g, b) = rgb(&self.settings.led_busy_rgb);
            LedRequest { mode: 6, r, g, b, period_ms: self.settings.led_busy_period_ms }
        } else {
            let (r, g, b) = rgb(&self.settings.led_normal_rgb);
            LedRequest { mode: 1, r, g, b, period_ms: 0 }
        };
        let _ = led.lock().unwrap().publish(&StringMsg { data: serde_json::to_string(&req).unwrap_or_default() });
    }

    /// Mission mutation lease from nav_action_server; refused while navigation or
    /// manual motion is active. Missing service = development bench, carry on.
    async fn lock(&self, acquire: bool) -> Result<(), RunError> {
        let available = match r2r::Node::is_available(&self.lock_client) {
            Ok(f) => tokio::time::timeout(Duration::from_secs(1), f).await.is_ok(),
            Err(_) => false,
        };
        if !available {
            if acquire {
                r2r::log_warn!(&self.logger, "mission operation lock service unavailable, continuing without it");
            }
            return Ok(());
        }
        let req = MissionOperationLock::Request { owner: self.lock_owner.clone(), operation: "pid auto-tune".to_string(), acquire };
        let fut = match self.lock_client.request(&req) {
            Ok(f) => f,
            Err(_) => return precondition("mission operation lock did not answer"),
        };
        let res = match tokio::time::timeout(Duration::from_secs(3), fut).await {
            Ok(Ok(r)) => r,
            _ => return precondition("mission operation lock did not answer"),
        };
        if acquire && !res.success {
            return precondition(format!("robot is busy: {}", res.message));
        }
        self.lock_held.store(acquire, Ordering::Relaxed);
        Ok(())
    }

    // ---------------------------------------------------------------- the run
    async fn run(self: Arc<Self>) {
        *self.started_mono.lock().unwrap() = Instant::now();
        self.alarm_logged.store(false, Ordering::Relaxed);
        {
            let mut st = self.status.lock().unwrap();
            *st = Status::blank("precheck", "checking the base");
            st.started_at = Some(unix_time_s());
        }
        self.publish_status();

        let mut old_gains: Option<WheelGains> = None;
        let mut old_closed_loop = true;
        let outcome: (&str, Option<String>) = match self.run_inner(&mut old_gains, &mut old_closed_loop).await {
            Ok(o) => o,
            Err(RunError::Aborted) => ("aborted", Some("aborted by operator".into())),
            Err(RunError::Precondition(e)) | Err(RunError::Tuning(e)) => ("failed", Some(e)),
        };

        // finally: whatever happened, the wheels stop and the gains come back
        self.tel.lock().unwrap().recording = None;
        self.override_wheels(0, 0, 0);
        if let Some(old) = &old_gains {
            self.abort.store(false, Ordering::Relaxed); // the restore must go through even after an abort
            let seq0 = self.send_pid(old, old_closed_loop, false);
            match self.wait_pid(old, seq0, old_closed_loop, 1.5, false).await {
                Ok(_) => {}
                Err(RunError::Aborted) => r2r::log_error!(&self.logger, "could not confirm the previous gains were restored: aborted"),
                Err(RunError::Precondition(e)) | Err(RunError::Tuning(e)) => r2r::log_error!(&self.logger, "could not confirm the previous gains were restored: {}", e),
            }
        }
        self.lights(false);
        if self.lock_held.load(Ordering::Relaxed) {
            if let Err(RunError::Precondition(e)) = self.lock(false).await {
                r2r::log_warn!(&self.logger, "{}", e);
            }
        }
        let (state, msg) = outcome;
        let error = if state == "done" || state == "idle" { None } else { msg.clone() };
        self.set_with(Some(state), if state == "done" { Some(1.0) } else { None }, msg.as_deref(), |st| st.error = error);
        self.active.store(false, Ordering::Release);
    }

    async fn run_inner(&self, old_gains: &mut Option<WheelGains>, old_closed_loop: &mut bool) -> Result<(&'static str, Option<String>), RunError> {
        let s = &self.settings;
        // 1. preconditions
        let pid = self.pid_status(None).await?;
        let old = gains_of(&pid)?;
        *old_gains = Some(old.clone());
        *old_closed_loop = int_of(&pid, "flags") & PID_FLAG_CLOSED_LOOP != 0;
        if self.nav_active.load(Ordering::Relaxed) {
            return precondition("navigation is active");
        }
        self.lock(true).await?;
        self.lights(true);
        let old_for_status = old.clone();
        self.set_with(None, Some(0.05), Some("base ok, switching to open loop"), |st| st.old_gains = Some(old_for_status));

        // 2. open-loop steps
        let hold = s.hold_s;
        let low = s.step_low_permille;
        let high = s.step_high_permille;
        let seq0 = self.send_pid(&old, false, false);
        self.wait_pid(&old, seq0, false, 1.5, false).await?;
        self.record(true);
        self.set(Some("open_loop"), Some(0.10), Some(&format!("open loop: 0 -> {:.0} % -> {:.0} % duty", low as f64 / 10.0, high as f64 / 10.0)));
        self.hold(0, 0.6).await?;
        let t_low = Instant::now();
        self.hold(low, hold).await?;
        let t_high = Instant::now();
        self.set(None, Some(0.30), None);
        self.hold(high, hold).await?;
        let t_high_end = Instant::now();
        self.hold(0, 0.3).await?;
        let rec = self.record(false).unwrap_or_default();
        {
            let samples = self.samples_json(&rec);
            let (lo, hi) = (self.rel(t_low), self.rel(t_high));
            let mut st = self.status.lock().unwrap();
            st.samples.open_loop = samples;
            st.marks.open_loop_low = Some(lo);
            st.marks.open_loop_high = Some(hi);
        }

        // 3. fit + tune (the low->high step; the 0->low one carries stiction)
        self.set(Some("fitting"), Some(0.50), Some("fitting the step response"));
        let mut models: Wheels<Option<FopdtModel>> = Wheels::default();
        let mut new_gains: Wheels<Gains> = Wheels { left: Gains { kp: 0.0, ki: 0.0, kd: 0.0 }, right: Gains { kp: 0.0, ki: 0.0, kd: 0.0 } };
        for w in WHEELS {
            let m = tuning::fit_fopdt(rec.get(w), now_secs(t_high), Some(now_secs(t_high_end))).map_err(|e| RunError::Tuning(format!("{w} wheel: {e}")))?;
            *new_gains.get_mut(w) = tuning::simc_pi(&m, &SimcOptions { tau_c_factor: s.tau_c_factor, ..SimcOptions::default() })?;
            *models.get_mut(w) = Some(m);
        }
        let models = Wheels { left: models.left.unwrap(), right: models.right.unwrap() };
        {
            let (m, g) = (models.clone(), new_gains.clone());
            self.set_with(None, Some(0.55), Some("gains computed, verifying closed loop"), |st| {
                st.model = Some(m);
                st.new_gains = Some(g);
            });
        }

        // 4. closed-loop verification with the new gains (RAM only)
        let seq0 = self.send_pid(&new_gains, true, false);
        self.wait_pid(&new_gains, seq0, true, 1.5, false).await?;
        self.record(true);
        self.set(Some("verify"), Some(0.60), Some(&format!("closed loop step to {:.0} % speed", s.verify_permille as f64 / 10.0)));
        self.hold(0, 0.5).await?;
        let t_v = Instant::now();
        let verify_hold = s.verify_hold_s;
        self.hold(s.verify_permille, verify_hold * 0.5).await?;
        let targets = {
            let tel = self.telemetry(None).await?;
            let one = |w: &str| obj(&tel, "wheel").and_then(|m| m.get(w)).and_then(Value::as_object).and_then(|side| float_of(side.get("target_rpm"))).unwrap_or(0.0);
            Wheels { left: one("left"), right: one("right") }
        };
        self.hold(s.verify_permille, verify_hold * 0.5).await?;
        let t_v_end = Instant::now();
        self.hold(0, 0.3).await?;
        let rec = self.record(false).unwrap_or_default();
        {
            let samples = self.samples_json(&rec);
            let mark = self.rel(t_v);
            let mut st = self.status.lock().unwrap();
            st.samples.verify = samples;
            st.marks.verify = Some(mark);
        }
        let mut verify: Wheels<Option<StepMetrics>> = Wheels::default();
        let mut problems: Vec<String> = Vec::new();
        for w in WHEELS {
            let m = tuning::step_metrics(rec.get(w), now_secs(t_v), *targets.get(w), Some(now_secs(t_v_end)))?;
            if m.overshoot_pct > s.max_overshoot_pct {
                problems.push(format!("{w} overshoot {:.0} %", m.overshoot_pct));
            }
            match m.settle_s {
                Some(t) if t <= s.max_settle_s => {}
                Some(t) => problems.push(format!("{w} settle {t:.2} s")),
                None => problems.push(format!("{w} settle never")),
            }
            if m.ss_error.abs() > s.max_ss_error_rpm {
                problems.push(format!("{w} steady-state error {:+.2} rpm", m.ss_error));
            }
            *verify.get_mut(w) = Some(m);
        }
        let verify = Wheels { left: verify.left.unwrap(), right: verify.right.unwrap() };
        self.set_with(None, Some(0.85), None, |st| st.verify = Some(verify));
        if !problems.is_empty() {
            return Err(RunError::Tuning(format!("verification failed: {}", problems.join(", "))));
        }

        // 5. review: operator decides
        self.set(Some("review"), Some(0.90), Some("tuned gains verified; apply to flash or discard"));
        let decided = self.wait_decision(s.review_timeout_s).await;
        if !decided {
            *self.decision.lock().unwrap() = Some("discard".into());
        }
        if self.abort.load(Ordering::Relaxed) {
            return Err(RunError::Aborted);
        }
        let decision = self.decision.lock().unwrap().clone();
        if decision.as_deref() == Some("apply") {
            self.set(Some("saving"), Some(0.95), Some("writing gains to STM32 flash"));
            self.persist(&new_gains).await?;
            *old_gains = None; // keep the new ones
            Ok(("done", Some("new gains saved to flash".into())))
        } else {
            Ok(("idle", Some("tuned gains discarded, previous gains restored".into())))
        }
    }

    /// `self._decision_event.wait(timeout)`: set by apply / discard / abort.
    async fn wait_decision(&self, timeout_s: f64) -> bool {
        let end = Instant::now() + Duration::from_secs_f64(timeout_s.max(0.0));
        loop {
            if self.decision.lock().unwrap().is_some() || self.abort.load(Ordering::Relaxed) {
                return true;
            }
            let now = Instant::now();
            if now >= end {
                return false;
            }
            tokio::time::sleep((end - now).min(Duration::from_millis(20))).await;
        }
    }
}

// ------------------------------------------------------- monotonic seconds
// Samples carry `time.monotonic()` in Python; here a process-wide anchor
// turns Instants into f64 seconds (and back for `_rel`).

fn anchor() -> Instant {
    static ANCHOR: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    *ANCHOR.get_or_init(Instant::now)
}

fn now_secs(at: Instant) -> f64 {
    at.saturating_duration_since(anchor()).as_secs_f64()
}

fn instant_of(secs: f64) -> Instant {
    anchor() + Duration::from_secs_f64(secs.max(0.0))
}

fn json_repr(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        Value::Null => "None".into(),
        Value::Bool(true) => "True".into(),
        Value::Bool(false) => "False".into(),
        other => other.to_string(),
    }
}

fn uuid_hex() -> String {
    use std::io::Read;
    let mut bytes = [0u8; 16];
    if let Ok(mut f) = std::fs::File::open("/dev/urandom") {
        let _ = f.read_exact(&mut bytes);
    }
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let _ = anchor();
    let ctx_r2r = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx_r2r, "pid_autotune", "")?;
    let logger = node.logger().to_string();

    let telemetry_topic = params::string(&node, "telemetry_topic", "/mower_base/telemetry");
    let pid_topic = params::string(&node, "pid_topic", "/mower_base/pid_command");
    let override_topic = params::string(&node, "override_topic", "/mower_base/wheel_override");
    let led_topic = params::string(&node, "led_topic", "/mower_base/led_command");
    let status_topic = params::string(&node, "status_topic", "/pid_autotune/status");
    let nav_active_topic = params::string(&node, "nav_active_topic", "/nav_operation_active");
    let lock_service = params::string(&node, "lock_service", "/mission_operation_lock");
    let settings = Settings {
        // open-loop identification: two forward steps so static friction at
        // standstill does not end up in the model
        step_low_permille: params::i64(&node, "step_low_permille", 400),
        step_high_permille: params::i64(&node, "step_high_permille", 700),
        hold_s: params::f64(&node, "hold_s", 2.5),
        // closed-loop check with the new gains
        verify_permille: params::i64(&node, "verify_permille", 500),
        verify_hold_s: params::f64(&node, "verify_hold_s", 3.0),
        tau_c_factor: params::f64(&node, "tau_c_factor", 1.0), // SIMC closed-loop time constant = factor * tau (>= dead time)
        max_overshoot_pct: params::f64(&node, "max_overshoot_pct", 25.0),
        max_settle_s: params::f64(&node, "max_settle_s", 1.5),
        max_ss_error_rpm: params::f64(&node, "max_ss_error_rpm", 1.5),
        telemetry_timeout_s: params::f64(&node, "telemetry_timeout_s", 0.5), // abort if the base goes quiet for this long
        review_timeout_s: params::f64(&node, "review_timeout_s", 300.0), // no apply/discard within this = discard
        led_busy_rgb: params::i64_array(&node, "led_busy_rgb", &[255, 180, 0]),
        led_busy_period_ms: params::i64(&node, "led_busy_period_ms", 1600),
        led_normal_rgb: params::i64_array(&node, "led_normal_rgb", &[110, 110, 110]),
    };

    let latched = QosProfile::default().keep_last(1).reliable().transient_local();
    let best_effort = QosProfile::default().keep_last(1).best_effort();

    let status_pub = node.create_publisher::<StringMsg>(&status_topic, latched.clone())?;
    let pid_pub = node.create_publisher::<StringMsg>(&pid_topic, QosProfile::default().keep_last(4))?;
    let override_pub = node.create_publisher::<StringMsg>(&override_topic, best_effort.clone())?;
    let led_pub = if led_topic.is_empty() { None } else { Some(Mutex::new(node.create_publisher::<StringMsg>(&led_topic, latched)?)) };
    let lock_client = node.create_client::<MissionOperationLock::Service>(&lock_service, QosProfile::services_default())?;
    let lock_owner = format!("{}:{}", node.fully_qualified_name()?, uuid_hex());

    let ctx = Arc::new(Ctx {
        logger: logger.clone(),
        settings,
        status: Mutex::new(Status::blank("idle", "ready")),
        tel: Mutex::new(Telemetry { latest: None, seq: None, recording: None }),
        nav_active: AtomicBool::new(false),
        active: AtomicBool::new(false),
        abort: AtomicBool::new(false),
        alarm_logged: AtomicBool::new(false),
        lock_held: AtomicBool::new(false),
        decision: Mutex::new(None),
        started_mono: Mutex::new(Instant::now()),
        lock_owner,
        status_pub: Mutex::new(status_pub),
        pid_pub: Mutex::new(pid_pub),
        override_pub: Mutex::new(override_pub),
        led_pub,
        lock_client,
    });

    // ---- inputs ------------------------------------------------------------
    {
        let mut stream = node.subscribe::<Bool>(&nav_active_topic, QosProfile::default().keep_last(10))?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                ctx.nav_active.store(msg.data, Ordering::Relaxed);
            }
        });
    }
    {
        let mut stream = node.subscribe::<StringMsg>(&telemetry_topic, best_effort)?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(msg) = stream.next().await {
                ctx.on_telemetry(&msg.data);
            }
        });
    }

    // ---- service ----------------------------------------------------------
    {
        let mut stream = node.create_service::<PidAutotune::Service>("/pid_autotune", QosProfile::services_default())?;
        let ctx = ctx.clone();
        tokio::spawn(async move {
            while let Some(req) = stream.next().await {
                let op = req.message.op.trim().to_lowercase();
                let (success, message) = match op.as_str() {
                    "start" => {
                        if ctx.active.load(Ordering::Acquire) {
                            (false, format!("auto-tune already running ({})", ctx.status.lock().unwrap().state))
                        } else {
                            ctx.abort.store(false, Ordering::Relaxed);
                            *ctx.decision.lock().unwrap() = None;
                            ctx.tel.lock().unwrap().latest = None;
                            ctx.active.store(true, Ordering::Release);
                            tokio::spawn(ctx.clone().run());
                            (true, "auto-tune started".to_string())
                        }
                    }
                    "abort" => {
                        if !ctx.active.load(Ordering::Acquire) {
                            (false, "nothing to abort".to_string())
                        } else {
                            ctx.abort.store(true, Ordering::Relaxed);
                            (true, "aborting".to_string())
                        }
                    }
                    "apply" | "discard" => {
                        if !ctx.active.load(Ordering::Acquire) || ctx.status.lock().unwrap().state != "review" {
                            (false, "no tuned gains waiting for review".to_string())
                        } else {
                            *ctx.decision.lock().unwrap() = Some(op.clone());
                            (true, op.clone())
                        }
                    }
                    _ => (false, format!("unknown op {} (start|abort|apply|discard)", python_repr(&req.message.op))),
                };
                let _ = req.respond(PidAutotune::Response { success, message });
            }
        });
    }

    // ---- 5 Hz progress while a session is active --------------------------
    {
        let ctx = ctx.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_millis(200));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                if ctx.active.load(Ordering::Acquire) {
                    ctx.publish_status();
                }
            }
        });
    }

    ctx.publish_status();
    r2r::log_info!(&logger, "pid_autotune ready: service /pid_autotune, status on {}", status_topic);

    // ---- spin until SIGINT / SIGTERM ----------------------------------------
    let running = Arc::new(AtomicBool::new(true));
    let spin = {
        let running = running.clone();
        tokio::task::spawn_blocking(move || {
            while running.load(Ordering::Relaxed) {
                node.spin_once(Duration::from_millis(100));
            }
            drop(node);
        })
    };
    let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = sigterm.recv() => {}
    }
    running.store(false, Ordering::Relaxed);
    let _ = spin.await;
    Ok(())
}

/// Python `repr()` of a str for the "unknown op" message.
fn python_repr(s: &str) -> String {
    if s.contains('\'') && !s.contains('"') {
        format!("\"{s}\"")
    } else {
        format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
    }
}

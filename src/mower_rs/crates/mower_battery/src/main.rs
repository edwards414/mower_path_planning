//! mower_battery: `sensor_msgs/BatteryState` for the real base from STM32
//! telemetry (port of `mower_mission/battery_state_node.py`, node name
//! `battery_state`, same parameters, topics and messages).
//!
//! Reads the `charger` (0x89: RS485 voltage / current / temperature meter in
//! the battery pack lead) and `analog` (0x8A: STM32 ADC, only when those
//! channels are populated) objects of `/mower_base/telemetry` (16.7 Hz JSON)
//! and runs [`estimator::BatteryEstimator`] on them:
//!
//! * `/battery_state`     -- 24 V main pack (6S): voltage, percentage,
//!   charging / discharging / full, signed current once the meter's shunt is
//!   wired into the lead (`meter_current_wired`)
//! * `/aon_battery_state` -- 3.7 V always-on cell; `present` only when the
//!   STM32 ADC divider for it exists
//!
//! Pack voltage: the meter when it answers, else the STM32 ADC main-battery
//! channel when valid. Charger presence: a pack voltage at or above
//! `charger_present_min_v` (the charger's CV, above any resting OCV).
//! `percentage` is NaN and `present` false until a valid reading arrives,
//! and again if telemetry goes stale for `stale_timeout_s`.
//!
//! rclpy spent ~10 % of a core on the 16.7 Hz JSON callbacks; this needs
//! well under 1 %.

mod estimator;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::StreamExt;
use mower_rs_common::params;
use r2r::builtin_interfaces::msg::Time;
use r2r::sensor_msgs::msg::BatteryState;
use r2r::std_msgs::msg::String as StringMsg;
use r2r::QosProfile;
use serde_json::Value;

use crate::estimator::{BatteryEstimator, Config, Status};

// sensor_msgs/BatteryState constants (r2r does not export them).
const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
const POWER_SUPPLY_STATUS_FULL: u8 = 4;
const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;

fn now_s() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

fn stamp_now() -> Time {
    let ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);
    Time { sec: (ns / 1_000_000_000) as i32, nanosec: (ns % 1_000_000_000) as u32 }
}

/// Python truthiness of a JSON value (`bool(charger.get('valid'))`).
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

/// `float(obj.get(key, nan))`; a value Python could not convert is NaN here
/// instead of an exception that would take the node down.
fn number(obj: &serde_json::Map<String, Value>, key: &str) -> f64 {
    match obj.get(key) {
        Some(Value::Number(n)) => n.as_f64().unwrap_or(f64::NAN),
        Some(Value::Bool(b)) => {
            if *b { 1.0 } else { 0.0 }
        }
        Some(Value::String(s)) => s.trim().parse::<f64>().unwrap_or(f64::NAN),
        _ => f64::NAN,
    }
}

/// `data.get('analog') or {}`
fn object<'a>(data: &'a Value, key: &str) -> serde_json::Map<String, Value> {
    match data.get(key) {
        Some(Value::Object(o)) => o.clone(),
        _ => serde_json::Map::new(),
    }
}

struct Settings {
    stale_timeout_s: f64,
    low_battery_pct: f64,
    frame_id: String,
    charger_present_min_v: f64,
    meter_current_wired: bool,
    meter_current_signed: bool,
    capacity_ah: f64,
}

struct State {
    main: BatteryEstimator,
    aon: BatteryEstimator,
    last_main_time: Option<f64>,
    last_aon_time: Option<f64>,
    low_warned: bool,
}

impl State {
    fn on_base_telemetry(&mut self, s: &Settings, logger: &str, text: &str) {
        let Ok(data) = serde_json::from_str::<Value>(text) else { return };
        let analog = object(&data, "analog");
        let charger = object(&data, "charger");
        let now = now_s();

        let meter_online = truthy(charger.get("valid")) && truthy(charger.get("online"));
        let mut voltage = f64::NAN;
        if meter_online {
            voltage = number(&charger, "voltage_v");
        } else if truthy(analog.get("valid")) && truthy(analog.get("main_battery_valid")) {
            voltage = number(&analog, "main_battery_v");
        }

        if voltage.is_finite() && voltage > 0.0 {
            let charger_present = voltage >= s.charger_present_min_v;
            let mut current = f64::NAN;
            if meter_online && s.meter_current_wired {
                current = signed_current(s, number(&charger, "current_a"), charger_present);
            }
            self.main.update(now, voltage, charger_present, current);
            self.last_main_time = Some(now);
            self.check_low(s, logger);
        }

        if truthy(analog.get("valid")) && truthy(analog.get("aon_battery_valid")) {
            self.aon.update(now, number(&analog, "aon_battery_v"), false, f64::NAN);
            self.last_aon_time = Some(now);
        }
    }

    fn check_low(&mut self, s: &Settings, logger: &str) {
        let pct = self.main.percentage();
        if pct.is_nan() {
            return;
        }
        if pct <= s.low_battery_pct && !matches!(self.main.status, Status::Charging | Status::Full) {
            if !self.low_warned {
                r2r::log_warn!(logger, "main battery low: {:.0}% ({:.2} V)", pct, self.main.voltage_v);
                self.low_warned = true;
            }
        } else if pct > s.low_battery_pct + 5.0 {
            self.low_warned = false;
        }
    }
}

/// Meter register -> ROS sign convention (+ charging, - discharging).
fn signed_current(s: &Settings, magnitude_a: f64, charger_present: bool) -> f64 {
    if !magnitude_a.is_finite() {
        return f64::NAN;
    }
    if s.meter_current_signed {
        // uint16 x0.01 A that wraps for negative values
        return if magnitude_a > 327.67 { magnitude_a - 655.36 } else { magnitude_a };
    }
    if charger_present { magnitude_a.abs() } else { -magnitude_a.abs() }
}

fn message(s: &Settings, est: &BatteryEstimator, last_time: Option<f64>, now: f64) -> BatteryState {
    let mut msg = BatteryState::default();
    msg.header.stamp = stamp_now();
    msg.header.frame_id = s.frame_id.clone();
    msg.power_supply_technology = POWER_SUPPLY_TECHNOLOGY_LION;
    msg.design_capacity = f32::NAN;
    msg.capacity = f32::NAN;
    msg.charge = f32::NAN;
    msg.temperature = f32::NAN;

    let fresh = matches!(last_time, Some(t) if now - t <= s.stale_timeout_s);
    if !fresh || est.fraction.is_nan() {
        msg.present = false;
        msg.voltage = f32::NAN;
        msg.current = f32::NAN;
        msg.percentage = f32::NAN;
        msg.power_supply_status = POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health = POWER_SUPPLY_HEALTH_UNKNOWN;
        return msg;
    }

    msg.present = true;
    msg.voltage = est.voltage_v as f32;
    // ROS convention: positive current = charging; NaN until the meter's
    // shunt is wired into the pack lead (docs/BATTERY.md).
    msg.current = est.current_a as f32;
    msg.percentage = est.fraction as f32;
    if est.capacity_ah() > 0.0 {
        msg.design_capacity = est.capacity_ah() as f32;
        msg.capacity = est.capacity_ah() as f32;
        msg.charge = (est.capacity_ah() * est.fraction) as f32;
    }
    msg.power_supply_status = match est.status {
        Status::Charging => POWER_SUPPLY_STATUS_CHARGING,
        Status::Full => POWER_SUPPLY_STATUS_FULL,
        Status::NotCharging => POWER_SUPPLY_STATUS_NOT_CHARGING,
        Status::Discharging | Status::Unknown => POWER_SUPPLY_STATUS_DISCHARGING,
    };
    msg.power_supply_health = POWER_SUPPLY_HEALTH_GOOD;
    if est.cell_voltage_v() < 3.0 {
        msg.power_supply_health = POWER_SUPPLY_HEALTH_DEAD;
    } else if est.cell_voltage_v() > 4.3 {
        msg.power_supply_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    // rclpy keeps the f64 in the field until serialisation, so the division
    // happens in f64 and only the result is narrowed.
    msg.cell_voltage = vec![(est.voltage_v / est.cell_count() as f64) as f32; est.cell_count()];
    msg
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "battery_state", "")?;
    let logger = node.logger().to_string();

    let base_telemetry_topic = params::string(&node, "base_telemetry_topic", "/mower_base/telemetry");
    let battery_topic = params::string(&node, "battery_topic", "/battery_state");
    let aon_battery_topic = params::string(&node, "aon_battery_topic", "/aon_battery_state");
    let publish_rate_hz = params::f64(&node, "publish_rate_hz", 1.0);
    let main_cell_count = params::i64(&node, "main_cell_count", 6);
    let filter_tau_s = params::f64(&node, "filter_tau_s", 20.0);
    let recovery = params::f64(&node, "recovery_rate_pct_per_min", 1.0) / 100.0 / 60.0;
    // Voltage-only charging ramp: 100 % / (capacity_ah / charger_a) / 60.
    let charge_rate = params::f64(&node, "charge_rate_pct_per_min", 0.5) / 100.0 / 60.0;
    let post_charge_settle_s = params::f64(&node, "post_charge_settle_s", 300.0);
    let capacity_ah = params::f64(&node, "capacity_ah", 0.0);
    let settings = Settings {
        stale_timeout_s: params::f64(&node, "stale_timeout_s", 5.0),
        low_battery_pct: params::f64(&node, "low_battery_pct", 20.0),
        frame_id: params::string(&node, "frame_id", "base_footprint"),
        // Pack voltage at or above this = charger connected (its CV).
        charger_present_min_v: params::f64(&node, "charger_present_min_v", 25.0),
        // Set once the meter's shunt carries the pack current. The meter's
        // register is unsigned: the sign is taken from charger presence
        // unless meter_current_signed says the register wraps for discharge.
        meter_current_wired: params::bool(&node, "meter_current_wired", false),
        meter_current_signed: params::bool(&node, "meter_current_signed", false),
        // Pack capacity for coulomb counting; 0 = unknown (voltage only).
        capacity_ah,
    };
    if main_cell_count < 1 {
        return Err("main_cell_count must be >= 1".into());
    }
    let main = BatteryEstimator::new(Config {
        cell_count: main_cell_count as usize,
        filter_tau_s,
        recovery_rate_per_s: recovery,
        charge_rate_per_s: charge_rate,
        post_charge_settle_s,
        capacity_ah: if capacity_ah > 0.0 { capacity_ah } else { f64::NAN },
        rest_current_a: params::f64(&node, "rest_current_a", 0.1),
        rest_hold_s: params::f64(&node, "rest_hold_s", 300.0),
        full_tail_current_a: params::f64(&node, "full_tail_current_a", 0.2),
        full_hold_s: params::f64(&node, "full_hold_s", 60.0),
        full_min_cell_v: params::f64(&node, "full_min_cell_v", 4.10),
    })?;
    // The AON cell is charged by its own on-board charger the STM32 does
    // not see, so it only gets the voltage lookup.
    let aon = BatteryEstimator::new(Config {
        cell_count: 1,
        filter_tau_s,
        recovery_rate_per_s: recovery,
        ..Config::default()
    })?;
    let settings = Arc::new(settings);
    let state = Arc::new(Mutex::new(State { main, aon, last_main_time: None, last_aon_time: None, low_warned: false }));

    let battery_pub = node.create_publisher::<BatteryState>(&battery_topic, QosProfile::default())?;
    let aon_pub = node.create_publisher::<BatteryState>(&aon_battery_topic, QosProfile::default())?;
    let mut telemetry = node.subscribe::<StringMsg>(&base_telemetry_topic, QosProfile::default().keep_last(1).best_effort())?;

    {
        let state = state.clone();
        let settings = settings.clone();
        let logger = logger.clone();
        tokio::spawn(async move {
            while let Some(msg) = telemetry.next().await {
                state.lock().unwrap().on_base_telemetry(&settings, &logger, &msg.data);
            }
        });
    }
    {
        let state = state.clone();
        let settings = settings.clone();
        let logger = logger.clone();
        let period = if publish_rate_hz > 0.0 { 1.0 / publish_rate_hz } else { 1.0 };
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs_f64(period));
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                let (main, aon) = {
                    let st = state.lock().unwrap();
                    let now = now_s();
                    (message(&settings, &st.main, st.last_main_time, now), message(&settings, &st.aon, st.last_aon_time, now))
                };
                if let Err(e) = battery_pub.publish(&main) {
                    r2r::log_error!(&logger, "publish battery_state failed: {:?}", e);
                }
                if let Err(e) = aon_pub.publish(&aon) {
                    r2r::log_error!(&logger, "publish aon_battery_state failed: {:?}", e);
                }
            }
        });
    }
    let mode = if settings.meter_current_wired && settings.capacity_ah > 0.0 { "coulomb counting" } else { "OCV estimate" };
    r2r::log_info!(&logger, "battery_state: {}S {} from {} -> {}", main_cell_count, mode, base_telemetry_topic, battery_topic);

    // ---- spin until SIGINT / SIGTERM (the subscription keeps the wait set
    // non-empty, so spin_once blocks up to its timeout) ---------------------
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

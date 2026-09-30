//! STM32 mower UART protocol.
//!
//! Port of `src/mower_hardware/src/mower_protocol.cpp` +
//! `include/mower_hardware/mower_protocol.hpp` (see `UART_OPEN_LOOP_PROTOCOL.md`).
//! Frame layout, little-endian throughout:
//!
//! ```text
//!   0    1    2    3    4    5    6 ..            n-2  n-1
//!  A5   5A  ver type  seq  len  payload[len]      crc_lo crc_hi
//!                     \___ CRC-16/CCITT-FALSE over ver..payload ___/
//! ```
//!
//! The encoders must stay byte-identical with the C++ ones and the decoders
//! field-identical; `tests/protocol_vectors.rs` checks that against vectors
//! emitted by a harness linked against the original C++ source.

pub const SOF0: u8 = 0xA5;
pub const SOF1: u8 = 0x5A;
pub const PROTOCOL_VERSION: u8 = 0x01;
/// sof0 sof1 ver type seq len ... crc_lo crc_hi
pub const FRAME_OVERHEAD: usize = 8;

// Frame types (host -> STM32)
pub const WHEEL_SPEED_COMMAND: u8 = 0x01;
pub const LAWER_MOTOR_COMMAND: u8 = 0x02;
pub const WS2812_COMMAND: u8 = 0x03;
pub const PID_CONFIG_COMMAND: u8 = 0x04;
pub const POWER_COMMAND: u8 = 0x05;
pub const INFO_REQUEST: u8 = 0x06;
pub const SERVO_COMMAND: u8 = 0x07;
// Frame types (STM32 -> host)
pub const MOTOR_STATUS: u8 = 0x81;
pub const LAWER_MOTOR_STATUS: u8 = 0x82;
pub const WS2812_STATUS: u8 = 0x83;
pub const PID_CONFIG_STATUS: u8 = 0x84;
pub const WHEEL_FEEDBACK_STATUS: u8 = 0x85;
pub const POWER_STATUS: u8 = 0x86;
pub const FIRMWARE_INFO: u8 = 0x87;
pub const SERVO_STATUS: u8 = 0x88;
pub const CHARGER_STATUS: u8 = 0x89;
// 0x8A (analog status) is retired: the ADC mux was never fitted.

// 0x03 payload mode (firmware/LED_COMMAND_MODES.md)
pub const LED_CLEAR: u8 = 0x00;
pub const LED_ALL_ON: u8 = 0x01;
pub const LED_FLOW: u8 = 0x02;
pub const LED_TURN_LEFT: u8 = 0x03;
pub const LED_TURN_RIGHT: u8 = 0x04;
pub const LED_SHOW: u8 = 0x05;
/// smooth comet around the strips (update indicator)
pub const LED_ORBIT: u8 = 0x06;

// 0x87 payload build_flags
pub const FW_BUILD_FLAG_DIRTY: u8 = 0x01;
pub const FW_BUILD_FLAG_UNVERSIONED: u8 = 0x02;

// 0x89 charger status flags
pub const CHARGER_FLAG_ONLINE: u8 = 0x01;
pub const CHARGER_FLAG_CURRENT_PRESENT: u8 = 0x02;
pub const CHARGER_FLAG_INPUT_PRESENT: u8 = 0x08;
pub const CHARGER_FLAG_EVER_SEEN: u8 = 0x10;

// 0x88 MG996 servo status flags
pub const SERVO_FLAG_ENABLED: u8 = 0x01;
pub const SERVO_FLAG_LIMIT_ACTIVE: u8 = 0x02;
pub const SERVO_FLAG_OUTPUT_ACTIVE: u8 = 0x04;
pub const SERVO_FLAG_TIMED_OUT: u8 = 0x08;
pub const SERVO_FLAG_LIMIT_UP: u8 = 0x10;
pub const SERVO_FLAG_LIMIT_DOWN: u8 = 0x20;

/// 0x07 servo command pulse range; 0 releases the servo.
pub const SERVO_MIN_PULSE_US: u16 = 500;
pub const SERVO_MAX_PULSE_US: u16 = 2500;

// 0x05 payload action
pub const POWER_ACTION_NONE: u8 = 0x00;
pub const POWER_ACTION_HOST_SHUTDOWN_ACK: u8 = 0x01;
pub const POWER_ACTION_REQUEST_SHUTDOWN: u8 = 0x02;
pub const POWER_ACTION_CANCEL_SHUTDOWN: u8 = 0x03;
pub const POWER_ACTION_FORCE_POWER_OFF: u8 = 0x04;

// 0x86 payload state (power_manager_state_t on the STM32)
pub const POWER_STATE_RUNNING: u8 = 0;
pub const POWER_STATE_LOW_POWER: u8 = 1;
pub const POWER_STATE_WAKE_PULSE: u8 = 2;
pub const POWER_STATE_SHUTDOWN_PENDING: u8 = 3;
pub const POWER_STATE_LIGHTS_OFF: u8 = 4;

// 0x86 payload flags
pub const POWER_FLAG_BUTTON_PRESSED: u8 = 0x01;
pub const POWER_FLAG_MAIN_POWER_ENABLED: u8 = 0x02;
pub const POWER_FLAG_SHUTDOWN_REQUESTED: u8 = 0x04;
pub const POWER_FLAG_HOST_ACK_RECEIVED: u8 = 0x08;
pub const POWER_FLAG_WAKE_ASSERTED: u8 = 0x10;

// 0x81 payload flags
pub const STATUS_FLAG_COMMAND_VALID: u8 = 0x01;
pub const STATUS_FLAG_COMMAND_TIMEOUT: u8 = 0x02;
pub const STATUS_FLAG_DRIVER_ALARM: u8 = 0x04;

// 0x85 payload flags
pub const WHEEL_FLAG_OUTPUT_ENABLED: u8 = 0x01;
pub const WHEEL_FLAG_CLOSED_LOOP: u8 = 0x02;

// 0x84 payload flags
pub const PID_FLAG_CLOSED_LOOP: u8 = 0x01;
pub const PID_FLAG_FLASH_VALID: u8 = 0x02;
pub const PID_FLAG_LAST_SAVE_OK: u8 = 0x04;
pub const PID_FLAG_LAST_APPLY_OK: u8 = 0x08;

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor).
pub fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// Build a complete frame (SOF + header + payload + CRC).
///
/// `len` is written as a single byte, so payloads longer than 255 bytes
/// truncate the length field exactly as the C++ `static_cast<uint8_t>(len)`
/// does; nothing in the protocol sends one.
pub fn build_frame(frame_type: u8, seq: u8, payload: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(FRAME_OVERHEAD + payload.len());
    f.push(SOF0);
    f.push(SOF1);
    f.push(PROTOCOL_VERSION);
    f.push(frame_type);
    f.push(seq);
    f.push(payload.len() as u8);
    f.extend_from_slice(payload);
    let crc = crc16_ccitt_false(&f[2..]); // version..payload
    f.push((crc & 0xFF) as u8);
    f.push((crc >> 8) as u8);
    f
}

#[inline]
fn put_i16(p: &mut [u8], v: i16) {
    let u = v as u16;
    p[0] = (u & 0xFF) as u8;
    p[1] = (u >> 8) as u8;
}
#[inline]
fn put_u16(p: &mut [u8], v: u16) {
    p[0] = (v & 0xFF) as u8;
    p[1] = (v >> 8) as u8;
}
#[inline]
fn get_i16(p: &[u8]) -> i16 {
    (p[0] as u16 | ((p[1] as u16) << 8)) as i16
}
#[inline]
fn get_u16(p: &[u8]) -> u16 {
    p[0] as u16 | ((p[1] as u16) << 8)
}
#[inline]
fn get_u32(p: &[u8]) -> u32 {
    p[0] as u32 | ((p[1] as u32) << 8) | ((p[2] as u32) << 16) | ((p[3] as u32) << 24)
}
#[inline]
fn get_i32(p: &[u8]) -> i32 {
    get_u32(p) as i32
}
#[inline]
fn get_f32(p: &[u8]) -> f32 {
    f32::from_le_bytes([p[0], p[1], p[2], p[3]])
}

/// 0x01: wheel speed, permille of `max_rpm`, with the STM32's own timeout.
pub fn build_wheel_speed_command(
    seq: u8,
    left_permille: i16,
    right_permille: i16,
    timeout_ms: u16,
) -> Vec<u8> {
    let mut p = [0u8; 8];
    put_i16(&mut p[0..], left_permille);
    put_i16(&mut p[2..], right_permille);
    put_u16(&mut p[4..], timeout_ms);
    put_u16(&mut p[6..], 0);
    build_frame(WHEEL_SPEED_COMMAND, seq, &p)
}

/// 0x02: blade (BLD120A) motor, permille.
pub fn build_lawer_motor_command(seq: u8, permille: i16, timeout_ms: u16) -> Vec<u8> {
    let mut p = [0u8; 8];
    put_i16(&mut p[0..], permille);
    put_u16(&mut p[2..], timeout_ms);
    put_u16(&mut p[4..], 0);
    put_u16(&mut p[6..], 0);
    build_frame(LAWER_MOTOR_COMMAND, seq, &p)
}

/// 0x07: lift servo. `pulse_us` 500-2500 (0 releases); `hold_timeout_ms` 0 =
/// hold until the next command.
pub fn build_servo_command(seq: u8, pulse_us: u16, hold_timeout_ms: u16) -> Vec<u8> {
    let mut p = [0u8; 8];
    put_u16(&mut p[0..], pulse_us);
    put_u16(&mut p[2..], hold_timeout_ms);
    put_u16(&mut p[4..], 0);
    put_u16(&mut p[6..], 0);
    build_frame(SERVO_COMMAND, seq, &p)
}

/// 0x05: power action.
pub fn build_power_command(seq: u8, action: u8) -> Vec<u8> {
    let p = [action, 0, 0, 0];
    build_frame(POWER_COMMAND, seq, &p)
}

/// 0x06: ask for the 0x87 firmware info.
pub fn build_info_request(seq: u8) -> Vec<u8> {
    build_frame(INFO_REQUEST, seq, &[])
}

/// 0x03: WS2812 strips.
pub fn build_ws2812_command(
    seq: u8,
    mode: u8,
    r: u8,
    g: u8,
    b: u8,
    effect_period_ms: u16,
) -> Vec<u8> {
    let mut p = [mode, r, g, b, 0, 0, 0, 0];
    put_u16(&mut p[4..], effect_period_ms);
    build_frame(WS2812_COMMAND, seq, &p)
}

/// 0x04: gains to run with (and optionally persist). Same float layout as
/// [`PidConfigStatus`]; the STM32 sanitises out-of-range values itself.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PidConfig {
    pub left_kp: f32,
    pub left_ki: f32,
    pub left_kd: f32,
    pub right_kp: f32,
    pub right_ki: f32,
    pub right_kd: f32,
    /// write sector 7 (do not do this at high rate)
    pub persist_to_flash: bool,
    /// false: 0x01 permille drives PWM duty directly
    pub closed_loop_enabled: bool,
}

/// `<ffffffBBH>`: six little-endian floats, persist, closed_loop, reserved.
pub fn build_pid_config_command(seq: u8, cfg: &PidConfig) -> Vec<u8> {
    let mut p = [0u8; 28];
    let gains = [
        cfg.left_kp,
        cfg.left_ki,
        cfg.left_kd,
        cfg.right_kp,
        cfg.right_ki,
        cfg.right_kd,
    ];
    for (i, g) in gains.iter().enumerate() {
        p[i * 4..i * 4 + 4].copy_from_slice(&g.to_le_bytes());
    }
    p[24] = cfg.persist_to_flash as u8;
    p[25] = cfg.closed_loop_enabled as u8;
    put_u16(&mut p[26..], 0);
    build_frame(PID_CONFIG_COMMAND, seq, &p)
}

/// Incremental byte-stream parser.
///
/// Reproduces the C++ `FrameParser` exactly, including its resync rules, which
/// are what makes the driver survive the `VMIN=0` non-blocking read path in
/// `serial_port.cpp` (arbitrary chunk boundaries) and line noise:
///
/// * bytes before a `A5 5A` pair are dropped;
/// * a trailing lone `0xA5` is kept in case `0x5A` arrives in the next read;
/// * a frame whose CRC fails or whose version byte is not
///   [`PROTOCOL_VERSION`] bumps [`FrameParser::crc_errors`] and the buffer is
///   rewound past that SOF only (two bytes), so a real frame hiding inside a
///   corrupt one is still found;
/// * an incomplete frame is left in the buffer until the rest arrives.
#[derive(Debug, Default, Clone)]
pub struct FrameParser {
    buf: Vec<u8>,
    crc_errors: usize,
}

impl FrameParser {
    pub fn new() -> Self {
        Self::default()
    }

    /// Frames rejected by the CRC or the version check since construction.
    pub fn crc_errors(&self) -> usize {
        self.crc_errors
    }

    /// Bytes still held waiting for the rest of a frame. Diagnostics only.
    pub fn buffered(&self) -> usize {
        self.buf.len()
    }

    /// Feed one `read()` worth of bytes; `on_frame(type, seq, payload)` runs
    /// for every CRC-valid frame, in order.
    pub fn feed<F: FnMut(u8, u8, &[u8])>(&mut self, data: &[u8], mut on_frame: F) {
        self.buf.extend_from_slice(data);

        loop {
            // find SOF
            let mut i = 0usize;
            while i + 1 < self.buf.len() && !(self.buf[i] == SOF0 && self.buf[i + 1] == SOF1) {
                i += 1;
            }
            if i + 1 >= self.buf.len() {
                // keep a trailing 0xA5 in case 0x5A arrives next
                if self.buf.last() == Some(&SOF0) {
                    let keep = self.buf.len() - 1;
                    self.buf.drain(..keep);
                } else {
                    self.buf.clear();
                }
                return;
            }
            if i > 0 {
                self.buf.drain(..i);
            }
            if self.buf.len() < FRAME_OVERHEAD {
                return;
            }
            let plen = self.buf[5] as usize;
            let total = FRAME_OVERHEAD + plen;
            if self.buf.len() < total {
                return;
            }
            let crc_rx = get_u16(&self.buf[6 + plen..]);
            let crc_calc = crc16_ccitt_false(&self.buf[2..6 + plen]);
            if crc_rx == crc_calc && self.buf[2] == PROTOCOL_VERSION {
                on_frame(self.buf[3], self.buf[4], &self.buf[6..6 + plen]);
                self.buf.drain(..total);
            } else {
                self.crc_errors += 1;
                self.buf.drain(..2); // resync past this SOF
            }
        }
    }
}

/// 0x85, every 50 ms: the closed-loop wheel controller's own view.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelFeedback {
    pub left_target_rpm: f64,
    pub left_measured_rpm: f64,
    pub right_target_rpm: f64,
    pub right_measured_rpm: f64,
    pub left_pid_output: i16,
    pub right_pid_output: i16,
    pub left_total_counts: i32,
    pub right_total_counts: i32,
    pub flags: u8,
    pub seq: u8,
}

/// 0x81: what the wheel driver did with the last 0x01.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MotorStatus {
    pub commanded_left_permille: i16,
    pub commanded_right_permille: i16,
    pub applied_left_pwm: i16,
    pub applied_right_pwm: i16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
}

/// 0x83: what the strips are currently showing.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Ws2812Status {
    pub mode: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub effect_period_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
}

/// 0x84: PID gains the wheel controller is running with.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PidConfigStatus {
    pub left_kp: f32,
    pub left_ki: f32,
    pub left_kd: f32,
    pub right_kp: f32,
    pub right_ki: f32,
    pub right_kd: f32,
    pub flags: u8,
    pub last_rx_seq: u8,
    /// low byte: HAL flash error of the last failed persist (0 = ok), high
    /// byte: FLASH_SR error bits found pending before it; 0 on older firmware
    pub flash_diag: u16,
}

/// 0x86: power manager.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PowerStatus {
    pub state: u8,
    pub flags: u8,
    /// 0 none, 1 button, 2 host, 3 forced
    pub shutdown_reason: u8,
    pub last_rx_seq: u8,
    pub press_ms: u16,
    pub shutdown_elapsed_ms: u16,
}

impl PowerStatus {
    pub fn shutdown_requested(&self) -> bool {
        self.flags & POWER_FLAG_SHUTDOWN_REQUESTED != 0
    }
}

/// 0x87: build identity of the running application (`firmware_version.h`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FirmwareInfo {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
    pub protocol_version: u8,
    /// first 4 bytes of the commit hash, 0 = unknown
    pub git_sha32: u32,
    /// build time, 0 = unknown
    pub build_unix: u32,
    pub build_flags: u8,
}

impl FirmwareInfo {
    pub fn dirty(&self) -> bool {
        self.build_flags & FW_BUILD_FLAG_DIRTY != 0
    }
    pub fn unversioned(&self) -> bool {
        self.build_flags & FW_BUILD_FLAG_UNVERSIONED != 0
    }
    /// `"0.6.0+abc12345.dirty"`, for logs.
    pub fn version_string(&self) -> String {
        format!(
            "{}.{}.{}+{:08x}{}{}",
            self.major,
            self.minor,
            self.patch,
            self.git_sha32,
            if self.dirty() { ".dirty" } else { "" },
            if self.unversioned() { ".unversioned" } else { "" }
        )
    }
    /// JSON with all fields, what `/mower_base/firmware_info` carries.
    pub fn to_json(&self) -> String {
        format!(
            concat!(
                "{{\"version\":\"{maj}.{min}.{pat}\",\"semver\":[{maj},{min},{pat}],",
                "\"protocol_version\":{proto},\"git_sha\":\"{sha:08x}\",\"git_sha32\":{sha},",
                "\"build_unix\":{build},\"dirty\":{dirty},\"unversioned\":{unver}}}"
            ),
            maj = self.major,
            min = self.minor,
            pat = self.patch,
            proto = self.protocol_version,
            sha = self.git_sha32,
            build = self.build_unix,
            dirty = self.dirty(),
            unver = self.unversioned()
        )
    }
}

/// 0x89: RS485 voltage / current / temperature meter in the battery lead.
/// Values are the last valid reply (or 0); trust them only when `online()`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ChargerStatus {
    /// battery terminal voltage, x0.01 V
    pub voltage_cv: u16,
    /// pack current magnitude, x0.01 A
    pub current_ca: u16,
    /// meter temperature, degC
    pub temp_c: u16,
    pub reg3: u16,
    pub reg4: u16,
    pub flags: u8,
    pub comm_error_count: u8,
    /// since the last valid reply, 0xFFFF = never
    pub age_ms: u16,
    pub last_exception_code: u8,
}

impl ChargerStatus {
    pub fn online(&self) -> bool {
        self.flags & CHARGER_FLAG_ONLINE != 0
    }
    pub fn current_present(&self) -> bool {
        self.flags & CHARGER_FLAG_CURRENT_PRESENT != 0
    }
    pub fn input_present(&self) -> bool {
        self.flags & CHARGER_FLAG_INPUT_PRESENT != 0
    }
}

/// 0x88: MG996 blade-lift servo, every 50 ms.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ServoStatus {
    pub pulse_us: u16,
    pub hold_timeout_ms: u16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
}

impl ServoStatus {
    pub fn enabled(&self) -> bool {
        self.flags & SERVO_FLAG_ENABLED != 0
    }
    pub fn limit_active(&self) -> bool {
        self.flags & SERVO_FLAG_LIMIT_ACTIVE != 0
    }
    pub fn output_active(&self) -> bool {
        self.flags & SERVO_FLAG_OUTPUT_ACTIVE != 0
    }
    pub fn timed_out(&self) -> bool {
        self.flags & SERVO_FLAG_TIMED_OUT != 0
    }
    pub fn limit_up(&self) -> bool {
        self.flags & SERVO_FLAG_LIMIT_UP != 0
    }
    pub fn limit_down(&self) -> bool {
        self.flags & SERVO_FLAG_LIMIT_DOWN != 0
    }
}

/// 0x82: BLD120A blade motor, every 50 ms. `flags` share the 0x81 layout;
/// the motor is one-directional, negative commands read as 0.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LawerMotorStatus {
    pub commanded_permille: i16,
    pub applied_pwm: i16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
}

/// All decoders return `None` when the payload length does not match, which
/// is the C++ `return false` path; the caller then drops the frame.
pub fn decode_wheel_feedback(seq: u8, p: &[u8]) -> Option<WheelFeedback> {
    if p.len() != 24 {
        return None;
    }
    Some(WheelFeedback {
        left_target_rpm: get_i16(&p[0..]) as f64 / 100.0,
        left_measured_rpm: get_i16(&p[2..]) as f64 / 100.0,
        right_target_rpm: get_i16(&p[4..]) as f64 / 100.0,
        right_measured_rpm: get_i16(&p[6..]) as f64 / 100.0,
        left_pid_output: get_i16(&p[8..]),
        right_pid_output: get_i16(&p[10..]),
        left_total_counts: get_i32(&p[12..]),
        right_total_counts: get_i32(&p[16..]),
        flags: p[20],
        seq,
    })
}

pub fn decode_motor_status(p: &[u8]) -> Option<MotorStatus> {
    if p.len() != 12 {
        return None;
    }
    Some(MotorStatus {
        commanded_left_permille: get_i16(&p[0..]),
        commanded_right_permille: get_i16(&p[2..]),
        applied_left_pwm: get_i16(&p[4..]),
        applied_right_pwm: get_i16(&p[6..]),
        command_age_ms: get_u16(&p[8..]),
        flags: p[10],
        last_rx_seq: p[11],
    })
}

pub fn decode_power_status(p: &[u8]) -> Option<PowerStatus> {
    if p.len() != 8 {
        return None;
    }
    Some(PowerStatus {
        state: p[0],
        flags: p[1],
        shutdown_reason: p[2],
        last_rx_seq: p[3],
        press_ms: get_u16(&p[4..]),
        shutdown_elapsed_ms: get_u16(&p[6..]),
    })
}

pub fn decode_ws2812_status(p: &[u8]) -> Option<Ws2812Status> {
    if p.len() != 8 {
        return None;
    }
    Some(Ws2812Status {
        mode: p[0],
        r: p[1],
        g: p[2],
        b: p[3],
        effect_period_ms: get_u16(&p[4..]),
        flags: p[6],
        last_rx_seq: p[7],
    })
}

pub fn decode_pid_config_status(p: &[u8]) -> Option<PidConfigStatus> {
    if p.len() != 28 {
        return None;
    }
    // IEEE-754 little-endian floats, same layout as the STM32 writes them
    Some(PidConfigStatus {
        left_kp: get_f32(&p[0..]),
        left_ki: get_f32(&p[4..]),
        left_kd: get_f32(&p[8..]),
        right_kp: get_f32(&p[12..]),
        right_ki: get_f32(&p[16..]),
        right_kd: get_f32(&p[20..]),
        flags: p[24],
        last_rx_seq: p[25],
        flash_diag: get_u16(&p[26..]),
    })
}

pub fn decode_firmware_info(p: &[u8]) -> Option<FirmwareInfo> {
    if p.len() != 16 {
        return None;
    }
    Some(FirmwareInfo {
        major: p[0],
        minor: p[1],
        patch: p[2],
        protocol_version: p[3],
        git_sha32: get_u32(&p[4..]),
        build_unix: get_u32(&p[8..]),
        build_flags: p[12],
    })
}

pub fn decode_charger_status(p: &[u8]) -> Option<ChargerStatus> {
    if p.len() != 16 {
        return None;
    }
    Some(ChargerStatus {
        voltage_cv: get_u16(&p[0..]),
        current_ca: get_u16(&p[2..]),
        temp_c: get_u16(&p[4..]),
        reg3: get_u16(&p[6..]),
        reg4: get_u16(&p[8..]),
        flags: p[10],
        comm_error_count: p[11],
        age_ms: get_u16(&p[12..]),
        last_exception_code: p[14],
    })
}

pub fn decode_servo_status(p: &[u8]) -> Option<ServoStatus> {
    if p.len() != 8 {
        return None;
    }
    Some(ServoStatus {
        pulse_us: get_u16(&p[0..]),
        hold_timeout_ms: get_u16(&p[2..]),
        command_age_ms: get_u16(&p[4..]),
        flags: p[6],
        last_rx_seq: p[7],
    })
}

pub fn decode_lawer_motor_status(p: &[u8]) -> Option<LawerMotorStatus> {
    if p.len() != 8 {
        return None;
    }
    Some(LawerMotorStatus {
        commanded_permille: get_i16(&p[0..]),
        applied_pwm: get_i16(&p[2..]),
        command_age_ms: get_u16(&p[4..]),
        flags: p[6],
        last_rx_seq: p[7],
    })
}

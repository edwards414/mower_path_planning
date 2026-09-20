//! UART frame protocol (see `UART_OPEN_LOOP_PROTOCOL.md`).
//!
//! ```text
//! A5 5A | version | type | seq | len | payload[len] | crc_lo | crc_hi
//!         \__________ CRC-16/CCITT-FALSE covers this __________/
//! ```
//!
//! Compared with `uart_interface.cpp` the safety differences are:
//!
//! * payload structs are decoded with `zerocopy`, so a length mismatch is a
//!   `None`, never a short `memcpy`;
//! * the parser's payload buffer is only ever indexed through a
//!   `heapless::Vec`, so a state-machine bug cannot write past it;
//! * frames are handed to the caller as an enum ([`Frame`]) — adding a new
//!   frame type without handling it is a compile error, not a silent drop.

use heapless::Vec;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout};

pub const SOF_0: u8 = 0xA5;
pub const SOF_1: u8 = 0x5A;
pub const VERSION: u8 = 0x01;
pub const MAX_PAYLOAD: usize = 32;
/// SOF(2) + version + type + seq + len + crc(2)
pub const FRAME_OVERHEAD: usize = 8;
pub const MAX_FRAME: usize = FRAME_OVERHEAD + MAX_PAYLOAD;
pub const STATUS_PERIOD_MS: u64 = 50;

pub mod frame_type {
    pub const MOTOR_OPEN_LOOP_COMMAND: u8 = 0x01;
    pub const LAWER_MOTOR_COMMAND: u8 = 0x02;
    pub const WS2812_COMMAND: u8 = 0x03;
    pub const PID_CONFIG_COMMAND: u8 = 0x04;
    pub const POWER_COMMAND: u8 = 0x05;
    /// 0x06 is INFO_REQUEST in the mower_path_planning/firmware build.
    pub const SERVO_COMMAND: u8 = 0x07;
    pub const ENTER_BOOTLOADER: u8 = 0x0F;
    pub const MOTOR_STATUS: u8 = 0x81;
    pub const LAWER_MOTOR_STATUS: u8 = 0x82;
    pub const WS2812_STATUS: u8 = 0x83;
    pub const PID_CONFIG_STATUS: u8 = 0x84;
    pub const WHEEL_FEEDBACK_STATUS: u8 = 0x85;
    pub const POWER_STATUS: u8 = 0x86;
    /// 0x87 is FIRMWARE_INFO in the mower_path_planning/firmware build.
    pub const SERVO_STATUS: u8 = 0x88;
    pub const CHARGER_STATUS: u8 = 0x89;
    /// 0x8A (analog status) is retired: the ADC mux was never fitted. Do not reuse.
    pub const ENTER_BOOTLOADER_ACK: u8 = 0x8F;
}

pub mod motor_status_flag {
    pub const COMMAND_VALID: u8 = 0x01;
    pub const COMMAND_TIMEOUT: u8 = 0x02;
    pub const DRIVER_ALARM: u8 = 0x04;
}

pub mod pid_status_flag {
    pub const CLOSED_LOOP_ENABLED: u8 = 0x01;
    pub const FLASH_VALID: u8 = 0x02;
    pub const LAST_SAVE_OK: u8 = 0x04;
    pub const LAST_APPLY_OK: u8 = 0x08;
}

pub mod power_action {
    pub const NONE: u8 = 0x00;
    pub const HOST_SHUTDOWN_ACK: u8 = 0x01;
    pub const REQUEST_SHUTDOWN: u8 = 0x02;
    pub const CANCEL_SHUTDOWN: u8 = 0x03;
    pub const FORCE_POWER_OFF: u8 = 0x04;
}

pub mod servo_status_flag {
    pub const ENABLED: u8 = 0x01;
    /// The last target was clamped by a limit switch; the next 0x07 clears it.
    pub const LIMIT_ACTIVE: u8 = 0x02;
    pub const OUTPUT_ACTIVE: u8 = 0x04;
    pub const TIMED_OUT: u8 = 0x08;
    /// UP switch pressed (debounced).
    pub const LIMIT_UP: u8 = 0x10;
    /// DOWN switch pressed (debounced).
    pub const LIMIT_DN: u8 = 0x20;
}

pub mod charger_status_flag {
    pub const ONLINE: u8 = 0x01;
    /// |current| above threshold: flow, not direction (meter in the pack lead)
    pub const CURRENT_PRESENT: u8 = 0x02;
    /// Was CV_PHASE for the CC/CV module; the V/A/temp meter never sets it.
    pub const RESERVED_CV_PHASE: u8 = 0x04;
    pub const INPUT_PRESENT: u8 = 0x08;
    pub const EVER_SEEN: u8 = 0x10;
}

pub const BOOT_REQUEST_MAGIC: u32 = 0xB007_B007;

// ---------------------------------------------------------------------------
// Payloads. Field order and packing match the C++ typedefs byte for byte;
// the `const _` asserts pin the sizes so a refactor cannot silently change
// the wire format.
// ---------------------------------------------------------------------------

macro_rules! wire_struct {
    ($(#[$m:meta])* $name:ident, $size:expr, { $($(#[$fm:meta])* pub $field:ident : $ty:ty),* $(,)? }) => {
        $(#[$m])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, FromBytes, IntoBytes, Immutable, KnownLayout)]
        #[repr(C, packed)]
        pub struct $name { $($(#[$fm])* pub $field: $ty),* }
        const _: () = assert!(core::mem::size_of::<$name>() == $size);
    };
}

wire_struct!(MotorOpenLoopCommand, 8, {
    pub left_command_permille: i16,
    pub right_command_permille: i16,
    pub command_timeout_ms: u16,
    pub reserved: u16,
});

wire_struct!(MotorStatus, 12, {
    pub commanded_left_permille: i16,
    pub commanded_right_permille: i16,
    pub applied_left_pwm: i16,
    pub applied_right_pwm: i16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
});

wire_struct!(LawerMotorCommand, 8, {
    pub command_permille: i16,
    pub command_timeout_ms: u16,
    pub reserved0: u16,
    pub reserved1: u16,
});

wire_struct!(LawerMotorStatus, 8, {
    pub commanded_permille: i16,
    pub applied_pwm: i16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
});

wire_struct!(Ws2812Command, 8, {
    pub mode: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub effect_period_ms: u16,
    pub reserved0: u8,
    pub reserved1: u8,
});

wire_struct!(Ws2812Status, 8, {
    pub mode: u8,
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub effect_period_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
});

wire_struct!(PidConfigCommand, 28, {
    pub left_kp: f32,
    pub left_ki: f32,
    pub left_kd: f32,
    pub right_kp: f32,
    pub right_ki: f32,
    pub right_kd: f32,
    pub persist_to_flash: u8,
    pub closed_loop_enabled: u8,
    pub reserved: u16,
});

wire_struct!(PidConfigStatus, 28, {
    pub left_kp: f32,
    pub left_ki: f32,
    pub left_kd: f32,
    pub right_kp: f32,
    pub right_ki: f32,
    pub right_kd: f32,
    pub flags: u8,
    pub last_rx_seq: u8,
    pub reserved: u16,
});

wire_struct!(WheelFeedbackStatus, 24, {
    pub left_target_rpm_x100: i16,
    pub left_measured_rpm_x100: i16,
    pub right_target_rpm_x100: i16,
    pub right_measured_rpm_x100: i16,
    pub left_pid_output: i16,
    pub right_pid_output: i16,
    /// accumulated encoder counts since boot
    pub left_total_counts: i32,
    pub right_total_counts: i32,
    pub flags: u8,
    pub reserved0: u8,
    pub reserved1: u8,
    pub reserved2: u8,
});

wire_struct!(PowerCommand, 4, {
    pub action: u8,
    pub reserved0: u8,
    pub reserved1: u16,
});

wire_struct!(PowerStatus, 8, {
    pub state: u8,
    pub flags: u8,
    pub shutdown_reason: u8,
    pub last_rx_seq: u8,
    pub press_ms: u16,
    pub shutdown_elapsed_ms: u16,
});

wire_struct!(ServoCommand, 8, {
    /// 500-2500 µs; 0 = stop pulses (servo goes limp)
    pub pulse_us: u16,
    /// 0 = hold until the next command
    pub hold_timeout_ms: u16,
    pub reserved0: u16,
    pub reserved1: u16,
});

wire_struct!(ServoStatus, 8, {
    pub pulse_us: u16,
    pub hold_timeout_ms: u16,
    pub command_age_ms: u16,
    pub flags: u8,
    pub last_rx_seq: u8,
});

wire_struct!(ChargerStatus, 16, {
    /// reg 0: charge line / battery voltage, x0.01 V
    pub voltage_cv: u16,
    /// reg 1: charge current, x0.01 A
    pub current_ca: u16,
    /// reg 2: meter temperature, degC
    pub temp_c: u16,
    /// reg 3 raw, meaning unknown
    pub reg3: u16,
    /// reg 4 raw, meaning unknown
    pub reg4: u16,
    pub flags: u8,
    /// wraps; timeouts + bad frames since boot
    pub comm_error_count: u8,
    /// since the last valid reply, 0xFFFF = never
    pub age_ms: u16,
    pub last_exception_code: u8,
    pub reserved: u8,
});

wire_struct!(BootEnter, 8, {
    pub magic: u32,
    pub reserved: u32,
});

wire_struct!(BootEnterAck, 4, {
    pub status: u8,
    pub reserved: [u8; 3],
});

// ---------------------------------------------------------------------------
// Decoded frames
// ---------------------------------------------------------------------------

/// A host → MCU frame whose CRC checked out and whose payload had exactly
/// the expected length. Anything else never reaches the caller.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Frame {
    MotorOpenLoop(MotorOpenLoopCommand),
    LawerMotor(LawerMotorCommand),
    Ws2812(Ws2812Command),
    PidConfig(PidConfigCommand),
    Power(PowerCommand),
    Servo(ServoCommand),
    EnterBootloader(BootEnter),
}

impl Frame {
    /// Decode a payload for `frame_type`. `None` for unknown types, wrong
    /// length or a bootloader request with the wrong magic.
    pub fn decode(frame_type: u8, payload: &[u8]) -> Option<Self> {
        use self::frame_type as t;
        // `read_from_bytes` requires an exact length match, which is the
        // `payload_len != sizeof(...)` check from the C++ handler.
        Some(match frame_type {
            t::MOTOR_OPEN_LOOP_COMMAND => Self::MotorOpenLoop(MotorOpenLoopCommand::read_from_bytes(payload).ok()?),
            t::LAWER_MOTOR_COMMAND => Self::LawerMotor(LawerMotorCommand::read_from_bytes(payload).ok()?),
            t::WS2812_COMMAND => Self::Ws2812(Ws2812Command::read_from_bytes(payload).ok()?),
            t::PID_CONFIG_COMMAND => Self::PidConfig(PidConfigCommand::read_from_bytes(payload).ok()?),
            t::POWER_COMMAND => Self::Power(PowerCommand::read_from_bytes(payload).ok()?),
            t::SERVO_COMMAND => Self::Servo(ServoCommand::read_from_bytes(payload).ok()?),
            t::ENTER_BOOTLOADER => {
                let boot = BootEnter::read_from_bytes(payload).ok()?;
                if boot.magic != BOOT_REQUEST_MAGIC {
                    return None;
                }
                Self::EnterBootloader(boot)
            }
            _ => return None,
        })
    }
}

/// A frame plus its sequence number, as delivered by the parser.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Received {
    pub seq: u8,
    pub frame: Frame,
}

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

/// A complete, CRC'd frame ready to hand to the UART.
pub type FrameBytes = Vec<u8, MAX_FRAME>;

/// Build a frame. Returns `None` only if `payload` exceeds [`MAX_PAYLOAD`].
pub fn encode(frame_type: u8, seq: u8, payload: &[u8]) -> Option<FrameBytes> {
    if payload.len() > MAX_PAYLOAD {
        return None;
    }
    let mut buf = FrameBytes::new();
    // Every push below is bounded by MAX_FRAME, so these cannot fail.
    buf.extend_from_slice(&[SOF_0, SOF_1, VERSION, frame_type, seq, payload.len() as u8]).ok()?;
    buf.extend_from_slice(payload).ok()?;
    let crc = crc16_ccitt(&buf[2..]);
    buf.extend_from_slice(&crc.to_le_bytes()).ok()?;
    Some(buf)
}

/// Encode a typed status payload.
pub fn encode_payload<T: IntoBytes + Immutable>(frame_type: u8, seq: u8, payload: &T) -> Option<FrameBytes> {
    encode(frame_type, seq, payload.as_bytes())
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    WaitSof0,
    WaitSof1,
    Version,
    Type,
    Seq,
    Len,
    Payload,
    CrcLow,
    CrcHigh,
}

/// Byte-at-a-time frame parser, resynchronising on `A5 5A`.
pub struct Parser {
    state: State,
    version: u8,
    frame_type: u8,
    seq: u8,
    payload_len: u8,
    payload: Vec<u8, MAX_PAYLOAD>,
    crc_low: u8,
}

impl Default for Parser {
    fn default() -> Self {
        Self::new()
    }
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            state: State::WaitSof0,
            version: 0,
            frame_type: 0,
            seq: 0,
            payload_len: 0,
            payload: Vec::new(),
            crc_low: 0,
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one byte. Returns a frame when a complete, CRC-valid, known frame
    /// has been assembled.
    pub fn push(&mut self, byte: u8) -> Option<Received> {
        match self.state {
            State::WaitSof0 => {
                if byte == SOF_0 {
                    self.state = State::WaitSof1;
                }
            }
            State::WaitSof1 => {
                self.state = match byte {
                    SOF_1 => State::Version,
                    SOF_0 => State::WaitSof1,
                    _ => State::WaitSof0,
                };
            }
            State::Version => {
                self.version = byte;
                self.state = State::Type;
            }
            State::Type => {
                self.frame_type = byte;
                self.state = State::Seq;
            }
            State::Seq => {
                self.seq = byte;
                self.state = State::Len;
            }
            State::Len => {
                if usize::from(byte) > MAX_PAYLOAD {
                    self.reset();
                    return None;
                }
                self.payload_len = byte;
                self.payload.clear();
                self.state = if byte == 0 { State::CrcLow } else { State::Payload };
            }
            State::Payload => {
                // Capacity is MAX_PAYLOAD and payload_len <= MAX_PAYLOAD, so
                // this cannot fail; if it ever did we drop the frame rather
                // than corrupt memory.
                if self.payload.push(byte).is_err() {
                    self.reset();
                    return None;
                }
                if self.payload.len() >= usize::from(self.payload_len) {
                    self.state = State::CrcLow;
                }
            }
            State::CrcLow => {
                self.crc_low = byte;
                self.state = State::CrcHigh;
            }
            State::CrcHigh => {
                let result = self.finish(byte);
                self.reset();
                return result;
            }
        }
        None
    }

    fn finish(&self, crc_high: u8) -> Option<Received> {
        if self.version != VERSION {
            return None;
        }
        let mut crc_input: Vec<u8, { 4 + MAX_PAYLOAD }> = Vec::new();
        crc_input.extend_from_slice(&[self.version, self.frame_type, self.seq, self.payload_len]).ok()?;
        crc_input.extend_from_slice(&self.payload).ok()?;
        let received = u16::from_le_bytes([self.crc_low, crc_high]);
        if crc16_ccitt(&crc_input) != received {
            return None;
        }
        Frame::decode(self.frame_type, &self.payload).map(|frame| Received { seq: self.seq, frame })
    }
}

/// Convert an f32 to the `x100` fixed-point wire representation, saturating.
pub fn f32_to_i16_x100(value: f32) -> i16 {
    // `as` saturates and maps NaN to 0, which is what the C++ clamp did minus
    // the NaN case (there it was UB).
    (value * 100.0) as i16
}

/// Saturating f32 → i16 for PWM outputs.
pub fn f32_to_i16(value: f32) -> i16 {
    value as i16
}

/// Saturating u32 → u16.
pub fn saturating_u16(value: u32) -> u16 {
    value.min(u32::from(u16::MAX)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(parser: &mut Parser, bytes: &[u8]) -> Option<Received> {
        let mut out = None;
        for &b in bytes {
            if let Some(f) = parser.push(b) {
                assert!(out.is_none(), "one frame expected");
                out = Some(f);
            }
        }
        out
    }

    #[test]
    fn crc_matches_reference_vector() {
        // CRC-16/CCITT-FALSE check value for "123456789"
        assert_eq!(crc16_ccitt(b"123456789"), 0x29B1);
    }

    #[test]
    fn round_trip_motor_command() {
        let cmd = MotorOpenLoopCommand {
            left_command_permille: 250,
            right_command_permille: -250,
            command_timeout_ms: 300,
            reserved: 0,
        };
        let bytes = encode_payload(frame_type::MOTOR_OPEN_LOOP_COMMAND, 7, &cmd).unwrap();
        assert_eq!(bytes.len(), FRAME_OVERHEAD + 8);
        assert_eq!(&bytes[..2], &[SOF_0, SOF_1]);

        let mut p = Parser::new();
        let rx = feed(&mut p, &bytes).expect("frame");
        assert_eq!(rx.seq, 7);
        assert_eq!(rx.frame, Frame::MotorOpenLoop(cmd));
    }

    #[test]
    fn corrupted_crc_is_dropped() {
        let cmd = LawerMotorCommand { command_permille: 500, command_timeout_ms: 0, reserved0: 0, reserved1: 0 };
        let mut bytes = encode_payload(frame_type::LAWER_MOTOR_COMMAND, 1, &cmd).unwrap();
        let last = bytes.len() - 1;
        bytes[last] ^= 0x01;
        assert!(feed(&mut Parser::new(), &bytes).is_none());
    }

    #[test]
    fn wrong_payload_length_is_dropped_even_with_valid_crc() {
        // A CRC-valid frame whose payload is one byte short for its type.
        let bytes = encode(frame_type::MOTOR_OPEN_LOOP_COMMAND, 3, &[0u8; 7]).unwrap();
        assert!(feed(&mut Parser::new(), &bytes).is_none());
    }

    #[test]
    fn oversized_length_byte_resets_parser() {
        let mut p = Parser::new();
        assert!(feed(&mut p, &[SOF_0, SOF_1, VERSION, 0x01, 0, (MAX_PAYLOAD + 1) as u8]).is_none());
        // parser must be back in sync for the next good frame
        let cmd = PowerCommand { action: power_action::CANCEL_SHUTDOWN, reserved0: 0, reserved1: 0 };
        let bytes = encode_payload(frame_type::POWER_COMMAND, 9, &cmd).unwrap();
        assert_eq!(feed(&mut p, &bytes).unwrap().frame, Frame::Power(cmd));
    }

    #[test]
    fn resyncs_after_garbage_and_repeated_sof0() {
        let cmd = Ws2812Command { mode: 2, r: 1, g: 2, b: 3, effect_period_ms: 100, reserved0: 0, reserved1: 0 };
        let frame = encode_payload(frame_type::WS2812_COMMAND, 4, &cmd).unwrap();
        let mut stream: std::vec::Vec<u8> = vec![0x00, 0xFF, SOF_0, SOF_0, 0x12];
        stream.extend_from_slice(&frame);
        let rx = feed(&mut Parser::new(), &stream).expect("frame after garbage");
        assert_eq!(rx.frame, Frame::Ws2812(cmd));
    }

    #[test]
    fn bootloader_request_needs_magic() {
        let bad = BootEnter { magic: 0xDEAD_BEEF, reserved: 0 };
        let bytes = encode_payload(frame_type::ENTER_BOOTLOADER, 0, &bad).unwrap();
        assert!(feed(&mut Parser::new(), &bytes).is_none());

        let good = BootEnter { magic: BOOT_REQUEST_MAGIC, reserved: 0 };
        let bytes = encode_payload(frame_type::ENTER_BOOTLOADER, 0, &good).unwrap();
        assert_eq!(feed(&mut Parser::new(), &bytes).unwrap().frame, Frame::EnterBootloader(good));
    }

    #[test]
    fn unknown_type_and_wrong_version_are_dropped() {
        assert!(feed(&mut Parser::new(), &encode(0x7E, 0, &[]).unwrap()).is_none());
        let mut bytes = encode(frame_type::POWER_COMMAND, 0, &[0u8; 4]).unwrap();
        bytes[2] = 0x02; // version — CRC now mismatches too, but the version check is first
        assert!(feed(&mut Parser::new(), &bytes).is_none());
    }

    #[test]
    fn encode_rejects_oversized_payload() {
        assert!(encode(0x81, 0, &[0u8; MAX_PAYLOAD + 1]).is_none());
        assert!(encode(0x81, 0, &[0u8; MAX_PAYLOAD]).is_some());
    }

    #[test]
    fn pid_config_layout_matches_c_struct() {
        // C++: six floats, two u8, one u16 => 28 bytes, u16 at offset 26.
        let cmd = PidConfigCommand {
            left_kp: 1.0,
            left_ki: 2.0,
            left_kd: 3.0,
            right_kp: 4.0,
            right_ki: 5.0,
            right_kd: 6.0,
            persist_to_flash: 1,
            closed_loop_enabled: 0,
            reserved: 0xBEEF,
        };
        let b = cmd.as_bytes();
        assert_eq!(&b[0..4], &1.0f32.to_le_bytes());
        assert_eq!(b[24], 1);
        assert_eq!(b[25], 0);
        assert_eq!(&b[26..28], &0xBEEFu16.to_le_bytes());
    }

    #[test]
    fn servo_command_round_trip_and_charger_layout() {
        let cmd = ServoCommand { pulse_us: 1500, hold_timeout_ms: 0, reserved0: 0, reserved1: 0 };
        let bytes = encode_payload(frame_type::SERVO_COMMAND, 5, &cmd).unwrap();
        let rx = feed(&mut Parser::new(), &bytes).expect("frame");
        assert_eq!(rx.frame, Frame::Servo(cmd));

        // C++: five u16, u8 flags, u8 errors, u16 age, u8 exception, u8 reserved => 16 bytes
        let st = ChargerStatus {
            voltage_cv: 2563,
            current_ca: 0,
            temp_c: 35,
            reg3: 11,
            reg4: 48961,
            flags: 0x19,
            comm_error_count: 3,
            age_ms: 120,
            last_exception_code: 0,
            reserved: 0,
        };
        let b = st.as_bytes();
        assert_eq!(&b[0..2], &2563u16.to_le_bytes());
        assert_eq!(b[10], 0x19);
        assert_eq!(b[11], 3);
        assert_eq!(&b[12..14], &120u16.to_le_bytes());
    }

    #[test]
    fn fixed_point_conversions_saturate() {
        assert_eq!(f32_to_i16_x100(12.345), 1234);
        assert_eq!(f32_to_i16_x100(1e9), i16::MAX);
        assert_eq!(f32_to_i16_x100(-1e9), i16::MIN);
        assert_eq!(f32_to_i16_x100(f32::NAN), 0);
        assert_eq!(saturating_u16(70_000), u16::MAX);
    }
}

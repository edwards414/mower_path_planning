//! WIT motion sensor serial protocol: 11-byte frames `55 <type> d0..d7 <sum>`
//! where `sum` is the low byte of the sum of the first ten bytes, and the
//! eight data bytes are four little-endian int16 registers. The parser is a
//! byte-for-byte port of `wit_ros2_imu.handle_serial_data`: it waits for a
//! 0x55 at frame position 0, collects eleven bytes, checks the type and the
//! checksum, then starts over (no rescan of the discarded bytes).

pub const FRAME_LEN: usize = 11;

/// Frame types the driver understands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Frame {
    /// 0x51 acceleration, m/s^2 (register / 32768 * 16 g, g = 9.8 as upstream)
    Acceleration,
    /// 0x52 angular velocity, rad/s (register / 32768 * 2000 deg/s)
    AngularVelocity,
    /// 0x53 orientation, degrees (register / 32768 * 180)
    Angle,
    /// 0x54 magnetometer, raw registers
    Magnetometer,
}

#[derive(Debug, Default)]
pub struct Parser {
    buf: [u8; FRAME_LEN],
    len: usize,
    pub acceleration: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub angle_degree: [f64; 3],
    pub magnetometer: [i16; 3],
    pub checksum_failures: u64,
}

fn regs(data: &[u8]) -> [i16; 4] {
    let mut out = [0i16; 4];
    for (i, r) in out.iter_mut().enumerate() {
        *r = i16::from_le_bytes([data[2 + 2 * i], data[3 + 2 * i]]);
    }
    out
}

impl Parser {
    /// Discard every partial frame (after a host-side scheduling gap).
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Feed one byte; returns the frame type when a valid frame completed.
    pub fn push(&mut self, byte: u8) -> Option<Frame> {
        self.buf[self.len] = byte;
        self.len += 1;
        if self.buf[0] != 0x55 {
            self.len = 0;
            return None;
        }
        if self.len < FRAME_LEN {
            return None;
        }
        let frame = &self.buf;
        let checksum_ok = frame[..10].iter().map(|b| *b as u32).sum::<u32>() & 0xff == frame[10] as u32;
        let result = match frame[1] {
            0x51 | 0x52 | 0x53 | 0x54 if !checksum_ok => {
                self.checksum_failures += 1;
                None
            }
            0x51 => {
                let r = regs(frame);
                for i in 0..3 {
                    self.acceleration[i] = r[i] as f64 / 32768.0 * 16.0 * 9.8;
                }
                Some(Frame::Acceleration)
            }
            0x52 => {
                let r = regs(frame);
                for i in 0..3 {
                    self.angular_velocity[i] = r[i] as f64 / 32768.0 * 2000.0 * std::f64::consts::PI / 180.0;
                }
                Some(Frame::AngularVelocity)
            }
            0x53 => {
                let r = regs(frame);
                for i in 0..3 {
                    self.angle_degree[i] = r[i] as f64 / 32768.0 * 180.0;
                }
                Some(Frame::Angle)
            }
            0x54 => {
                let r = regs(frame);
                self.magnetometer = [r[0], r[1], r[2]];
                Some(Frame::Magnetometer)
            }
            _ => None,
        };
        self.len = 0;
        result
    }
}

/// Euler angles (radians) -> quaternion [x, y, z, w], the formula the Python
/// driver used (`get_quaternion_from_euler`).
pub fn quaternion_from_euler(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(kind: u8, regs: [i16; 4]) -> Vec<u8> {
        let mut f = vec![0x55, kind];
        for r in regs {
            f.extend_from_slice(&r.to_le_bytes());
        }
        let sum = f.iter().map(|b| *b as u32).sum::<u32>() & 0xff;
        f.push(sum as u8);
        f
    }

    #[test]
    fn frames_decode_with_the_upstream_scales() {
        let mut p = Parser::default();
        let mut out = vec![];
        for b in frame(0x51, [2048, -2048, 32767, 0]) {
            out.push(p.push(b));
        }
        assert_eq!(out.last().unwrap(), &Some(Frame::Acceleration));
        assert!(out[..10].iter().all(|o| o.is_none()));
        assert!((p.acceleration[0] - 2048.0 / 32768.0 * 16.0 * 9.8).abs() < 1e-12);
        assert!((p.acceleration[1] + 2048.0 / 32768.0 * 16.0 * 9.8).abs() < 1e-12);
        for b in frame(0x52, [16384, 0, -16384, 0]) {
            p.push(b);
        }
        assert!((p.angular_velocity[0] - 1000.0f64.to_radians()).abs() < 1e-9);
        assert!((p.angular_velocity[2] + 1000.0f64.to_radians()).abs() < 1e-9);
        let mut last = None;
        for b in frame(0x53, [0, 0, 16384, 0]) {
            last = p.push(b);
        }
        assert_eq!(last, Some(Frame::Angle));
        assert!((p.angle_degree[2] - 90.0).abs() < 1e-9);
        for b in frame(0x54, [1, -2, 3, 0]) {
            last = p.push(b);
        }
        assert_eq!(last, Some(Frame::Magnetometer));
        assert_eq!(p.magnetometer, [1, -2, 3]);
        assert_eq!(p.checksum_failures, 0);
    }

    #[test]
    fn bad_checksum_and_garbage_are_dropped_and_sync_recovers() {
        let mut p = Parser::default();
        let mut bad = frame(0x51, [1, 2, 3, 4]);
        bad[10] ^= 0x01;
        let mut results = vec![];
        for b in bad.iter().chain([0x00u8, 0x13].iter()).chain(frame(0x53, [0, 0, 0, 0]).iter()) {
            results.push(p.push(*b));
        }
        assert_eq!(p.checksum_failures, 1);
        assert_eq!(results.iter().filter(|r| r.is_some()).count(), 1);
        assert_eq!(results.last().unwrap(), &Some(Frame::Angle));
        // unknown frame type resets the parser without a result
        let mut last = Some(Frame::Angle);
        for b in frame(0x59, [0, 0, 0, 0]) {
            last = p.push(b);
        }
        assert_eq!(last, None);
        p.reset();
        assert_eq!(p.len, 0);
    }

    #[test]
    fn quaternion_matches_the_python_formula() {
        let q = quaternion_from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        assert!((q[2] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        assert!((q[3] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        let q = quaternion_from_euler(0.3, -0.2, 1.1);
        let norm: f64 = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12);
        // reference values from the numpy implementation
        assert!((q[0] - 0.178359).abs() < 1e-5, "{q:?}");
        assert!((q[1] - (-0.006436)).abs() < 1e-5, "{q:?}");
        assert!((q[2] - 0.526955).abs() < 1e-5, "{q:?}");
        assert!((q[3] - 0.830942).abs() < 1e-5, "{q:?}");
    }
}

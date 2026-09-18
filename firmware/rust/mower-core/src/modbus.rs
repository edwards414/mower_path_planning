//! Minimal Modbus RTU master codec (mirrors `Module/Src/modbus_rtu.cpp`).
//!
//! Only what the 数控 30V5A charger needs: CRC-16, an FC03 read request,
//! an FC16 write request for a later CC/CV set command, and a reply scanner
//! that tolerates an echoed request in front of the reply (half-duplex
//! transceivers with RE grounded loop TX back into RX).

use heapless::Vec;

pub const FC_READ_HOLDING: u8 = 0x03;
pub const FC_WRITE_MULTIPLE: u8 = 0x10;
pub const EXCEPTION_BIT: u8 = 0x80;

/// addr + fc + start(2) + count(2) + crc(2)
pub const READ_REQUEST_LEN: usize = 8;
/// Longest write this codec will build: 7 header bytes + 16 registers + CRC.
pub const MAX_WRITE_REGS: usize = 16;
pub const MAX_WRITE_REQUEST_LEN: usize = 7 + MAX_WRITE_REGS * 2 + 2;

/// Standard Modbus CRC-16 (poly 0xA001 reflected, init 0xFFFF). The low
/// byte goes on the wire first.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= u16::from(byte);
        for _ in 0..8 {
            crc = if crc & 0x0001 != 0 { (crc >> 1) ^ 0xA001 } else { crc >> 1 };
        }
    }
    crc
}

fn crc_matches(frame: &[u8]) -> bool {
    let (body, wire) = frame.split_at(frame.len() - 2);
    crc16(body) == u16::from_le_bytes([wire[0], wire[1]])
}

/// `addr 03 start(2) count(2) crc(2)`.
pub fn build_read_holding(addr: u8, start_reg: u16, reg_count: u16) -> [u8; READ_REQUEST_LEN] {
    let mut out = [0u8; READ_REQUEST_LEN];
    out[0] = addr;
    out[1] = FC_READ_HOLDING;
    out[2..4].copy_from_slice(&start_reg.to_be_bytes());
    out[4..6].copy_from_slice(&reg_count.to_be_bytes());
    let crc = crc16(&out[..6]);
    out[6..8].copy_from_slice(&crc.to_le_bytes());
    out
}

/// `addr 10 start(2) count(2) bytes(1) data... crc(2)`. `None` for an
/// empty or oversized register list.
pub fn build_write_multiple(addr: u8, start_reg: u16, regs: &[u16]) -> Option<Vec<u8, MAX_WRITE_REQUEST_LEN>> {
    if regs.is_empty() || regs.len() > MAX_WRITE_REGS {
        return None;
    }
    let mut out = Vec::new();
    // Bounded by MAX_WRITE_REQUEST_LEN, so none of these pushes can fail.
    out.extend_from_slice(&[addr, FC_WRITE_MULTIPLE]).ok()?;
    out.extend_from_slice(&start_reg.to_be_bytes()).ok()?;
    out.extend_from_slice(&(regs.len() as u16).to_be_bytes()).ok()?;
    out.push((regs.len() * 2) as u8).ok()?;
    for reg in regs {
        out.extend_from_slice(&reg.to_be_bytes()).ok()?;
    }
    let crc = crc16(&out);
    out.extend_from_slice(&crc.to_le_bytes()).ok()?;
    Some(out)
}

/// Result of scanning a receive buffer for an FC03 reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReadReply {
    /// No complete, CRC-valid frame for this address in the buffer (yet).
    None,
    /// `regs[..count]` were filled; the frame started at `offset`.
    Ok { count: usize, offset: usize },
    /// The device answered `FC | 0x80`.
    Exception { code: u8, offset: usize },
}

/// Scan `buf` for an FC03 reply from `addr` with a valid CRC, starting at
/// every offset so an echoed request or leading noise is skipped. At most
/// `regs.len()` registers are stored (big-endian on the wire).
pub fn find_read_reply(buf: &[u8], addr: u8, regs: &mut [u16]) -> ReadReply {
    if buf.len() < 5 {
        return ReadReply::None;
    }
    for offset in 0..=buf.len() - 5 {
        let f = &buf[offset..];
        if f[0] != addr {
            continue;
        }
        if f[1] == FC_READ_HOLDING | EXCEPTION_BIT {
            // addr, fc|0x80, exception, crc(2)
            if crc_matches(&f[..5]) {
                return ReadReply::Exception { code: f[2], offset };
            }
            continue;
        }
        if f[1] != FC_READ_HOLDING {
            continue;
        }
        // addr, fc, byte_count, data..., crc(2)
        let byte_count = usize::from(f[2]);
        let frame_len = 3 + byte_count + 2;
        if byte_count == 0 || byte_count % 2 != 0 || f.len() < frame_len {
            continue;
        }
        if !crc_matches(&f[..frame_len]) {
            continue;
        }
        let count = byte_count / 2;
        for (i, reg) in regs.iter_mut().take(count).enumerate() {
            *reg = u16::from_be_bytes([f[3 + i * 2], f[4 + i * 2]]);
        }
        return ReadReply::Ok { count, offset };
    }
    ReadReply::None
}

#[cfg(test)]
mod tests {
    use super::*;

    // Frames from the vendor document (数控32V5A协议.doc).
    const REQ: [u8; 8] = [0x01, 0x03, 0x00, 0x00, 0x00, 0x05, 0x85, 0xC9];
    const REPLY: [u8; 15] = [0x01, 0x03, 0x0A, 0x04, 0xEB, 0x01, 0xF1, 0x00, 0x00, 0x00, 0xFA, 0x01, 0xF4, 0xDE, 0xB2];

    #[test]
    fn read_request_matches_vendor_example() {
        assert_eq!(build_read_holding(1, 0, 5), REQ);
    }

    #[test]
    fn reply_decodes_to_vendor_values() {
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&REPLY, 1, &mut regs), ReadReply::Ok { count: 5, offset: 0 });
        // Vin 12.59 V, Vout 4.97 V, Iout 0 A, CC 2.50 A, CV 5.00 V
        assert_eq!(regs, [0x04EB, 0x01F1, 0x0000, 0x00FA, 0x01F4]);
    }

    #[test]
    fn echoed_request_in_front_of_reply_is_skipped() {
        let mut buf = [0u8; 23];
        buf[..8].copy_from_slice(&REQ);
        buf[8..].copy_from_slice(&REPLY);
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&buf, 1, &mut regs), ReadReply::Ok { count: 5, offset: 8 });
        assert_eq!(regs[0], 0x04EB);
    }

    #[test]
    fn echo_alone_is_not_a_reply() {
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&REQ, 1, &mut regs), ReadReply::None);
        assert_eq!(find_read_reply(&REPLY[..14], 1, &mut regs), ReadReply::None);
        assert_eq!(find_read_reply(&[], 1, &mut regs), ReadReply::None);
    }

    #[test]
    fn corrupted_crc_is_rejected() {
        let mut bad = REPLY;
        bad[5] ^= 0x01;
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&bad, 1, &mut regs), ReadReply::None);
    }

    #[test]
    fn wrong_address_is_ignored() {
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&REPLY, 2, &mut regs), ReadReply::None);
    }

    #[test]
    fn exception_frame_is_reported() {
        let mut exc = [0x01, 0x83, 0x02, 0, 0];
        let crc = crc16(&exc[..3]).to_le_bytes();
        exc[3..].copy_from_slice(&crc);
        let mut regs = [0u16; 5];
        assert_eq!(find_read_reply(&exc, 1, &mut regs), ReadReply::Exception { code: 2, offset: 0 });
    }

    #[test]
    fn short_register_buffer_truncates_but_reports_count() {
        let mut regs = [0u16; 2];
        assert_eq!(find_read_reply(&REPLY, 1, &mut regs), ReadReply::Ok { count: 5, offset: 0 });
        assert_eq!(regs, [0x04EB, 0x01F1]);
    }

    #[test]
    fn write_request_matches_vendor_example() {
        // Set CC = 2.50 A, CV = 5.00 V
        let frame = build_write_multiple(1, 0, &[0x00FA, 0x01F4]).unwrap();
        assert_eq!(frame.as_slice(), &[0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0xFA, 0x01, 0xF4, 0xD3, 0x89]);
        assert!(build_write_multiple(1, 0, &[]).is_none());
        assert!(build_write_multiple(1, 0, &[0; MAX_WRITE_REGS + 1]).is_none());
    }
}

//! The STM32 UART bootloader protocol (`firmware/BOOTLOADER.md`).
//!
//! A mirror of `firmware/Module/Inc/boot_shared.h` — the flash layout, the
//! reset-surviving RAM mailbox, the frame types and the packed payloads — plus
//! the sector table and erase rule of `firmware/bootloader/Src/bl_flash.c` and
//! the CRC-32 the bootloader checks an image with (`BlFlash_Crc32`, which is
//! zlib's `crc32`). The bootloader speaks the application's framing, so its
//! frames are built with [`crate::protocol::build_frame`] and parsed with
//! [`crate::protocol::FrameParser`].
//!
//! Host -> bootloader frames carry at most [`MAX_COMMAND_PAYLOAD`] bytes (a
//! longer one is dropped whole by `bl_protocol.c`); bootloader -> host frames
//! at most [`MAX_REPLY_PAYLOAD`].
//!
//! `tests/boot_vectors.rs` checks every constant, size, field offset and
//! packed layout against `tests/vectors/boot_oracle.json`, which a small C
//! program compiled against `boot_shared.h` and the unmodified `bl_flash.c`
//! writes (`tests/oracle/boot_generate.sh`), and the CRC against zlib.

use crate::protocol::build_frame;

// ---- flash layout --------------------------------------------------------

/// Start of the STM32F411CE's internal flash.
pub const FLASH_BASE_ADDRESS: u32 = 0x0800_0000;
/// Sectors 0-1 hold the bootloader.
pub const BOOTLOADER_SIZE: u32 = 0x0000_8000;
/// Sector 2: the application's vector table.
pub const APP_START_ADDRESS: u32 = 0x0800_8000;
/// Sector 7 (the PID settings) starts here; nothing writes it.
pub const APP_END_ADDRESS: u32 = 0x0806_0000;
/// 352 KB.
pub const APP_MAX_SIZE: u32 = APP_END_ADDRESS - APP_START_ADDRESS;
pub const APP_FIRST_SECTOR: u32 = 2;
pub const APP_LAST_SECTOR: u32 = 6;
pub const APP_VECTOR_OFFSET: u32 = APP_START_ADDRESS - FLASH_BASE_ADDRESS;

pub const RAM_BASE_ADDRESS: u32 = 0x2000_0000;
/// 128 KB.
pub const RAM_SIZE: u32 = 0x0002_0000;

// ---- reset-surviving mailbox ---------------------------------------------

/// The first 32 bytes of RAM, outside both linker scripts.
pub const SHARED_RAM_ADDRESS: u32 = 0x2000_0000;
pub const SHARED_RAM_SIZE: u32 = 32;
/// The app writes this to the mailbox and resets; the bootloader then stays.
pub const BOOT_REQUEST_MAGIC: u32 = 0xB007_B007;

// ---- bootloader behaviour ------------------------------------------------

/// What `BL_INFO` reports as `bootloader_version`.
pub const BOOT_VERSION: u8 = 1;
/// After a reset with a valid app and no request in the mailbox, the
/// bootloader waits this long for a `BL_PING` before it jumps.
pub const GRACE_PERIOD_MS: u32 = 500;
/// Data bytes per `BL_WRITE`, a multiple of 4.
pub const WRITE_CHUNK_MAX: usize = 128;

// ---- frame types ---------------------------------------------------------

/// Host -> app: reboot into the bootloader ([`enter_payload`]).
pub const ENTER_BOOTLOADER: u8 = 0x0F;
/// App -> host: [`EnterAck`], sent just before the reset.
pub const ENTER_BOOTLOADER_ACK: u8 = 0x8F;
/// Host -> bootloader, no payload; answered by [`BL_INFO`], and keeps the
/// bootloader from jumping.
pub const BL_PING: u8 = 0x10;
/// Host -> bootloader: [`erase_payload`].
pub const BL_ERASE: u8 = 0x11;
/// Host -> bootloader: [`write_payload`].
pub const BL_WRITE: u8 = 0x12;
/// Host -> bootloader: [`verify_payload`].
pub const BL_VERIFY: u8 = 0x13;
/// Host -> bootloader, no payload.
pub const BL_RUN_APP: u8 = 0x14;
/// Bootloader -> host: [`BlInfo`], the reply to [`BL_PING`].
pub const BL_INFO: u8 = 0x90;
/// Bootloader -> host: [`BlAck`], the reply to ERASE / WRITE / VERIFY / RUN.
pub const BL_ACK: u8 = 0x91;

// ---- payload sizes (sizeof the packed structs) ---------------------------

/// `boot_enter_payload_t`: u32 magic, u32 reserved.
pub const ENTER_PAYLOAD_LEN: usize = 8;
/// `boot_enter_ack_payload_t`: u8 status, u8 reserved[3].
pub const ENTER_ACK_PAYLOAD_LEN: usize = 4;
/// `boot_info_payload_t`.
pub const INFO_PAYLOAD_LEN: usize = 16;
/// `boot_erase_payload_t`: u32 image_size.
pub const ERASE_PAYLOAD_LEN: usize = 4;
/// `boot_write_header_t`: u32 offset, followed by the data.
pub const WRITE_HEADER_LEN: usize = 4;
/// `boot_verify_payload_t`: u32 image_size, u32 crc32.
pub const VERIFY_PAYLOAD_LEN: usize = 8;
/// `boot_ack_payload_t`: u8 command, u8 status, u32 value.
pub const ACK_PAYLOAD_LEN: usize = 6;
/// The largest payload the bootloader takes (`MAX_PAYLOAD` in
/// `bl_protocol.c`); its parser drops a frame announcing more.
pub const MAX_COMMAND_PAYLOAD: usize = WRITE_HEADER_LEN + WRITE_CHUNK_MAX;
/// The largest payload the bootloader sends (`send_frame` in `bl_protocol.c`).
pub const MAX_REPLY_PAYLOAD: usize = 32;

// ---- BL_INFO flags -------------------------------------------------------

/// The vector table at the app start looks like a real image.
pub const INFO_FLAG_APP_VALID: u8 = 0x01;
/// The app asked for the bootloader through the mailbox.
pub const INFO_FLAG_ENTERED_BY_REQUEST: u8 = 0x02;
/// An erase succeeded in this bootloader session.
pub const INFO_FLAG_ERASED: u8 = 0x04;

// ---- BL_ACK status -------------------------------------------------------

pub const STATUS_OK: u8 = 0;
pub const STATUS_BAD_ARGUMENT: u8 = 1;
pub const STATUS_FLASH_ERROR: u8 = 2;
pub const STATUS_CRC_MISMATCH: u8 = 3;
pub const STATUS_OUT_OF_RANGE: u8 = 4;
pub const STATUS_NOT_ERASED: u8 = 5;
pub const STATUS_NO_VALID_APP: u8 = 6;

/// The name `mower_flash.py` prints for a status (`STATUS_NAMES`); `None`
/// for a code the bootloader does not define.
pub fn status_name(status: u8) -> Option<&'static str> {
    Some(match status {
        STATUS_OK => "OK",
        STATUS_BAD_ARGUMENT => "BAD_ARGUMENT",
        STATUS_FLASH_ERROR => "FLASH_ERROR",
        STATUS_CRC_MISMATCH => "CRC_MISMATCH",
        STATUS_OUT_OF_RANGE => "OUT_OF_RANGE",
        STATUS_NOT_ERASED => "NOT_ERASED",
        STATUS_NO_VALID_APP => "NO_VALID_APP",
        _ => return None,
    })
}

// ---- sectors (bl_flash.c kSectors) ---------------------------------------

/// One flash sector: start address and size in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sector {
    pub start: u32,
    pub size: u32,
}

/// The STM32F411CE sector map, `kSectors` in `bl_flash.c`.
pub const SECTORS: [Sector; 8] = [
    Sector { start: 0x0800_0000, size: 16 * 1024 },
    Sector { start: 0x0800_4000, size: 16 * 1024 },
    Sector { start: 0x0800_8000, size: 16 * 1024 },
    Sector { start: 0x0800_C000, size: 16 * 1024 },
    Sector { start: 0x0801_0000, size: 64 * 1024 },
    Sector { start: 0x0802_0000, size: 128 * 1024 },
    Sector { start: 0x0804_0000, size: 128 * 1024 },
    Sector { start: 0x0806_0000, size: 128 * 1024 },
];

/// The sectors `BlFlash_EraseForImage(image_size)` erases: from
/// [`APP_FIRST_SECTOR`] through the last one that starts below the image's
/// end. `None` where it refuses (0 bytes, or more than [`APP_MAX_SIZE`]).
pub fn erase_sectors(image_size: u32) -> Option<std::ops::RangeInclusive<u32>> {
    if image_size == 0 || image_size > APP_MAX_SIZE {
        return None;
    }
    let image_end = APP_START_ADDRESS + image_size;
    let mut last = APP_FIRST_SECTOR;
    for s in APP_FIRST_SECTOR..=APP_LAST_SECTOR {
        if SECTORS[s as usize].start < image_end {
            last = s;
        }
    }
    Some(APP_FIRST_SECTOR..=last)
}

/// `BlFlash_AppLooksValid` on the first two words of the app region: the
/// stack pointer inside RAM and word aligned, the reset handler inside the
/// app region with the Thumb bit set.
pub fn app_looks_valid(sp: u32, pc: u32) -> bool {
    let sp_ok = sp > RAM_BASE_ADDRESS && sp <= RAM_BASE_ADDRESS + RAM_SIZE && sp & 0x3 == 0;
    let pc_ok = pc & 0x1 == 0x1 && pc >= APP_START_ADDRESS && pc < APP_END_ADDRESS;
    sp_ok && pc_ok
}

// ---- host -> bootloader payloads -------------------------------------------

#[inline]
fn get_u32(p: &[u8]) -> u32 {
    u32::from_le_bytes([p[0], p[1], p[2], p[3]])
}

/// `boot_enter_payload_t { magic, reserved = 0 }`.
pub fn enter_payload(magic: u32) -> [u8; ENTER_PAYLOAD_LEN] {
    let mut p = [0u8; ENTER_PAYLOAD_LEN];
    p[..4].copy_from_slice(&magic.to_le_bytes());
    p
}

/// `boot_erase_payload_t { image_size }`.
pub fn erase_payload(image_size: u32) -> [u8; ERASE_PAYLOAD_LEN] {
    image_size.to_le_bytes()
}

/// `boot_write_header_t { offset }` followed by `data`. `offset` counts from
/// [`APP_START_ADDRESS`]; the bootloader wants it and `data.len()` to be
/// multiples of 4, and `data` at most [`WRITE_CHUNK_MAX`] bytes.
pub fn write_payload(offset: u32, data: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(WRITE_HEADER_LEN + data.len());
    p.extend_from_slice(&offset.to_le_bytes());
    p.extend_from_slice(data);
    p
}

/// `boot_verify_payload_t { image_size, crc32 }`.
pub fn verify_payload(image_size: u32, crc32: u32) -> [u8; VERIFY_PAYLOAD_LEN] {
    let mut p = [0u8; VERIFY_PAYLOAD_LEN];
    p[..4].copy_from_slice(&image_size.to_le_bytes());
    p[4..].copy_from_slice(&crc32.to_le_bytes());
    p
}

/// `(magic, reserved)` of a [`ENTER_BOOTLOADER`] payload; `None` unless it is
/// [`ENTER_PAYLOAD_LEN`] long (the app ignores it then).
pub fn decode_enter_payload(p: &[u8]) -> Option<(u32, u32)> {
    if p.len() != ENTER_PAYLOAD_LEN {
        return None;
    }
    Some((get_u32(&p[0..]), get_u32(&p[4..])))
}

/// `image_size` of a [`BL_ERASE`] payload; `None` unless it is
/// [`ERASE_PAYLOAD_LEN`] long.
pub fn decode_erase_payload(p: &[u8]) -> Option<u32> {
    if p.len() != ERASE_PAYLOAD_LEN {
        return None;
    }
    Some(get_u32(p))
}

/// `(offset, data)` of a [`BL_WRITE`] payload; `None` unless there is at
/// least one data byte after the header (the bootloader's BAD_ARGUMENT).
pub fn decode_write_payload(p: &[u8]) -> Option<(u32, &[u8])> {
    if p.len() <= WRITE_HEADER_LEN {
        return None;
    }
    Some((get_u32(p), &p[WRITE_HEADER_LEN..]))
}

/// `(image_size, crc32)` of a [`BL_VERIFY`] payload; `None` unless it is
/// [`VERIFY_PAYLOAD_LEN`] long.
pub fn decode_verify_payload(p: &[u8]) -> Option<(u32, u32)> {
    if p.len() != VERIFY_PAYLOAD_LEN {
        return None;
    }
    Some((get_u32(&p[0..]), get_u32(&p[4..])))
}

/// 0x0F with [`BOOT_REQUEST_MAGIC`], what a host sends the running app.
pub fn build_enter_bootloader(seq: u8) -> Vec<u8> {
    build_frame(ENTER_BOOTLOADER, seq, &enter_payload(BOOT_REQUEST_MAGIC))
}

/// 0x10, no payload.
pub fn build_ping(seq: u8) -> Vec<u8> {
    build_frame(BL_PING, seq, &[])
}

/// 0x11.
pub fn build_erase(seq: u8, image_size: u32) -> Vec<u8> {
    build_frame(BL_ERASE, seq, &erase_payload(image_size))
}

/// 0x12.
pub fn build_write(seq: u8, offset: u32, data: &[u8]) -> Vec<u8> {
    build_frame(BL_WRITE, seq, &write_payload(offset, data))
}

/// 0x13.
pub fn build_verify(seq: u8, image_size: u32, crc32: u32) -> Vec<u8> {
    build_frame(BL_VERIFY, seq, &verify_payload(image_size, crc32))
}

/// 0x14, no payload.
pub fn build_run_app(seq: u8) -> Vec<u8> {
    build_frame(BL_RUN_APP, seq, &[])
}

// ---- bootloader / app -> host payloads -------------------------------------

/// 0x90, `boot_info_payload_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BlInfo {
    pub app_start_address: u32,
    pub app_max_size: u32,
    pub write_chunk_max: u16,
    pub bootloader_version: u8,
    /// `INFO_FLAG_*`
    pub flags: u8,
    pub reserved: u32,
}

impl BlInfo {
    pub fn app_valid(&self) -> bool {
        self.flags & INFO_FLAG_APP_VALID != 0
    }
    pub fn entered_by_request(&self) -> bool {
        self.flags & INFO_FLAG_ENTERED_BY_REQUEST != 0
    }
    pub fn erased(&self) -> bool {
        self.flags & INFO_FLAG_ERASED != 0
    }
    /// The packed bytes, as `send_info` fills them.
    pub fn encode(&self) -> [u8; INFO_PAYLOAD_LEN] {
        let mut p = [0u8; INFO_PAYLOAD_LEN];
        p[0..4].copy_from_slice(&self.app_start_address.to_le_bytes());
        p[4..8].copy_from_slice(&self.app_max_size.to_le_bytes());
        p[8..10].copy_from_slice(&self.write_chunk_max.to_le_bytes());
        p[10] = self.bootloader_version;
        p[11] = self.flags;
        p[12..16].copy_from_slice(&self.reserved.to_le_bytes());
        p
    }
}

/// `None` unless the payload is [`INFO_PAYLOAD_LEN`] long, like the other
/// decoders here and in [`crate::protocol`].
pub fn decode_bl_info(p: &[u8]) -> Option<BlInfo> {
    if p.len() != INFO_PAYLOAD_LEN {
        return None;
    }
    Some(BlInfo {
        app_start_address: get_u32(&p[0..]),
        app_max_size: get_u32(&p[4..]),
        write_chunk_max: u16::from_le_bytes([p[8], p[9]]),
        bootloader_version: p[10],
        flags: p[11],
        reserved: get_u32(&p[12..]),
    })
}

/// 0x91, `boot_ack_payload_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BlAck {
    /// The frame type acknowledged.
    pub command: u8,
    /// `STATUS_*`
    pub status: u8,
    /// ERASE: the image size; WRITE: the offset; VERIFY: the CRC-32 the
    /// bootloader computed over the flash; RUN_APP: the app start address.
    pub value: u32,
}

impl BlAck {
    pub fn encode(&self) -> [u8; ACK_PAYLOAD_LEN] {
        let mut p = [0u8; ACK_PAYLOAD_LEN];
        p[0] = self.command;
        p[1] = self.status;
        p[2..6].copy_from_slice(&self.value.to_le_bytes());
        p
    }
}

/// `None` unless the payload is [`ACK_PAYLOAD_LEN`] long.
pub fn decode_bl_ack(p: &[u8]) -> Option<BlAck> {
    if p.len() != ACK_PAYLOAD_LEN {
        return None;
    }
    Some(BlAck { command: p[0], status: p[1], value: get_u32(&p[2..]) })
}

/// 0x8F, `boot_enter_ack_payload_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EnterAck {
    pub status: u8,
    pub reserved: [u8; 3],
}

impl EnterAck {
    pub fn encode(&self) -> [u8; ENTER_ACK_PAYLOAD_LEN] {
        [self.status, self.reserved[0], self.reserved[1], self.reserved[2]]
    }
}

/// `None` unless the payload is [`ENTER_ACK_PAYLOAD_LEN`] long.
pub fn decode_enter_ack(p: &[u8]) -> Option<EnterAck> {
    if p.len() != ENTER_ACK_PAYLOAD_LEN {
        return None;
    }
    Some(EnterAck { status: p[0], reserved: [p[1], p[2], p[3]] })
}

// ---- CRC-32 ----------------------------------------------------------------

const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut bit = 0;
        while bit < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

/// Incremental CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320, init and final
/// xor 0xFFFFFFFF): `BlFlash_Crc32`, and zlib's / Python's `zlib.crc32`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Crc32 {
    state: u32,
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc32 {
    pub fn new() -> Self {
        Self { state: 0xFFFF_FFFF }
    }
    pub fn update(&mut self, data: &[u8]) {
        let mut c = self.state;
        for &b in data {
            c = CRC32_TABLE[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
        }
        self.state = c;
    }
    pub fn finish(&self) -> u32 {
        !self.state
    }
}

/// One-shot [`Crc32`].
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32_ieee(b""), 0);
    }

    #[test]
    fn erase_covers_exactly_the_image() {
        assert_eq!(erase_sectors(0), None);
        assert_eq!(erase_sectors(APP_MAX_SIZE + 1), None);
        assert_eq!(erase_sectors(1), Some(2..=2));
        assert_eq!(erase_sectors(16 * 1024), Some(2..=2));
        assert_eq!(erase_sectors(16 * 1024 + 1), Some(2..=3));
        assert_eq!(erase_sectors(APP_MAX_SIZE), Some(2..=6));
    }

    #[test]
    fn frames_round_trip() {
        use crate::protocol::FrameParser;
        let mut got = Vec::new();
        let mut parser = FrameParser::new();
        let mut stream = build_ping(1);
        stream.extend(build_write(2, 0x80, &[1, 2, 3, 4]));
        stream.extend(build_verify(3, 4096, 0xDEAD_BEEF));
        parser.feed(&stream, |t, s, p| got.push((t, s, p.to_vec())));
        assert_eq!(got.len(), 3);
        assert_eq!((got[0].0, got[0].1, got[0].2.len()), (BL_PING, 1, 0));
        assert_eq!(decode_write_payload(&got[1].2), Some((0x80, &[1u8, 2, 3, 4][..])));
        assert_eq!(decode_verify_payload(&got[2].2), Some((4096, 0xDEAD_BEEF)));
    }
}

//! `mower_base_core::boot` against the firmware's own definitions.
//!
//! `tests/vectors/boot_oracle.json` is written by `tests/oracle/boot_generate.sh`:
//! a C program compiled against `firmware/Module/Inc/boot_shared.h` and the
//! unmodified `firmware/bootloader/Src/bl_flash.c` (constants, sizeof /
//! offsetof, packed bytes, the sector map, what `BlFlash_EraseForImage`
//! erases), zlib's CRC-32, and what `firmware/tools/mower_flash.py` itself
//! builds and decodes. `tests/vectors/pop_frame_oracle.json` is the original
//! `FrameReader._pop_frame` of `mower_flash.py` run over byte streams cut into
//! random reads: the Rust flasher reads with `protocol::FrameParser`, which
//! must pop the same frames after every read.

use mower_base_core::boot::*;
use mower_base_core::protocol::{self, FrameParser};
use serde_json::Value;

fn vectors() -> Value {
    serde_json::from_str(include_str!("vectors/boot_oracle.json")).expect("boot oracle vectors are valid JSON")
}

fn unhex(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "odd hex string: {s}");
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).expect("hex"))
        .collect()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn u(v: &Value, k: &str) -> u64 {
    v[k].as_u64().unwrap_or_else(|| panic!("missing uint {k} in {v}"))
}

#[test]
fn constants_match_boot_shared_h() {
    let vs = vectors();
    let c = &vs["constants"];
    let expect: &[(&str, u64)] = &[
        ("BOOT_FLASH_BASE_ADDRESS", FLASH_BASE_ADDRESS as u64),
        ("BOOT_BOOTLOADER_SIZE", BOOTLOADER_SIZE as u64),
        ("BOOT_APP_START_ADDRESS", APP_START_ADDRESS as u64),
        ("BOOT_APP_END_ADDRESS", APP_END_ADDRESS as u64),
        ("BOOT_APP_MAX_SIZE", APP_MAX_SIZE as u64),
        ("BOOT_APP_FIRST_SECTOR", APP_FIRST_SECTOR as u64),
        ("BOOT_APP_LAST_SECTOR", APP_LAST_SECTOR as u64),
        ("BOOT_APP_VECTOR_OFFSET", APP_VECTOR_OFFSET as u64),
        ("BOOT_RAM_BASE_ADDRESS", RAM_BASE_ADDRESS as u64),
        ("BOOT_RAM_SIZE", RAM_SIZE as u64),
        ("BOOT_SHARED_RAM_ADDRESS", SHARED_RAM_ADDRESS as u64),
        ("BOOT_SHARED_RAM_SIZE", SHARED_RAM_SIZE as u64),
        ("BOOT_REQUEST_MAGIC", BOOT_REQUEST_MAGIC as u64),
        ("BOOT_VERSION", BOOT_VERSION as u64),
        ("BOOT_GRACE_PERIOD_MS", GRACE_PERIOD_MS as u64),
        ("BOOT_WRITE_CHUNK_MAX", WRITE_CHUNK_MAX as u64),
        ("BOOT_FRAME_TYPE_ENTER_BOOTLOADER", ENTER_BOOTLOADER as u64),
        ("BOOT_FRAME_TYPE_ENTER_BOOTLOADER_ACK", ENTER_BOOTLOADER_ACK as u64),
        ("BOOT_FRAME_TYPE_BL_PING", BL_PING as u64),
        ("BOOT_FRAME_TYPE_BL_ERASE", BL_ERASE as u64),
        ("BOOT_FRAME_TYPE_BL_WRITE", BL_WRITE as u64),
        ("BOOT_FRAME_TYPE_BL_VERIFY", BL_VERIFY as u64),
        ("BOOT_FRAME_TYPE_BL_RUN_APP", BL_RUN_APP as u64),
        ("BOOT_FRAME_TYPE_BL_INFO", BL_INFO as u64),
        ("BOOT_FRAME_TYPE_BL_ACK", BL_ACK as u64),
        ("BOOT_INFO_FLAG_APP_VALID", INFO_FLAG_APP_VALID as u64),
        ("BOOT_INFO_FLAG_ENTERED_BY_REQUEST", INFO_FLAG_ENTERED_BY_REQUEST as u64),
        ("BOOT_INFO_FLAG_ERASED", INFO_FLAG_ERASED as u64),
        ("BOOT_STATUS_OK", STATUS_OK as u64),
        ("BOOT_STATUS_BAD_ARGUMENT", STATUS_BAD_ARGUMENT as u64),
        ("BOOT_STATUS_FLASH_ERROR", STATUS_FLASH_ERROR as u64),
        ("BOOT_STATUS_CRC_MISMATCH", STATUS_CRC_MISMATCH as u64),
        ("BOOT_STATUS_OUT_OF_RANGE", STATUS_OUT_OF_RANGE as u64),
        ("BOOT_STATUS_NOT_ERASED", STATUS_NOT_ERASED as u64),
        ("BOOT_STATUS_NO_VALID_APP", STATUS_NO_VALID_APP as u64),
    ];
    assert_eq!(c.as_object().unwrap().len(), expect.len(), "a constant was added to the oracle");
    for (name, value) in expect {
        assert_eq!(u(c, name), *value, "{name}");
    }
}

#[test]
fn payload_sizes_and_offsets_match_the_packed_structs() {
    let vs = vectors();
    let s = &vs["sizes"];
    assert_eq!(u(s, "boot_enter_payload_t"), ENTER_PAYLOAD_LEN as u64);
    assert_eq!(u(s, "boot_enter_ack_payload_t"), ENTER_ACK_PAYLOAD_LEN as u64);
    assert_eq!(u(s, "boot_info_payload_t"), INFO_PAYLOAD_LEN as u64);
    assert_eq!(u(s, "boot_erase_payload_t"), ERASE_PAYLOAD_LEN as u64);
    assert_eq!(u(s, "boot_write_header_t"), WRITE_HEADER_LEN as u64);
    assert_eq!(u(s, "boot_verify_payload_t"), VERIFY_PAYLOAD_LEN as u64);
    assert_eq!(u(s, "boot_ack_payload_t"), ACK_PAYLOAD_LEN as u64);
    // bl_protocol.c: MAX_PAYLOAD = sizeof(boot_write_header_t) + BOOT_WRITE_CHUNK_MAX
    assert_eq!(MAX_COMMAND_PAYLOAD as u64, u(s, "boot_write_header_t") + u(&vs["constants"], "BOOT_WRITE_CHUNK_MAX"));
    let o = &vs["offsets"];
    for (field, at) in [
        ("boot_info_payload_t.app_start_address", 0),
        ("boot_info_payload_t.app_max_size", 4),
        ("boot_info_payload_t.write_chunk_max", 8),
        ("boot_info_payload_t.bootloader_version", 10),
        ("boot_info_payload_t.flags", 11),
        ("boot_info_payload_t.reserved", 12),
        ("boot_ack_payload_t.command", 0),
        ("boot_ack_payload_t.status", 1),
        ("boot_ack_payload_t.value", 2),
        ("boot_verify_payload_t.image_size", 0),
        ("boot_verify_payload_t.crc32", 4),
        ("boot_enter_payload_t.magic", 0),
        ("boot_enter_payload_t.reserved", 4),
        ("boot_enter_ack_payload_t.status", 0),
        ("boot_enter_ack_payload_t.reserved", 1),
        ("boot_erase_payload_t.image_size", 0),
        ("boot_write_header_t.offset", 0),
    ] {
        assert_eq!(u(o, field), at, "{field}");
    }
}

#[test]
fn packed_bytes_encode_and_decode_like_the_c_structs() {
    let vs = vectors();
    let cases = vs["packed"].as_array().unwrap();
    let mut seen = std::collections::BTreeSet::new();
    for c in cases {
        let f = &c["fields"];
        let bytes = unhex(c["bytes"].as_str().unwrap());
        let name = c["struct"].as_str().unwrap();
        seen.insert(name.to_string());
        match name {
            "boot_enter_payload_t" => {
                let (magic, reserved) = (u(f, "magic") as u32, u(f, "reserved") as u32);
                if reserved == 0 {
                    assert_eq!(enter_payload(magic).to_vec(), bytes);
                }
                assert_eq!(decode_enter_payload(&bytes), Some((magic, reserved)));
            }
            "boot_enter_ack_payload_t" => {
                let r: Vec<u8> = f["reserved"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap() as u8).collect();
                let ack = EnterAck { status: u(f, "status") as u8, reserved: [r[0], r[1], r[2]] };
                assert_eq!(ack.encode().to_vec(), bytes);
                assert_eq!(decode_enter_ack(&bytes), Some(ack));
            }
            "boot_info_payload_t" => {
                let info = BlInfo {
                    app_start_address: u(f, "app_start_address") as u32,
                    app_max_size: u(f, "app_max_size") as u32,
                    write_chunk_max: u(f, "write_chunk_max") as u16,
                    bootloader_version: u(f, "bootloader_version") as u8,
                    flags: u(f, "flags") as u8,
                    reserved: u(f, "reserved") as u32,
                };
                assert_eq!(info.encode().to_vec(), bytes);
                assert_eq!(decode_bl_info(&bytes), Some(info));
            }
            "boot_erase_payload_t" => {
                let size = u(f, "image_size") as u32;
                assert_eq!(erase_payload(size).to_vec(), bytes);
                assert_eq!(decode_erase_payload(&bytes), Some(size));
            }
            "boot_write_header_t" => {
                let off = u(f, "offset") as u32;
                let p = write_payload(off, &[0xAA, 0xBB, 0xCC, 0xDD]);
                assert_eq!(&p[..WRITE_HEADER_LEN], &bytes[..]);
                assert_eq!(decode_write_payload(&p), Some((off, &[0xAA, 0xBB, 0xCC, 0xDD][..])));
            }
            "boot_verify_payload_t" => {
                let (size, crc) = (u(f, "image_size") as u32, u(f, "crc32") as u32);
                assert_eq!(verify_payload(size, crc).to_vec(), bytes);
                assert_eq!(decode_verify_payload(&bytes), Some((size, crc)));
            }
            "boot_ack_payload_t" => {
                let ack = BlAck { command: u(f, "command") as u8, status: u(f, "status") as u8, value: u(f, "value") as u32 };
                assert_eq!(ack.encode().to_vec(), bytes);
                assert_eq!(decode_bl_ack(&bytes), Some(ack));
            }
            other => panic!("unexpected struct {other}"),
        }
    }
    assert_eq!(seen.len(), 7, "every payload struct has vectors");
}

#[test]
fn decoders_reject_every_wrong_length() {
    let buf = [0u8; 64];
    for n in 0..buf.len() {
        let p = &buf[..n];
        assert_eq!(decode_bl_info(p).is_some(), n == INFO_PAYLOAD_LEN, "info {n}");
        assert_eq!(decode_bl_ack(p).is_some(), n == ACK_PAYLOAD_LEN, "ack {n}");
        assert_eq!(decode_enter_ack(p).is_some(), n == ENTER_ACK_PAYLOAD_LEN, "enter ack {n}");
        assert_eq!(decode_enter_payload(p).is_some(), n == ENTER_PAYLOAD_LEN, "enter {n}");
        assert_eq!(decode_erase_payload(p).is_some(), n == ERASE_PAYLOAD_LEN, "erase {n}");
        assert_eq!(decode_verify_payload(p).is_some(), n == VERIFY_PAYLOAD_LEN, "verify {n}");
        // bl_protocol.c: payload_len <= sizeof(boot_write_header_t) is BAD_ARGUMENT
        assert_eq!(decode_write_payload(p).is_some(), n > WRITE_HEADER_LEN, "write {n}");
    }
}

#[test]
fn sector_map_and_erase_match_bl_flash_c() {
    let vs = vectors();
    let sectors = vs["sectors"].as_array().unwrap();
    assert_eq!(sectors.len(), SECTORS.len());
    for (s, v) in SECTORS.iter().zip(sectors) {
        assert_eq!(s.start as u64, u(v, "start"));
        assert_eq!(s.size as u64, u(v, "size"));
    }
    for e in vs["erase"].as_array().unwrap() {
        let size = u(e, "image_size") as u32;
        let ok = e["ok"].as_bool().unwrap();
        match erase_sectors(size) {
            None => assert!(!ok && u(e, "calls") == 0, "{e}"),
            Some(r) => {
                assert!(ok && u(e, "calls") == 1, "{e}");
                assert_eq!(*r.start() as u64, u(e, "first"), "{e}");
                assert_eq!((*r.end() - *r.start() + 1) as u64, u(e, "count"), "{e}");
            }
        }
    }
}

#[test]
fn app_vector_check_follows_bl_flash_c() {
    // BlFlash_AppLooksValid, read off the source: SP in (RAM base, RAM end]
    // and word aligned; PC odd and inside [app start, app end).
    assert!(app_looks_valid(0x2002_0000, 0x0800_8001));
    assert!(app_looks_valid(0x2000_0004, 0x0805_FFFF));
    assert!(!app_looks_valid(0x2000_0000, 0x0800_8001), "SP at the RAM base");
    assert!(!app_looks_valid(0x2002_0004, 0x0800_8001), "SP past the RAM end");
    assert!(!app_looks_valid(0x2001_FFFE, 0x0800_8001), "SP not word aligned");
    assert!(!app_looks_valid(0x2002_0000, 0x0800_8000), "PC without the Thumb bit");
    assert!(!app_looks_valid(0x2002_0000, 0x0800_7FFF), "PC below the app");
    assert!(!app_looks_valid(0x2002_0000, 0x0806_0001), "PC in the settings sector");
    assert!(!app_looks_valid(0xFFFF_FFFF, 0xFFFF_FFFF), "erased flash");
}

#[test]
fn crc32_matches_zlib() {
    let vs = vectors();
    for c in vs["crc32"].as_array().unwrap() {
        let data = unhex(c["data"].as_str().unwrap());
        let want = u(c, "crc") as u32;
        assert_eq!(crc32_ieee(&data), want, "crc32 of {} bytes", data.len());
        // any split gives the same as one pass
        for cut in [0, 1, data.len() / 3, data.len()] {
            let cut = cut.min(data.len());
            let mut inc = Crc32::new();
            inc.update(&data[..cut]);
            inc.update(&data[cut..]);
            assert_eq!(inc.finish(), want);
        }
    }
    for c in vs["crc32_fill"].as_array().unwrap() {
        let data = vec![u(c, "fill") as u8; u(c, "len") as usize];
        assert_eq!(crc32_ieee(&data), u(c, "crc") as u32, "{c}");
    }
}

#[test]
fn mower_flash_py_mirror_agrees() {
    let vs = vectors();
    let py = &vs["python"];
    let c = &py["constants"];
    for (name, value) in [
        ("SOF0", protocol::SOF0 as u64),
        ("SOF1", protocol::SOF1 as u64),
        ("VERSION", protocol::PROTOCOL_VERSION as u64),
        ("T_ENTER_BOOTLOADER", ENTER_BOOTLOADER as u64),
        ("T_ENTER_BOOTLOADER_ACK", ENTER_BOOTLOADER_ACK as u64),
        ("T_BL_PING", BL_PING as u64),
        ("T_BL_ERASE", BL_ERASE as u64),
        ("T_BL_WRITE", BL_WRITE as u64),
        ("T_BL_VERIFY", BL_VERIFY as u64),
        ("T_BL_RUN_APP", BL_RUN_APP as u64),
        ("T_BL_INFO", BL_INFO as u64),
        ("T_BL_ACK", BL_ACK as u64),
        ("T_INFO_REQUEST", protocol::INFO_REQUEST as u64),
        ("T_FIRMWARE_INFO", protocol::FIRMWARE_INFO as u64),
        ("BOOT_REQUEST_MAGIC", BOOT_REQUEST_MAGIC as u64),
        ("APP_START_ADDRESS", APP_START_ADDRESS as u64),
        ("APP_MAX_SIZE", APP_MAX_SIZE as u64),
        ("WRITE_CHUNK_MAX", WRITE_CHUNK_MAX as u64),
        ("INFO_FLAG_APP_VALID", INFO_FLAG_APP_VALID as u64),
        ("INFO_FLAG_ENTERED_BY_REQUEST", INFO_FLAG_ENTERED_BY_REQUEST as u64),
        ("INFO_FLAG_ERASED", INFO_FLAG_ERASED as u64),
    ] {
        assert_eq!(u(c, name), value, "mower_flash.py {name}");
    }
    let frames = py["host_frames"].as_array().unwrap();
    assert!(frames.len() >= 40);
    for f in frames {
        let a = f["args"].as_array().unwrap();
        let n = |i: usize| a[i].as_u64().unwrap();
        let seq = n(0) as u8;
        let got = match f["fn"].as_str().unwrap() {
            "ping" => build_ping(seq),
            "run_app" => build_run_app(seq),
            "enter_bootloader" => build_enter_bootloader(seq),
            "info_request" => protocol::build_info_request(seq),
            "erase" => build_erase(seq, n(1) as u32),
            "verify" => build_verify(seq, n(1) as u32, n(2) as u32),
            "write" => build_write(seq, n(1) as u32, &unhex(a[2].as_str().unwrap())),
            other => panic!("unexpected fn {other}"),
        };
        assert_eq!(hex(&got), f["bytes"].as_str().unwrap(), "{f}");
    }
    for p in py["parse_info"].as_array().unwrap() {
        let info = decode_bl_info(&unhex(p["bytes"].as_str().unwrap())).unwrap();
        let d = &p["parse_info"];
        assert_eq!(info.app_start_address as u64, u(d, "app_start"));
        assert_eq!(info.app_max_size as u64, u(d, "app_max_size"));
        assert_eq!(info.write_chunk_max as u64, u(d, "write_chunk_max"));
        assert_eq!(info.bootloader_version as u64, u(d, "version"));
        assert_eq!(info.flags as u64, u(d, "flags"));
        assert_eq!(info.app_valid(), d["app_valid"].as_bool().unwrap());
        assert_eq!(info.entered_by_request(), d["entered_by_request"].as_bool().unwrap());
        assert_eq!(info.erased(), d["erased"].as_bool().unwrap());
    }
    for a in py["ack_unpack"].as_array().unwrap() {
        let ack = decode_bl_ack(&unhex(a["bytes"].as_str().unwrap())).unwrap();
        let t = a["unpack"].as_array().unwrap();
        assert_eq!(
            (ack.command as u64, ack.status as u64, ack.value as u64),
            (t[0].as_u64().unwrap(), t[1].as_u64().unwrap(), t[2].as_u64().unwrap())
        );
    }
    let names = py["status_names"].as_object().unwrap();
    for status in 0..=255u8 {
        assert_eq!(
            status_name(status),
            names.get(&status.to_string()).map(|v| v.as_str().unwrap()),
            "status {status}"
        );
    }
}

type Popped = Vec<(u8, u8, Vec<u8>)>;

fn feed(parser: &mut FrameParser, data: &[u8]) -> Popped {
    let mut out = Vec::new();
    parser.feed(data, |t, s, p| out.push((t, s, p.to_vec())));
    out
}

#[test]
fn frame_parser_pops_what_mower_flash_py_pops() {
    let v: Value = serde_json::from_str(include_str!("vectors/pop_frame_oracle.json")).unwrap();
    let cases = v["cases"].as_array().unwrap();
    assert!(cases.len() >= 50);
    let mut frames = 0;
    for (i, reads) in cases.iter().enumerate() {
        let mut parser = FrameParser::new();
        for (j, r) in reads.as_array().unwrap().iter().enumerate() {
            let got = feed(&mut parser, &unhex(r["data"].as_str().unwrap()));
            let want: Popped = r["frames"]
                .as_array()
                .unwrap()
                .iter()
                .map(|f| {
                    let f = f.as_array().unwrap();
                    (f[0].as_u64().unwrap() as u8, f[1].as_u64().unwrap() as u8, unhex(f[2].as_str().unwrap()))
                })
                .collect();
            frames += want.len();
            assert_eq!(got, want, "case {i} read {j}");
            assert_eq!(parser.buffered() as u64, u(r, "left"), "bytes kept, case {i} read {j}");
        }
    }
    assert!(frames > 200, "the vectors pop real frames ({frames})");
}

/// `FrameReader._pop_frame` of `mower_flash.py`, line for line. It drops one
/// byte after a bad CRC where `FrameParser` drops two; the second byte is
/// always the `0x5A` of the SOF, which cannot start a frame, so the two must
/// agree on every stream. The vectors above check this copy against the real
/// Python; the random streams below check `FrameParser` against it at scale.
struct PyReader {
    buf: Vec<u8>,
}

impl PyReader {
    fn pop(&mut self) -> Option<(u8, u8, Vec<u8>)> {
        loop {
            let start = self.buf.windows(2).position(|w| w == [0xA5, 0x5A]);
            let Some(start) = start else {
                if self.buf.last() == Some(&0xA5) {
                    let keep = self.buf.len() - 1;
                    self.buf.drain(..keep);
                } else {
                    self.buf.clear();
                }
                return None;
            };
            self.buf.drain(..start);
            if self.buf.len() < 6 {
                return None;
            }
            let plen = self.buf[5] as usize;
            let total = 8 + plen;
            if self.buf.len() < total {
                return None;
            }
            let crc_rx = self.buf[6 + plen] as u16 | (self.buf[7 + plen] as u16) << 8;
            if crc_rx == protocol::crc16_ccitt_false(&self.buf[2..6 + plen]) && self.buf[2] == 0x01 {
                let f = (self.buf[3], self.buf[4], self.buf[6..6 + plen].to_vec());
                self.buf.drain(..total);
                return Some(f);
            }
            self.buf.drain(..1);
        }
    }
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 33) as u32
    }
    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

fn random_stream(rng: &mut Lcg) -> Vec<u8> {
    let mut out = Vec::new();
    for _ in 0..(4 + rng.below(20)) {
        let plen = rng.below(48) as usize;
        let payload: Vec<u8> = (0..plen).map(|_| rng.next() as u8).collect();
        let mut f = protocol::build_frame(rng.next() as u8, rng.next() as u8, &payload);
        match rng.below(10) {
            0 => {
                let i = f.len() - 1 - rng.below(2) as usize;
                f[i] ^= 1 << rng.below(8);
            }
            1 => f[2] = 0x02,
            2 => f.truncate(1 + rng.below(f.len() as u32 - 1) as usize),
            3 => f = (0..1 + rng.below(12)).map(|_| [0xA5, 0x5A, 0x00, 0xFF, rng.next() as u8][rng.below(5) as usize]).collect(),
            4 => f = vec![0xA5, 0x5A, 0x01, 0x91, 0x00, [0x40, 0x80, 0xFF][rng.below(3) as usize]],
            _ => {}
        }
        out.extend(f);
    }
    out
}

#[test]
fn frame_parser_equals_pop_frame_on_random_streams() {
    let mut rng = Lcg(0xE6E6_2026_1030);
    let mut frames = 0usize;
    for case in 0..3000 {
        let data = random_stream(&mut rng);
        let mut parser = FrameParser::new();
        let mut py = PyReader { buf: Vec::new() };
        let mut i = 0;
        while i < data.len() {
            let span = if rng.below(4) == 0 { 300 } else { 24 };
            let n = (1 + rng.below(span) as usize).min(data.len() - i);
            let chunk = &data[i..i + n];
            i += n;
            let got = feed(&mut parser, chunk);
            py.buf.extend_from_slice(chunk);
            let mut want = Vec::new();
            while let Some(f) = py.pop() {
                want.push(f);
            }
            frames += want.len();
            assert_eq!(got, want, "case {case}");
            assert_eq!(parser.buffered(), py.buf.len(), "case {case}");
        }
    }
    assert!(frames > 10_000, "random streams carry real frames ({frames})");
}

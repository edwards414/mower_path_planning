//! mower_flash against the real Python (`tests/vectors/python_oracle.json`,
//! written by `tests/oracle/pyfmt_gen.py` with CPython 3.12, the robot
//! image's): float and string repr, json.loads / json.dumps / str() on valid
//! and broken texts, os.path.splitext, load_image, and the original do_sync
//! run on a simulated port and clock for ~100 manifests. The same simulation
//! here must give the same stdout, report bytes, exit code or exception line,
//! frames and elapsed time.

use std::cell::{Cell, RefCell};
use std::rc::Rc;

use mower_base_core::protocol::build_frame;
use mower_flash::flasher::Session;
use mower_flash::image::{load_image, splitext_root};
use mower_flash::link::{Clock, Link};
use mower_flash::pyjson::{dumps, float_repr, loads, py_repr, py_str, str_repr};
use mower_flash::{Failure, Out};
use serde_json::Value;

fn vectors() -> Value {
    serde_json::from_str(include_str!("vectors/python_oracle.json")).expect("valid JSON")
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
}

#[test]
fn float_repr_matches() {
    let v = vectors();
    for c in v["float_repr"].as_array().unwrap() {
        let bits = unhex(c["bits"].as_str().unwrap());
        let x = f64::from_le_bytes(bits.try_into().unwrap());
        assert_eq!(float_repr(x), c["repr"].as_str().unwrap(), "{x:e}");
    }
}

#[test]
fn str_repr_matches() {
    let v = vectors();
    for c in v["str_repr"].as_array().unwrap() {
        assert_eq!(str_repr(c["s"].as_str().unwrap()), c["repr"].as_str().unwrap(), "{c}");
    }
}

#[test]
fn json_like_python() {
    let v = vectors();
    for c in v["loads_ok"].as_array().unwrap() {
        let text = c["text"].as_str().unwrap();
        let value = loads(text).unwrap_or_else(|e| panic!("{text}: {e}"));
        assert_eq!(dumps(&value, None), c["dumps"].as_str().unwrap(), "{text}");
        assert_eq!(dumps(&value, Some(2)), c["dump2"].as_str().unwrap(), "{text}");
        assert_eq!(py_str(&value), c["str"].as_str().unwrap(), "{text}");
        assert_eq!(py_repr(&value), c["repr"].as_str().unwrap(), "{text}");
    }
    for c in v["loads_err"].as_array().unwrap() {
        let text = c["text"].as_str().unwrap();
        assert_eq!(loads(text).err().as_deref(), Some(c["error"].as_str().unwrap()), "{text:?}");
    }
}

#[test]
fn splitext_matches() {
    let v = vectors();
    for c in v["splitext"].as_array().unwrap() {
        assert_eq!(splitext_root(c["p"].as_str().unwrap()), c["root"].as_str().unwrap(), "{c}");
    }
}

fn failure_line(f: &Failure) -> String {
    match f {
        Failure::Exit(m) | Failure::Uncaught(m) | Failure::Runtime(m) | Failure::Timeout(m) => m.clone(),
    }
}

#[test]
fn load_image_matches() {
    let v = vectors();
    let dir = tempdir("load_image");
    for c in v["load_image"].as_array().unwrap() {
        let name = c["name"].as_str().unwrap();
        let path = format!("{dir}/{name}");
        match c["kind"].as_str().unwrap() {
            "dir" => std::fs::create_dir(&path).unwrap(),
            "missing" => {}
            _ => {
                let data = match c.get("data") {
                    Some(d) => unhex(d.as_str().unwrap()),
                    None => {
                        let mut d = unhex(c["head"].as_str().unwrap());
                        d.resize(8 + c["zeros"].as_u64().unwrap() as usize, 0);
                        d
                    }
                };
                std::fs::write(&path, data).unwrap();
            }
        }
        match load_image(&path) {
            Ok(img) => {
                let ok = &c["ok"];
                assert_eq!(img.len() as u64, ok["len"].as_u64().unwrap(), "{name}");
                assert_eq!(mower_base_core::boot::crc32_ieee(&img) as u64, ok["crc32"].as_u64().unwrap(), "{name}");
            }
            Err(Failure::Exit(m)) => assert_eq!(Some(m.as_str()), c["exit"].as_str(), "{name}"),
            Err(f) => assert_eq!(Some(failure_line(&f).replace(&dir, "DIR").as_str()), c["error"].as_str(), "{name}"),
        }
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

fn tempdir(tag: &str) -> String {
    let d = std::env::temp_dir().join(format!("mower_flash_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d.to_string_lossy().into_owned()
}

/// pyfmt_gen.py's Clock and Port.
#[derive(Clone)]
struct SimClock(Rc<Cell<f64>>);

impl Clock for SimClock {
    fn monotonic(&self) -> f64 {
        self.0.get()
    }
    fn time(&self) -> f64 {
        1700000000.5
    }
    fn sleep(&self, s: f64) {
        self.0.set(self.0.get() + s);
    }
}

struct SimPort {
    clock: SimClock,
    board: String,
    pending: Vec<u8>,
    writes: Rc<RefCell<Vec<String>>>,
}

impl Link for SimPort {
    fn write(&mut self, data: &[u8]) -> Result<(), Failure> {
        self.writes.borrow_mut().push(data.iter().map(|b| format!("{b:02x}")).collect());
        if data[3] == 0x06 && (self.board == "app" || self.board == "unversioned") {
            let mut p = vec![0, 6, 0, 1];
            p.extend(0xE6E6_E6E6u32.to_le_bytes());
            p.extend(1_793_000_000u32.to_le_bytes());
            p.extend([if self.board == "unversioned" { 0x02 } else { 0 }, 0, 0, 0]);
            self.pending.extend(build_frame(0x87, 0, &p));
        }
        Ok(())
    }

    fn read(&mut self, max: usize, window: f64) -> Result<Vec<u8>, Failure> {
        if self.pending.len() >= max {
            return Ok(self.pending.drain(..max).collect());
        }
        self.clock.sleep(window);
        Ok(std::mem::take(&mut self.pending))
    }
}

#[test]
fn do_sync_matches_the_original_on_a_simulated_port() {
    let v = vectors();
    let cases = v["sync"].as_array().unwrap();
    assert!(cases.len() > 150);
    let dir = tempdir("sync");
    let img = format!("{dir}/img.bin");
    let mut image = 0x2002_0000u32.to_le_bytes().to_vec();
    image.extend(0x0800_81C1u32.to_le_bytes());
    image.extend(0u8..56);
    std::fs::write(&img, image).unwrap();
    let manifest = format!("{dir}/m.json");
    let report = format!("{dir}/r.json");
    for c in cases {
        let text = c["text"].as_str().unwrap();
        let bytes: Vec<u8> =
            if c["bytes"].as_bool().unwrap() { text.chars().map(|ch| ch as u32 as u8).collect() } else { text.as_bytes().to_vec() };
        std::fs::write(&manifest, bytes).unwrap();
        let _ = std::fs::remove_file(&report);
        let clock = SimClock(Rc::new(Cell::new(1000.0)));
        let writes = Rc::new(RefCell::new(Vec::new()));
        let port = SimPort { clock: clock.clone(), board: c["board"].as_str().unwrap().into(), pending: Vec::new(), writes: writes.clone() };
        let mut s = Session::new(port, clock.clone(), Out::capture(), None);
        let what = format!("{} / {} / force={}", c["manifest"], c["board"], c["force"]);
        let result = s.do_sync(&img, Some(&manifest), Some(&report), c["force"].as_bool().unwrap());
        match (&result, c["rc"].as_i64()) {
            (Ok(rc), Some(want)) => assert_eq!(*rc as i64, want, "{what}"),
            (Err(f), None) => {
                assert_eq!(Some(failure_line(f).replace(&dir, "DIR").as_str()), c["error"].as_str(), "{what}")
            }
            (got, want) => panic!("{what}: {got:?} vs rc {want:?} / {}", c["error"]),
        }
        assert_eq!(s.out.text(), c["stdout"].as_str().unwrap(), "{what}");
        let got_report = std::fs::read_to_string(&report).ok();
        assert_eq!(got_report.as_deref(), c["report"].as_str(), "{what}");
        let want_writes: Vec<String> = c["writes"].as_array().unwrap().iter().map(|w| w.as_str().unwrap().to_string()).collect();
        assert_eq!(*writes.borrow(), want_writes, "{what}");
        let elapsed = clock.monotonic() - 1000.0;
        assert!((elapsed - c["elapsed"].as_f64().unwrap()).abs() < 1e-6, "{what}: {elapsed} s");
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

/// pyfmt_gen.py's BlPort: a scripted bootloader, every reply at once, the
/// app after RUN_APP.
struct BlPort {
    clock: SimClock,
    cfg: Value,
    pending: Vec<u8>,
    writes: Rc<RefCell<Vec<String>>>,
    writes_seen: u64,
    in_app: bool,
}

impl BlPort {
    fn cfg(&self, key: &str, default: u64) -> u64 {
        self.cfg.get(key).and_then(Value::as_u64).unwrap_or(default)
    }
    fn listed(&self, key: &str, n: u64) -> bool {
        self.cfg.get(key).and_then(Value::as_array).is_some_and(|a| a.iter().any(|v| v.as_u64() == Some(n)))
    }
    fn reply(&mut self, ftype: u8, seq: u8, payload: &[u8]) {
        self.pending.extend(build_frame(ftype, seq, payload));
    }
    fn ack(&mut self, cmd: u8, seq: u8, status: u8, value: u32) {
        let mut p = vec![cmd, status];
        p.extend(value.to_le_bytes());
        self.reply(0x91, seq, &p);
    }
}

impl Link for BlPort {
    fn write(&mut self, data: &[u8]) -> Result<(), Failure> {
        self.writes.borrow_mut().push(data.iter().map(|b| format!("{b:02x}")).collect());
        let (ftype, seq, payload) = (data[3], data[4], data[6..data.len() - 2].to_vec());
        let u32_at = |p: &[u8], i: usize| u32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        if self.in_app {
            if ftype == 0x06 {
                let mut p = vec![0, 6, 0, 1];
                p.extend(0xE6E6_E6E6u32.to_le_bytes());
                p.extend(1_793_000_000u32.to_le_bytes());
                p.extend([0, 0, 0, 0]);
                self.reply(0x87, 0, &p);
            }
            return Ok(());
        }
        match ftype {
            0x10 => {
                let mut info = 0x0800_8000u32.to_le_bytes().to_vec();
                info.extend(0x58000u32.to_le_bytes());
                info.extend((self.cfg("chunk", 128) as u16).to_le_bytes());
                info.extend([1, 0x01, 0, 0, 0, 0]);
                info.truncate(self.cfg("info_len", 16) as usize);
                self.reply(0x90, seq, &info);
            }
            0x11 => {
                let v = u32_at(&payload, 0);
                self.ack(0x11, seq, self.cfg("erase_status", 0) as u8, v);
            }
            0x12 => {
                self.writes_seen += 1;
                let n = self.writes_seen;
                let off = u32_at(&payload, 0);
                if self.listed("drop", n) {
                    return Ok(());
                }
                if self.listed("stale", n) {
                    self.ack(0x12, seq.wrapping_sub(1), 0, off);
                    self.ack(0x11, seq, 0, off);
                }
                let status = if Some(n) == self.cfg.get("nak_at").and_then(Value::as_u64) { self.cfg("nak_status", 0) } else { 0 };
                self.ack(0x12, seq, status as u8, off);
            }
            0x13 => {
                let crc = u32_at(&payload, 4);
                self.ack(0x13, seq, self.cfg("verify_status", 0) as u8, self.cfg("verify_value", crc as u64) as u32);
            }
            0x14 => {
                let status = self.cfg("run_status", 0);
                self.ack(0x14, seq, status as u8, 0x0800_8000);
                self.in_app = status == 0;
            }
            _ => {}
        }
        Ok(())
    }

    fn read(&mut self, max: usize, window: f64) -> Result<Vec<u8>, Failure> {
        if self.pending.len() >= max {
            return Ok(self.pending.drain(..max).collect());
        }
        self.clock.sleep(window);
        Ok(std::mem::take(&mut self.pending))
    }
}

fn sha256_hex(data: &[u8]) -> String {
    // FIPS 180-4, enough for comparing a digest the vectors carry
    let mut h: [u32; 8] = [0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01,
        0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08,
        0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    ];
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend(((data.len() as u64) * 8).to_be_bytes());
    for block in msg.chunks(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([block[4 * i], block[4 * i + 1], block[4 * i + 2], block[4 * i + 3]]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16].wrapping_add(s0).wrapping_add(w[i - 7]).wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6) ^ v[4].rotate_right(11) ^ v[4].rotate_right(25);
            let ch = (v[4] & v[5]) ^ (!v[4] & v[6]);
            let t1 = v[7].wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2) ^ v[0].rotate_right(13) ^ v[0].rotate_right(22);
            let maj = (v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]);
            let t2 = s0.wrapping_add(maj);
            v = [t1.wrapping_add(t2), v[0], v[1], v[2], v[3].wrapping_add(t1), v[4], v[5], v[6]];
        }
        for i in 0..8 {
            h[i] = h[i].wrapping_add(v[i]);
        }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

#[test]
fn do_flash_matches_the_original_on_a_scripted_bootloader() {
    assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    let v = vectors();
    let cases = v["flash"].as_array().unwrap();
    assert!(cases.len() >= 20);
    let dir = tempdir("flash");
    let manifest = format!("{dir}/m.json");
    let report = format!("{dir}/r.json");
    let img = format!("{dir}/img.bin");
    // with_() of pyfmt_gen.py: the canonical manifest, as json.dumps(indent=2) + "\n"
    let canonical = v["sync"].as_array().unwrap().iter().find(|c| c["manifest"] == "canonical").unwrap()["text"].as_str().unwrap().to_string();
    std::fs::write(&manifest, canonical).unwrap();
    for c in cases {
        let size = c["size"].as_u64().unwrap() as usize;
        let mut image = 0x2002_0000u32.to_le_bytes().to_vec();
        image.extend(0x0800_81C1u32.to_le_bytes());
        image.extend((0..size - 8).map(|i| (i * 7) as u8));
        std::fs::write(&img, image).unwrap();
        let _ = std::fs::remove_file(&report);
        let clock = SimClock(Rc::new(Cell::new(1000.0)));
        let writes = Rc::new(RefCell::new(Vec::new()));
        let port = BlPort { clock: clock.clone(), cfg: c["cfg"].clone(), pending: Vec::new(), writes: writes.clone(), writes_seen: 0, in_app: false };
        let mut s = Session::new(port, clock.clone(), Out::capture(), None);
        let what = c["name"].as_str().unwrap();
        let result = s.do_sync(&img, Some(&manifest), Some(&report), false);
        match (&result, c["rc"].as_i64()) {
            (Ok(rc), Some(want)) => assert_eq!(*rc as i64, want, "{what}"),
            (Err(f), None) => assert_eq!(Some(failure_line(f).as_str()), c["error"].as_str(), "{what}"),
            (got, want) => panic!("{what}: {got:?} vs rc {want:?} / {}", c["error"]),
        }
        assert_eq!(s.out.text(), c["stdout"].as_str().unwrap(), "{what}");
        assert_eq!(std::fs::read_to_string(&report).ok().as_deref(), c["report"].as_str(), "{what}");
        let got = writes.borrow().clone();
        match &c["writes"] {
            Value::Array(want) => {
                let want: Vec<String> = want.iter().map(|w| w.as_str().unwrap().to_string()).collect();
                assert_eq!(got, want, "{what}");
            }
            digest => {
                assert_eq!(got.len() as u64, digest["count"].as_u64().unwrap(), "{what}");
                assert_eq!(sha256_hex(got.join("\n").as_bytes()), digest["sha256"].as_str().unwrap(), "{what}");
            }
        }
        let elapsed = clock.monotonic() - 1000.0;
        assert!((elapsed - c["elapsed"].as_f64().unwrap()).abs() < 1e-6, "{what}: {elapsed} s");
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

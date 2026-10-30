//! `mower_flash`: the STM32 firmware flasher, a port of
//! `firmware/tools/mower_flash.py` (E6 of `docs/ROS_FREE_PLAN.md`).
//!
//! It must be indistinguishable from the Python tool on the wire and in its
//! results: the same frames in the same order with the same timeouts, sleeps,
//! retries and 50 ms read windows, the same flash contents, the same
//! `--report` bytes, the same stdout, the same exit codes, and the same error
//! handling down to what Python does not catch (exit 1, no report; the
//! exception's line on stderr). `tools/flash_check.py` proves it against the
//! unmodified bootloader source on a fake board (`tools/fake_base.py
//! --bootloader-lib`); `tests/` holds the pieces to vectors the real Python
//! wrote.
//!
//! The one deliberate difference: the port is opened exclusively
//! ([`link::SerialLink`]).
//!
//! * [`pyjson`] — `json.load` / `json.dump` / `str()` / `repr()` as CPython
//!   3.12 does them;
//! * [`link`] — the serial port with pyserial's read semantics, and the
//!   clocks;
//! * [`flasher`] — `FrameReader`, `Bootloader`, `read_app_info`, `do_flash`,
//!   `do_sync`;
//! * [`image`] — `load_image`, `load_manifest`;
//! * [`cli`] — the command line and `main()`.

pub mod cli;
pub mod flasher;
pub mod image;
pub mod link;
pub mod pyjson;

use std::io::Write;

/// How a call in `mower_flash.py` ends when it does not return: the exception
/// it raises, sorted by who catches it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Failure {
    /// `TimeoutError(msg)`: `do_sync` reports it ("flash failed: ..."),
    /// `main` prints `error: msg` and exits 1; the write loop retries it.
    Timeout(String),
    /// `RuntimeError(msg)`: the same, but never retried.
    Runtime(String),
    /// `sys.exit(msg)`: `msg` on stderr, exit 1.
    Exit(String),
    /// Anything Python does not catch: exit 1, no report; the payload is the
    /// last line of its traceback, e.g. `KeyError: 'version'`.
    Uncaught(String),
}

/// Python's `sys.stdout`: line buffered on a terminal, otherwise written only
/// when flushed (`print(..., flush=True)`), when 8 KB have piled up, or at
/// exit — which is what `docker logs` and a killed flasher show.
pub struct Out {
    buf: Vec<u8>,
    sink: Option<Box<dyn Write>>,
    line_buffered: bool,
    /// Everything flushed so far, when there is no sink (tests).
    pub captured: Vec<u8>,
}

impl Out {
    /// The process's stdout.
    pub fn stdout() -> Out {
        let tty = unsafe { libc::isatty(1) } == 1;
        Out { buf: Vec::new(), sink: Some(Box::new(std::io::stdout())), line_buffered: tty, captured: Vec::new() }
    }

    /// A capture, for tests.
    pub fn capture() -> Out {
        Out { buf: Vec::new(), sink: None, line_buffered: false, captured: Vec::new() }
    }

    /// `print(text, end="")`.
    pub fn write(&mut self, text: &str) {
        self.buf.extend_from_slice(text.as_bytes());
        if (self.line_buffered && text.contains('\n')) || self.buf.len() >= 8192 {
            self.flush();
        }
    }

    /// `print(text)`.
    pub fn println(&mut self, text: &str) {
        self.write(&format!("{text}\n"));
    }

    pub fn flush(&mut self) {
        if self.buf.is_empty() {
            return;
        }
        match self.sink.as_mut() {
            Some(sink) => {
                let _ = sink.write_all(&self.buf);
                let _ = sink.flush();
            }
            None => self.captured.extend_from_slice(&self.buf),
        }
        self.buf.clear();
    }

    /// What has been printed, flushed or not (tests).
    pub fn text(&self) -> String {
        let mut all = self.captured.clone();
        all.extend_from_slice(&self.buf);
        String::from_utf8_lossy(&all).into_owned()
    }
}

/// `--trace FILE`: one JSON line per frame sent or received, in the format of
/// `tools/fake_base.py`'s log, so a trace taken on the robot lines up with a
/// transcript taken on the fake.
pub struct Trace {
    file: std::fs::File,
}

impl Trace {
    pub fn create(path: &str) -> std::io::Result<Trace> {
        Ok(Trace { file: std::fs::File::create(path)? })
    }

    pub fn frame(&mut self, t: f64, dir: &str, ftype: u8, seq: u8, payload: &[u8]) {
        let hex: String = payload.iter().map(|b| format!("{b:02x}")).collect();
        let _ = writeln!(
            self.file,
            "{{\"t\":{t:.6},\"dir\":\"{dir}\",\"type\":{ftype},\"seq\":{seq},\"payload\":\"{hex}\"}}"
        );
    }
}

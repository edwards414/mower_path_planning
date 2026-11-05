//! `mowerctl pair`: `deploy/host/mower-pair` without Python (E6 design 3.5). The
//! robot's identity and the pairing QR.
//!
//! `identity.json` holds `robot_id` (from `/etc/machine-id`), `name`, `secret`,
//! `created` and, after a rotation, `rotated`, plus whatever else is in it: the
//! agent adds `device_key`. It is 0600 and owned by the robot user, since Studio
//! reads it over ssh as that user. The QR's URL carries the parameters the app
//! parses (`v`, `id`, `s`, `n`, `h`, `l`) with `quote_plus`'s escaping.
//!
//! The flags, defaults, files and stdout are mower-pair's, held to
//! `deploy/test/golden/host/g5_pair.json`. That includes what the design wants
//! changed later (D6): an `identity.json` that does not parse is silently
//! replaced and its `device_key` lost, and a new secret needs a stack restart.
//!
//! It runs as root and the state dir is the robot user's, so reading and writing
//! there go through the open directory and the open file, as in mower-pair: no
//! link is followed and no owner or mode is set by name (`mower_link::statedir`).
//!
//! Differences nothing can see or that do not come up:
//! - the owner comes from `/etc/passwd`, not NSS, which keeps this a static
//!   binary; an owner only LDAP knows counts as unknown, and is skipped as
//!   `KeyError` is;
//! - a help text of its own; on an argument that is not UTF-8 mowerctl stops
//!   before getting here;
//! - `json::loads`'s gaps: an int outside 64 bits makes the file one that does
//!   not parse, a lone surrogate escape comes back as U+FFFD. Neither mower-pair
//!   nor the agent writes those.

use std::fs::File;
use std::io::{IsTerminal, Read, Write};

use mower_link::py;
use mower_link::statedir::StateDir;
use mower_pyfmt::json::{self, JsonOptions};
use mower_pyfmt::{repr, Py};
use sha2::{Digest, Sha256};

use crate::Fail;

const ID_ALPHABET: &[u8; 32] = b"0123456789ABCDEFGHJKMNPQRSTVWXYZ";
const PAIR_BASE_URL: &str = "https://mower.fxrbindi.com/pair";
const MACHINE_ID_FILES: [&str; 2] = ["/etc/machine-id", "/var/lib/dbus/machine-id"];
const DEFAULT_STATE_DIR: &str = mower_link::DEFAULT_STATE_DIR;
const DEFAULT_OWNER: &str = "cat";
const DEFAULT_ENV: &str = "/opt/mower/.env";
/// argparse's exit code, kept: nothing here is a connection (main.rs's 2).
const EXIT_USAGE: u8 = 2;

const HELP: &str = "\
usage: mowerctl pair [-h] [--state-dir STATE_DIR] [--owner OWNER] [--env ENV]
                     [--name NAME] [--rotate] [--json] [--no-qr]

Robot identity and pairing QR code (run on the robot, as root or the robot user).

    sudo mowerctl pair              # create identity.json if missing, print id + QR
    sudo mowerctl pair --rotate     # new secret: every paired phone must re-scan
    sudo mowerctl pair --json       # machine-readable
    sudo mowerctl pair --name front # set the display name

options:
  --state-dir DIR  where identity.json lives ($MOWER_STATE_DIR, /home/cat/.mower)
  --owner USER     owner of identity.json (the robot user, cat)
  --env FILE       where PAIR_RELAY_URL is (/opt/mower/.env)
  --name NAME      display name shown in the app
  --rotate         generate a new secret
  --json           machine-readable
  --no-qr          no QR code
";

#[derive(Debug, Clone, PartialEq)]
pub struct Args {
    pub state_dir: String,
    pub owner: String,
    pub env: String,
    pub name: Option<String>,
    pub rotate: bool,
    pub json: bool,
    pub no_qr: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parsed {
    Run(Args),
    Help,
}

/// Option strings, in the order argparse knows them, and whether one takes a value.
const OPTIONS: [(&str, bool); 9] = [
    ("-h", false),
    ("--help", false),
    ("--state-dir", true),
    ("--owner", true),
    ("--env", true),
    ("--name", true),
    ("--rotate", false),
    ("--json", false),
    ("--no-qr", false),
];

fn takes_value(opt: &str) -> bool {
    OPTIONS.iter().any(|(o, v)| *o == opt && *v)
}

fn known(opt: &str) -> Option<&'static str> {
    OPTIONS.iter().find(|(o, _)| *o == opt).map(|(o, _)| *o)
}

/// What argparse calls an argument before it consumes any.
#[derive(Debug, Clone, PartialEq)]
enum Class {
    Positional,
    Option { opt: &'static str, explicit: Option<String> },
    Unknown,
}

fn is_negative_number(arg: &str) -> bool {
    // '^-\d+$|^-\d*\.\d+$'
    let Some(rest) = arg.strip_prefix('-') else {
        return false;
    };
    let digits = |s: &str| !s.is_empty() && s.chars().all(|c| c.is_ascii_digit());
    digits(rest) || rest.split_once('.').is_some_and(|(a, b)| (a.is_empty() || digits(a)) && digits(b))
}

/// `_parse_optional` with `_get_option_tuples`, CPython 3.12.
fn classify(arg: &str) -> Result<Class, String> {
    if arg.is_empty() || !arg.starts_with('-') {
        return Ok(Class::Positional);
    }
    if let Some(opt) = known(arg) {
        return Ok(Class::Option { opt, explicit: None });
    }
    if arg.chars().count() == 1 {
        return Ok(Class::Positional);
    }
    if let Some((o, v)) = arg.split_once('=') {
        if let Some(opt) = known(o) {
            return Ok(Class::Option { opt, explicit: Some(v.to_string()) });
        }
    }
    let mut hits: Vec<(&'static str, Option<String>)> = Vec::new();
    if arg.starts_with("--") {
        let (prefix, explicit) = match arg.split_once('=') {
            Some((p, v)) => (p, Some(v.to_string())),
            None => (arg, None),
        };
        hits.extend(OPTIONS.iter().filter(|(o, _)| o.starts_with(prefix)).map(|(o, _)| (*o, explicit.clone())));
    } else {
        let split = arg.char_indices().nth(2).map_or(arg.len(), |(i, _)| i);
        let (short, rest) = arg.split_at(split);
        for (o, _) in OPTIONS.iter() {
            if *o == short {
                hits.push((*o, Some(rest.to_string())));
            } else if o.starts_with(arg) {
                hits.push((*o, None));
            }
        }
    }
    match hits.len() {
        0 => {}
        1 => {
            let (opt, explicit) = hits.remove(0);
            return Ok(Class::Option { opt, explicit });
        }
        _ => {
            let names: Vec<&str> = hits.iter().map(|(o, _)| *o).collect();
            return Err(format!("ambiguous option: {arg} could match {}", names.join(", ")));
        }
    }
    if is_negative_number(arg) || arg.contains(' ') {
        return Ok(Class::Positional);
    }
    Ok(Class::Unknown)
}

/// mower-pair's command line by argparse's rules: an option may be cut short
/// while it stays unique, a value is the next argument unless that looks like an
/// option, options act in their order, so `-h` wins over a later mistake, a
/// value missing before it loses, and only at the end do unknown arguments count.
pub fn parse_args(argv: &[String], env_state_dir: Option<String>) -> Result<Parsed, String> {
    let mut args = Args {
        state_dir: env_state_dir.unwrap_or_else(|| DEFAULT_STATE_DIR.to_string()),
        owner: DEFAULT_OWNER.to_string(),
        env: DEFAULT_ENV.to_string(),
        name: None,
        rotate: false,
        json: false,
        no_qr: false,
    };
    // the first pass of _parse_known_args: an ambiguous option fails at once
    let dashes = argv.iter().position(|a| a == "--").unwrap_or(argv.len());
    let mut classes: Vec<Class> = Vec::with_capacity(argv.len());
    for (i, arg) in argv.iter().enumerate() {
        classes.push(if i < dashes { classify(arg)? } else { Class::Positional });
    }
    let mut extras: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < argv.len() {
        let class = &classes[i];
        let arg = argv[i].as_str();
        i += 1;
        let (opt, explicit) = match class {
            Class::Positional | Class::Unknown => {
                extras.push(arg);
                continue;
            }
            Class::Option { opt, explicit } => (*opt, explicit.clone()),
        };
        let value = if takes_value(opt) {
            match explicit {
                Some(v) => v,
                None => {
                    // an optional's pattern has no room for a `--`
                    if i >= argv.len() || i == dashes || !matches!(classes[i], Class::Positional) {
                        return Err(format!("argument {opt}: expected one argument"));
                    }
                    i += 1;
                    argv[i - 1].clone()
                }
            }
        } else {
            if let Some(e) = &explicit {
                // `-hx` is the help and an extra `-x`, and the help goes first
                if opt == "-h" && !arg.starts_with("-h=") && !e.is_empty() && !e.starts_with('-') {
                    return Ok(Parsed::Help);
                }
                return Err(format!("argument {opt}: ignored explicit argument {}", py::repr_str(e)));
            }
            String::new()
        };
        match opt {
            "-h" | "--help" => return Ok(Parsed::Help),
            "--state-dir" => args.state_dir = value,
            "--owner" => args.owner = value,
            "--env" => args.env = value,
            "--name" => args.name = Some(value),
            "--rotate" => args.rotate = true,
            "--json" => args.json = true,
            "--no-qr" => args.no_qr = true,
            _ => unreachable!("every option is handled"),
        }
    }
    if extras.is_empty() {
        Ok(Parsed::Run(args))
    } else {
        Err(format!("unrecognized arguments: {}", extras.join(" ")))
    }
}

/// What pair takes from the machine: all of it replaced in the tests.
pub trait Host {
    /// One of the machine-id files.
    fn machine_id(&self, path: &str) -> std::io::Result<Vec<u8>>;
    /// `socket.gethostname()`
    fn hostname(&self) -> String;
    /// `secrets.token_bytes(n)`
    fn random(&mut self, n: usize) -> Result<Vec<u8>, String>;
    /// `int(time.time())`
    fn now(&self) -> i64;
    /// The address the default route leaves by, or empty.
    fn lan(&self) -> String;
    /// `pwd.getpwnam(name)`: uid and gid.
    fn user(&self, name: &str) -> Option<(u32, u32)>;
    /// `os.fchown`
    fn chown(&mut self, file: &File, uid: u32, gid: u32) -> std::io::Result<()>;
    /// `shutil.which('qrencode')`
    fn qrencode(&self) -> Option<String>;
    fn stdout_is_terminal(&self) -> bool;
    fn stdout(&mut self, text: &str);
    fn run_qrencode(&mut self, exe: &str, url: &str) -> std::io::Result<()>;
}

fn robot_id_from_seed(seed: &[u8]) -> String {
    let mut h = Sha256::new();
    h.update(b"mower-robot-id:");
    h.update(seed);
    let digest = h.finalize();
    let tail: String = digest[..6].iter().map(|b| ID_ALPHABET[usize::from(*b) % ID_ALPHABET.len()] as char).collect();
    format!("MW-{tail}")
}

fn machine_seed(host: &mut dyn Host) -> Result<Vec<u8>, Fail> {
    for path in MACHINE_ID_FILES {
        let Ok(bytes) = host.machine_id(path) else {
            continue;
        };
        // a read error goes on to the next file; bytes that are not UTF-8 do not
        let text = String::from_utf8(bytes).map_err(|_| Fail::failed(format!("{path}: not UTF-8")))?;
        let text = py::universal_newlines(&text);
        let value = py::strip(&text);
        if !value.is_empty() {
            return Ok(value.as_bytes().to_vec());
        }
    }
    let mut seed = host.hostname().into_bytes();
    seed.extend(host.random(8).map_err(Fail::failed)?);
    Ok(seed)
}

fn new_secret(host: &mut dyn Host) -> Result<String, Fail> {
    let bytes = host.random(20).map_err(Fail::failed)?;
    Ok(data_encoding::BASE32.encode(&bytes).trim_end_matches('=').to_string())
}

/// A Python value's truth.
fn truthy(v: Option<&Py>) -> bool {
    match v {
        None | Some(Py::None) => false,
        Some(Py::Bool(b)) => *b,
        Some(Py::Int(i)) => *i != 0,
        Some(Py::Float(f)) => *f != 0.0,
        Some(Py::Str(s)) => !s.is_empty(),
        Some(Py::List(l)) => !l.is_empty(),
        Some(Py::Dict(d)) => !d.is_empty(),
        Some(_) => true,
    }
}

/// `repr(v)`
fn py_repr(v: &Py) -> String {
    match v {
        Py::None => "None".to_string(),
        Py::Bool(true) => "True".to_string(),
        Py::Bool(false) => "False".to_string(),
        Py::Int(i) => i.to_string(),
        Py::Float(f) => repr::float(*f),
        Py::Str(s) => py::repr_str(s),
        Py::List(items) => format!("[{}]", items.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        Py::Dict(items) => {
            let inner: Vec<String> = items.iter().map(|(k, v)| format!("{}: {}", py::repr_str(k), py_repr(v))).collect();
            format!("{{{}}}", inner.join(", "))
        }
        other => other.isoformat().unwrap_or_default(),
    }
}

/// `str(v)`
fn py_str(v: &Py) -> String {
    match v {
        Py::Str(s) => s.clone(),
        other => py_repr(other),
    }
}

/// `load`: None for a file that is not there, not UTF-8, not JSON, not an object
/// or without a truthy `robot_id` and `secret`.
fn load(path: &str) -> Option<Py> {
    let dir = StateDir::open(&py::dirname(path)).ok()?;
    let mut bytes = Vec::new();
    dir.open_read(py::basename(path)).ok()?.read_to_end(&mut bytes).ok()?;
    let text = String::from_utf8(bytes).ok()?;
    let data = json::loads(&py::universal_newlines(&text)).ok()?;
    (matches!(data, Py::Dict(_)) && truthy(data.get("robot_id")) && truthy(data.get("secret"))).then_some(data)
}

/// `d[key] = value`: where the key already is, otherwise at the end.
fn set(d: &mut Py, key: &str, value: Py) {
    let Py::Dict(items) = d else {
        unreachable!("an identity is a dict");
    };
    match items.iter_mut().find(|(k, _)| k == key) {
        Some(slot) => slot.1 = value,
        None => items.push((key.to_string(), value)),
    }
}

fn os_fail(e: &std::io::Error, path: &str) -> Fail {
    Fail::failed(py::io_error(e, path))
}

/// `save`: the parent made, a new `<file>.tmp` written, 0600 from its creation,
/// its owner set on the open file when that is known and allowed, then renamed
/// over the file.
fn save(path: &str, data: &Py, owner: &str, host: &mut dyn Host) -> Result<(), Fail> {
    let dir = py::dirname(path);
    if dir.is_empty() {
        return Err(Fail::failed(py::os_error(libc::ENOENT, Some(""))));
    }
    std::fs::create_dir_all(&dir).map_err(|e| os_fail(&e, &dir))?;
    let name = py::basename(path);
    let tmp = format!("{name}.tmp");
    let text = dumps(data)? + "\n";
    let state = StateDir::open(&dir).map_err(|e| os_fail(&e, &dir))?;
    let mut f = state.create_fresh(&tmp, 0o600).map_err(|e| os_fail(&e, &tmp))?;
    f.write_all(text.as_bytes()).map_err(|e| os_fail(&e, &tmp))?;
    {
        use std::os::unix::fs::PermissionsExt;
        f.set_permissions(std::fs::Permissions::from_mode(0o600)).map_err(|e| os_fail(&e, &tmp))?;
    }
    if let Some((uid, gid)) = host.user(owner) {
        if let Err(e) = host.chown(&f, uid, gid) {
            // PermissionError is both of these; anything else is not swallowed
            if !matches!(e.raw_os_error(), Some(libc::EPERM | libc::EACCES)) {
                return Err(os_fail(&e, &tmp));
            }
        }
    }
    drop(f);
    state.rename(&tmp, name).map_err(|e| os_fail(&e, &tmp))
}

fn dumps(data: &Py) -> Result<String, Fail> {
    json::dumps(data, &JsonOptions { indent: Some(2), ..JsonOptions::default() }).map_err(|e| Fail::failed(e.to_string()))
}

/// `read_env(path).get('PAIR_RELAY_URL', '')`: comments and lines without `=`
/// skipped, the last one wins, quotes off the value. A file that cannot be read
/// has none; one that is not UTF-8 raises in Python, and fails here.
fn relay_from(path: &str) -> Result<String, Fail> {
    let Ok(bytes) = std::fs::read(path) else {
        return Ok(String::new());
    };
    let text = String::from_utf8(bytes).map_err(|_| Fail::failed(format!("{path}: not UTF-8")))?;
    let text = py::universal_newlines(&text);
    let mut relay = String::new();
    for line in text.split('\n') {
        let line = py::strip(line);
        if line.starts_with('#') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            if py::strip(k) == "PAIR_RELAY_URL" {
                relay = py::strip(v).trim_matches('"').trim_matches('\'').to_string();
            }
        }
    }
    Ok(relay)
}

/// `urllib.parse.quote_plus`: `~` as it stands and `*` escaped, which is not what
/// form-urlencoding does; the app's QR parser takes either, the golden says this.
fn quote_plus(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'_' | b'.' | b'-' | b'~' => out.push(b as char),
            b' ' => out.push('+'),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

fn pair_url(identity: &Py, relay: &str, lan: &str) -> String {
    let at = |k: &str| identity.get(k).map(py_str).unwrap_or_default();
    let mut params: Vec<(&str, String)> = vec![("v", "1".to_string()), ("id", at("robot_id")), ("s", at("secret"))];
    if truthy(identity.get("name")) {
        params.push(("n", at("name")));
    }
    if !relay.is_empty() {
        params.push(("h", relay.to_string()));
    }
    if !lan.is_empty() {
        params.push(("l", lan.to_string()));
    }
    let query: Vec<String> = params.iter().map(|(k, v)| format!("{k}={}", quote_plus(v))).collect();
    format!("{PAIR_BASE_URL}?{}", query.join("&"))
}

/// `main` after the argument parser.
pub fn run(args: &Args, host: &mut dyn Host) -> Result<(), Fail> {
    let path = py::join(&args.state_dir, "identity.json");
    let mut changed = false;
    let mut identity = match load(&path) {
        Some(d) => d,
        None => {
            let seed = machine_seed(host)?;
            let robot_id = robot_id_from_seed(&seed);
            let name = host.hostname();
            let secret = new_secret(host)?;
            changed = true;
            Py::dict([
                ("robot_id", Py::Str(robot_id)),
                ("name", Py::Str(name)),
                ("secret", Py::Str(secret)),
                ("created", Py::Int(host.now())),
            ])
        }
    };
    if args.rotate {
        let secret = new_secret(host)?;
        set(&mut identity, "secret", Py::Str(secret));
        set(&mut identity, "rotated", Py::Int(host.now()));
        changed = true;
    }
    if let Some(name) = args.name.as_deref().filter(|n| !n.is_empty()) {
        set(&mut identity, "name", Py::str(name));
        changed = true;
    }
    if changed {
        save(&path, &identity, &args.owner, host)?;
    }

    let relay = relay_from(&args.env)?;
    let lan = host.lan();
    let url = pair_url(&identity, &relay, &lan);

    if args.json {
        let mut doc = identity;
        set(&mut doc, "relay", Py::str(&relay));
        set(&mut doc, "lan", Py::str(&lan));
        set(&mut doc, "url", Py::str(&url));
        host.stdout(&(dumps(&doc)? + "\n"));
        return Ok(());
    }

    let mut text = String::new();
    let robot_id = identity.get("robot_id").map(py_str).unwrap_or_default();
    let name = identity.get("name").map(py_str).unwrap_or_default();
    text.push_str(&format!("robot_id : {robot_id}\n"));
    text.push_str(&format!("name     : {name}\n"));
    text.push_str(&format!("relay    : {}\n", if relay.is_empty() { "(none, LAN only)" } else { &relay }));
    text.push_str(&format!("lan      : {}:9090\n", if lan.is_empty() { "?" } else { &lan }));
    text.push_str(&format!("identity : {path}{}\n", if changed { "  (new secret)" } else { "" }));
    if args.rotate {
        text.push_str("secret rotated: paired phones must scan again; restart the stack: sudo systemctl restart mower\n");
    }
    text.push_str(&format!("\n{url}\n\n"));
    let mut qr = None;
    if !args.no_qr {
        match host.qrencode() {
            Some(exe) => qr = Some(exe),
            None => text.push_str("(install qrencode to print the QR code here)\n"),
        }
    }
    // Python's stdout is line-buffered on a terminal and block-buffered on a
    // pipe, and qrencode writes to the same descriptor: on a pipe the QR comes
    // out first. Kept: the golden was taken on a pipe, a person sees the terminal.
    let result = match qr {
        None => {
            host.stdout(&text);
            Ok(())
        }
        Some(exe) if host.stdout_is_terminal() => {
            host.stdout(&text);
            host.run_qrencode(&exe, &url)
        }
        Some(exe) => {
            let r = host.run_qrencode(&exe, &url);
            host.stdout(&text);
            r
        }
    };
    result.map_err(|e| Fail::failed(format!("qrencode: {e}")))
}

/// The real machine.
struct Machine;

impl Host for Machine {
    fn machine_id(&self, path: &str) -> std::io::Result<Vec<u8>> {
        std::fs::read(path)
    }

    fn hostname(&self) -> String {
        let mut buf = [0u8; 256];
        // SAFETY: the buffer is valid for its length, and one shorter is passed
        // so the name is terminated even where gethostname truncates.
        let r = unsafe { libc::gethostname(buf.as_mut_ptr().cast(), buf.len() - 1) };
        if r != 0 {
            return String::new();
        }
        let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
        String::from_utf8_lossy(&buf[..end]).into_owned()
    }

    fn random(&mut self, n: usize) -> Result<Vec<u8>, String> {
        use std::io::Read;
        let mut b = vec![0u8; n];
        std::fs::File::open("/dev/urandom").and_then(|mut f| f.read_exact(&mut b)).map_err(|e| format!("/dev/urandom: {e}"))?;
        Ok(b)
    }

    fn now(&self) -> i64 {
        mower_link::now_unix()
    }

    fn lan(&self) -> String {
        use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
        let local = || -> std::io::Result<String> {
            let s = UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0))?;
            s.connect(SocketAddrV4::new(Ipv4Addr::new(10, 255, 255, 255), 1))?;
            Ok(s.local_addr()?.ip().to_string())
        };
        local().unwrap_or_default()
    }

    fn user(&self, name: &str) -> Option<(u32, u32)> {
        let text = std::fs::read_to_string("/etc/passwd").ok()?;
        user_in_passwd(&text, name)
    }

    fn chown(&mut self, file: &File, uid: u32, gid: u32) -> std::io::Result<()> {
        std::os::unix::fs::fchown(file, Some(uid), Some(gid))
    }

    fn qrencode(&self) -> Option<String> {
        let path = match std::env::var("PATH") {
            Ok(p) => p,
            Err(_) => "/bin:/usr/bin".to_string(),
        };
        if path.is_empty() {
            return None;
        }
        path.split(':').map(|d| py::join(d, "qrencode")).find(|f| executable(f))
    }

    fn stdout_is_terminal(&self) -> bool {
        std::io::stdout().is_terminal()
    }

    fn stdout(&mut self, text: &str) {
        let mut out = std::io::stdout();
        let _ = out.write_all(text.as_bytes());
        let _ = out.flush();
    }

    fn run_qrencode(&mut self, exe: &str, url: &str) -> std::io::Result<()> {
        std::process::Command::new(exe).args(["-t", "ANSIUTF8", "-m", "2", url]).status().map(|_| ())
    }
}

/// `pwd.getpwnam` over the `files` database.
fn user_in_passwd(text: &str, name: &str) -> Option<(u32, u32)> {
    text.lines().find_map(|line| {
        let mut f = line.split(':');
        if f.next() != Some(name) || name.is_empty() {
            return None;
        }
        let (_, uid, gid) = (f.next()?, f.next()?, f.next()?);
        Some((uid.parse().ok()?, gid.parse().ok()?))
    })
}

/// `shutil._access_check`: there, executable by us, not a directory.
fn executable(path: &str) -> bool {
    let Ok(c) = std::ffi::CString::new(path) else {
        return false;
    };
    // SAFETY: a NUL-terminated path.
    let ok = unsafe { libc::access(c.as_ptr(), libc::X_OK) } == 0;
    ok && std::fs::metadata(path).is_ok_and(|m| !m.is_dir())
}

pub fn main(argv: &[String]) -> Result<(), Fail> {
    let parsed = parse_args(argv, std::env::var("MOWER_STATE_DIR").ok()).map_err(|msg| Fail { code: EXIT_USAGE, msg: format!("pair: {msg}") })?;
    match parsed {
        Parsed::Help => {
            print!("{HELP}");
            Ok(())
        }
        Parsed::Run(args) => run(&args, &mut Machine),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::fs::{MetadataExt, PermissionsExt};
    use std::path::Path;
    use std::time::{Duration, UNIX_EPOCH};

    use serde_json::Value;

    const STATE: &str = "/home/cat/.mower";
    const OLD_MTIME: u64 = 1_000_000_000;

    fn golden() -> Value {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../../deploy/test/golden/host/g5_pair.json");
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        serde_json::from_str(&text).unwrap()
    }

    fn latin1(s: &str) -> Vec<u8> {
        s.chars().map(|c| u8::try_from(c as u32).expect("a latin-1 character")).collect()
    }

    fn strs(v: &Value) -> Vec<String> {
        v.as_array().unwrap().iter().map(|x| x.as_str().unwrap().to_string()).collect()
    }

    /// The Python driver's replacements, from one golden case.
    struct Fake {
        machine: Vec<(String, Vec<u8>)>,
        hostname: String,
        random: Vec<Vec<u8>>,
        lan: String,
        passwd: Value,
        chown_errno: Option<i32>,
        qrencode: bool,
        out: String,
        draws: Vec<usize>,
        chowns: Vec<(String, u32, u32)>,
        qr: Vec<Vec<String>>,
    }

    impl Host for Fake {
        fn machine_id(&self, path: &str) -> std::io::Result<Vec<u8>> {
            self.machine
                .iter()
                .find(|(p, _)| p == path)
                .map(|(_, v)| v.clone())
                .ok_or_else(|| std::io::Error::from_raw_os_error(libc::ENOENT))
        }
        fn hostname(&self) -> String {
            self.hostname.clone()
        }
        fn random(&mut self, n: usize) -> Result<Vec<u8>, String> {
            self.draws.push(n);
            let b = self.random.remove(0);
            assert_eq!(b.len(), n);
            Ok(b)
        }
        fn now(&self) -> i64 {
            1_790_000_000
        }
        fn lan(&self) -> String {
            self.lan.clone()
        }
        fn user(&self, name: &str) -> Option<(u32, u32)> {
            let row = self.passwd.get(name)?;
            Some((row[0].as_u64()? as u32, row[1].as_u64()? as u32))
        }
        fn chown(&mut self, _file: &File, uid: u32, gid: u32) -> std::io::Result<()> {
            self.chowns.push(("fd".to_string(), uid, gid));
            match self.chown_errno {
                Some(e) => Err(std::io::Error::from_raw_os_error(e)),
                None => Ok(()),
            }
        }
        fn qrencode(&self) -> Option<String> {
            self.qrencode.then(|| "/fake/qrencode".to_string())
        }
        fn stdout_is_terminal(&self) -> bool {
            false // as the golden: captured on a pipe
        }
        fn stdout(&mut self, text: &str) {
            self.out.push_str(text);
        }
        fn run_qrencode(&mut self, exe: &str, url: &str) -> std::io::Result<()> {
            assert_eq!(exe, "/fake/qrencode");
            self.qr.push(["-t", "ANSIUTF8", "-m", "2", url].map(str::to_string).to_vec());
            self.out.push_str("<QR>\n");
            Ok(())
        }
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    /// As Python's run_pair: the files, the fakes, then the same comparisons.
    fn run_case(i: usize, case: &str, want: &Value) {
        let given = &want["given"];
        let dir = std::env::temp_dir().join(format!("mowerctl_pair_{}/{i}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let flag = |k: &str| given.get(k).and_then(Value::as_bool).unwrap_or(false);
        let state = dir.join("state");
        let path = state.join("identity.json");
        let other = dir.join("other-file");
        const OTHER_FILE: &str = "{\"robot_id\": \"MW-OTHER\", \"secret\": \"OTHER\", \"stale\": false}\n";
        std::fs::write(&other, OTHER_FILE).unwrap();
        let elsewhere = dir.join("elsewhere");
        if flag("state_is_file") {
            std::fs::write(&state, "a file").unwrap();
        } else if flag("state_is_link") {
            std::fs::create_dir(&elsewhere).unwrap();
            std::os::unix::fs::symlink(&elsewhere, &state).unwrap();
        } else if !flag("no_state_dir") {
            std::fs::create_dir(&state).unwrap();
        }
        if let Some(name) = given.get("link_at").and_then(Value::as_str) {
            std::os::unix::fs::symlink(&other, state.join(name)).unwrap();
        }
        if let Some(text) = given.get("tmp_before").and_then(Value::as_str) {
            std::fs::write(state.join("identity.json.tmp"), text).unwrap();
        }
        if let Some(text) = given.get("identity").and_then(Value::as_str) {
            std::fs::write(&path, latin1(text)).unwrap();
            let mode = given.get("identity_mode").and_then(Value::as_u64).unwrap_or(0o600) as u32;
            std::fs::set_permissions(&path, std::fs::Permissions::from_mode(mode)).unwrap();
            let f = std::fs::File::options().write(true).open(&path).unwrap();
            f.set_modified(UNIX_EPOCH + Duration::from_secs(OLD_MTIME)).unwrap();
        }
        if flag("identity_is_dir") {
            std::fs::create_dir(&path).unwrap();
        }
        let env_path = dir.join("env");
        if flag("env_is_dir") {
            std::fs::create_dir(&env_path).unwrap();
        } else if let Some(text) = given.get("env_file").and_then(Value::as_str) {
            std::fs::write(&env_path, latin1(text)).unwrap();
        }
        let machine: Vec<(String, Vec<u8>)> = match given.get("machine_ids") {
            None => vec![("/etc/machine-id".to_string(), b"9041cf96a09a4c2e964969d4befc76b3\n".to_vec())],
            Some(m) => m.as_object().unwrap().iter().map(|(k, v)| (k.clone(), latin1(v.as_str().unwrap()))).collect(),
        };
        let mut argv: Vec<String> = Vec::new();
        let suffix = given.get("state_arg_suffix").and_then(Value::as_str).unwrap_or("");
        if !flag("state_from_env") && !flag("state_env_empty") {
            argv.extend(["--state-dir".to_string(), format!("{}{suffix}", state.display())]);
        }
        argv.extend(["--env".to_string(), env_path.to_string_lossy().into_owned()]);
        argv.extend(strs(&given["argv"]));
        let env_state = if flag("state_from_env") {
            Some(state.to_string_lossy().into_owned())
        } else if flag("state_env_empty") {
            Some(String::new())
        } else {
            None
        };
        let random = match given.get("random") {
            None => vec![hex(&("00".repeat(19) + "01"))],
            Some(list) => strs(list).iter().map(|s| hex(s)).collect(),
        };
        let mut fake = Fake {
            machine,
            hostname: given.get("hostname").and_then(Value::as_str).unwrap_or("lubancat").to_string(),
            random,
            lan: given.get("lan").and_then(Value::as_str).unwrap_or("192.168.0.113").to_string(),
            passwd: given.get("passwd").cloned().unwrap_or_else(|| serde_json::json!({"cat": [1000, 1000]})),
            chown_errno: given.get("chown_error").and_then(Value::as_str).map(|e| match e {
                "EPERM" => libc::EPERM,
                "EACCES" => libc::EACCES,
                "EINVAL" => libc::EINVAL,
                other => panic!("{other}"),
            }),
            qrencode: flag("qrencode"),
            out: String::new(),
            draws: Vec::new(),
            chowns: Vec::new(),
            qr: Vec::new(),
        };
        let rc = match parse_args(&argv, env_state) {
            Err(_) => 2,
            Ok(Parsed::Help) => 0,
            Ok(Parsed::Run(args)) => match run(&args, &mut fake) {
                Ok(()) => 0,
                Err(f) => i64::from(f.code),
            },
        };
        let label = |s: &str| s.replace(&state.to_string_lossy().into_owned(), STATE);
        assert_eq!(rc, want["rc"].as_i64().unwrap(), "{case}: rc; stdout {:?}", fake.out);
        let want_stdout = strs(&want["stdout"]);
        let got_stdout: Vec<String> = label(&fake.out).strip_suffix('\n').map(|s| s.split('\n').map(str::to_string).collect()).unwrap_or_default();
        assert_eq!(got_stdout, want_stdout, "{case}: stdout");
        let is_file = path.is_file();
        let got_identity = is_file.then(|| std::fs::read(&path).unwrap().iter().map(|b| *b as char).collect::<String>());
        assert_eq!(got_identity.as_deref(), want["identity"].as_str(), "{case}: the file");
        if let Some(mode) = want["mode"].as_u64() {
            assert_eq!(u64::from(std::fs::metadata(&path).unwrap().mode() & 0o7777), mode, "{case}: its mode");
        }
        let rewritten = is_file && std::fs::metadata(&path).unwrap().mtime() != OLD_MTIME as i64;
        assert_eq!(rewritten, want["rewritten"].as_bool().unwrap(), "{case}: rewritten");
        let draws: Vec<u64> = fake.draws.iter().map(|n| *n as u64).collect();
        let want_draws: Vec<u64> = want["draws"].as_array().unwrap().iter().map(|d| d.as_u64().unwrap()).collect();
        assert_eq!(draws, want_draws, "{case}: the random draws");
        let chowns: Vec<(String, u64, u64)> = fake.chowns.iter().map(|(p, u, g)| (label(p), u64::from(*u), u64::from(*g))).collect();
        let want_chowns: Vec<(String, u64, u64)> = want["chown"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| (r[0].as_str().unwrap().to_string(), r[1].as_u64().unwrap(), r[2].as_u64().unwrap()))
            .collect();
        assert_eq!(chowns, want_chowns, "{case}: chown");
        assert_eq!(state.join("identity.json.tmp").is_file(), want["tmp_left"].as_bool().unwrap(), "{case}: the tmp");
        assert_eq!(std::fs::read_to_string(&other).unwrap(), OTHER_FILE, "{case}: what a link leads to");
        if let Some(empty) = want["elsewhere_empty"].as_bool() {
            assert_eq!(std::fs::read_dir(&elsewhere).unwrap().next().is_none(), empty, "{case}: where a link leads");
        }
        let qr: Vec<Vec<String>> = fake.qr.iter().map(|c| c.iter().map(|a| label(a)).collect()).collect();
        let want_qr: Vec<Vec<String>> = want["qrencode"].as_array().unwrap().iter().map(strs).collect();
        assert_eq!(qr, want_qr, "{case}: qrencode");
        if let Some(mode) = want["state_dir_mode"].as_u64() {
            assert_eq!(u64::from(std::fs::metadata(&state).unwrap().mode() & 0o7777), mode, "{case}: the new dir's mode");
        }
        let _ = std::fs::remove_dir_all(&dir);
        let _ = dir.parent().map(std::fs::remove_dir); // the last one out
    }

    #[test]
    fn every_case_does_what_python_did() {
        // SAFETY: umask has no failure mode; every test of this binary is fine with 022.
        unsafe {
            libc::umask(0o022);
        }
        let g = golden();
        let cases = g["cases"].as_object().unwrap();
        assert!(cases.len() > 100, "the golden's table: {}", cases.len());
        for (i, (case, want)) in cases.iter().enumerate() {
            run_case(i, case, want);
        }
    }

    #[test]
    fn the_defaults_are_the_tools() {
        let g = golden();
        let check = |want: &Value, args: &Args| {
            assert_eq!(want["state_dir"], args.state_dir.as_str());
            assert_eq!(want["owner"], args.owner.as_str());
            assert_eq!(want["env"], args.env.as_str());
            assert!(want["name"].is_null() && args.name.is_none());
            assert_eq!(want["rotate"], args.rotate);
            assert_eq!(want["json"], args.json);
            assert_eq!(want["no_qr"], args.no_qr);
        };
        let Parsed::Run(args) = parse_args(&[], None).unwrap() else { panic!() };
        check(&g["defaults"]["unset"], &args);
        let Parsed::Run(args) = parse_args(&[], Some("/some/where".to_string())).unwrap() else { panic!() };
        check(&g["defaults"]["from-env"], &args);
    }

    #[test]
    fn the_quoting_rule_is_not_form_urlencoding() {
        assert_eq!(quote_plus("a~b*c d&前"), "a~b%2Ac+d%26%E5%89%8D");
    }

    #[test]
    fn the_owner_comes_from_the_passwd_lines() {
        let passwd = "root:x:0:0:root:/root:/bin/bash\ncat:x:1000:1001::/home/cat:/bin/bash\nbad:x:nope:1\n";
        assert_eq!(user_in_passwd(passwd, "cat"), Some((1000, 1001)));
        assert_eq!(user_in_passwd(passwd, "dog"), None);
        assert_eq!(user_in_passwd(passwd, "bad"), None);
        assert_eq!(user_in_passwd(passwd, ""), None);
    }

    /// argparse's corners, as Python 3.12.3 and 3.12.6 took them; not in the
    /// golden, since these have moved between Python versions.
    #[test]
    fn corners_of_the_command_line() {
        let parse = |a: &[&str]| parse_args(&a.iter().map(|s| s.to_string()).collect::<Vec<_>>(), None);
        assert_eq!(parse(&["-hh"]), Ok(Parsed::Help));
        assert_eq!(parse(&["-hx"]), Ok(Parsed::Help));
        assert!(parse(&["-h-x"]).is_err());
        assert!(parse(&["-h=x"]).is_err());
        assert!(parse(&["--"]).is_err());
        assert!(parse(&["--rotate", "--"]).is_err());
        assert!(parse(&["--name", "--", "x"]).is_err());
        assert!(parse(&["--name", "--"]).is_err());
        assert!(parse(&["--", "x"]).is_err());
        assert!(parse(&["--name", "x", "--"]).is_err());
    }
}

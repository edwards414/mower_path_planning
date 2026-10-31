//! mower_ws_bridge: the app's WebSocket door.
//!
//! Replaces rosbridge_websocket + rosapi + rosbridge_auth_proxy with the
//! rosbridge v2 subset the clients use (see client.rs), the pairing gate on
//! the public listener (auth.rs, same X-Mower-* HMAC hand-shake) and an
//! unauthenticated loopback listener for the on-robot relay agent. One
//! subscription per topic feeds every client (a message is serialised
//! once), service clients are kept, and subscribe requests are answered in
//! parallel instead of one by one, so a phone that subscribes to twenty
//! topics sees its first heartbeat in milliseconds rather than seconds.
//! Each `call_service` runs in a task of its own, at most 16 per connection
//! (E3.3, client.rs); every other frame keeps its order.
//!
//! Two backends (hub.rs), chosen with `--backend`:
//! - `r2r`, the default: one r2r node of its own, with untyped endpoints.
//! - `bus` (step E3): mower_bus's erased access in the module's scope; the
//!   names only DDS has reach it through the bus bridge, which the manifest's
//!   `bridge` module plans for. Its r2r node stays, with no endpoint, so
//!   that the graph keeps it (E3 decision 10).
//!
//! Usage (rosbridge.launch.py):
//!   mower_ws_bridge --address 10.77.0.2 --port 9090 --loopback-port 9091
//!                   --policy ws_bridge.yaml [--state-dir DIR] [--max-size N]
//!                   [--backend r2r|bus]

pub mod auth;
mod bus_hub;
#[cfg(test)]
mod bus_tests;
mod client;
mod config;
mod entry;
#[cfg(test)]
mod golden;
mod hub;
mod r2r_hub;

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use futures::{SinkExt, StreamExt};
use mower_rs_common::{ModuleCtx, ModuleResult};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::handshake::server::{ErrorResponse, Request, Response};
use tokio_tungstenite::tungstenite::protocol::WebSocketConfig;
use tokio_tungstenite::tungstenite::Message;

use crate::auth::{Identity, NonceCache};
use crate::bus_hub::BusHub;
use crate::client::Client;
use crate::config::Policy;
use crate::hub::Hub;
use crate::r2r_hub::R2rHub;

/// What the bridge's hub talks to (`--backend`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// One r2r node and its spin thread: the bridge as it has always been.
    R2r,
    /// mower_bus, in the module's scope (step E3).
    Bus,
}

impl Backend {
    fn parse(value: &str) -> Result<Backend, String> {
        match value {
            "r2r" => Ok(Backend::R2r),
            "bus" => Ok(Backend::Bus),
            other => Err(format!("--backend takes r2r or bus, not {other:?}")),
        }
    }
}

/// The backend the module arguments `argv` ask for, as `run` reads them:
/// r2r unless `--backend bus` comes before the ROS block. For mower_rsd's
/// policy and the binary's `main`, which decide before `run` does; an error
/// for a value `run` would refuse.
pub fn backend_of(argv: &[String]) -> Result<Backend, String> {
    let mut backend = Backend::R2r;
    let mut it = argv.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--ros-args" => break,
            "--backend" => backend = Backend::parse(it.next().ok_or("--backend needs a value")?)?,
            _ => {}
        }
    }
    Ok(backend)
}

struct Args {
    address: String,
    port: u16,
    loopback_port: Option<u16>,
    policy: String,
    state_dir: String,
    max_size: usize,
    service_timeout_s: u64,
    backend: Backend,
}

fn parse_args(argv: &[String]) -> Result<Args, String> {
    let mut args = Args {
        address: "127.0.0.1".into(),
        port: 9090,
        loopback_port: Some(9091),
        policy: String::new(),
        state_dir: mower_rs_common::state_dir(),
        max_size: 64 * 1024 * 1024,
        service_timeout_s: 30,
        backend: Backend::R2r,
    };
    let mut it = argv.iter().cloned();
    while let Some(a) = it.next() {
        let mut value = |name: &str| it.next().ok_or_else(|| format!("{name} needs a value"));
        match a.as_str() {
            "--address" => args.address = value("--address")?,
            "--port" => args.port = value("--port")?.parse().map_err(|_| "--port".to_string())?,
            "--loopback-port" => {
                let v = value("--loopback-port")?;
                args.loopback_port = if v == "0" || v == "none" { None } else { Some(v.parse().map_err(|_| "--loopback-port".to_string())?) };
            }
            "--policy" => args.policy = value("--policy")?,
            "--state-dir" => args.state_dir = value("--state-dir")?,
            "--max-size" => args.max_size = value("--max-size")?.parse().map_err(|_| "--max-size".to_string())?,
            "--service-timeout" => args.service_timeout_s = value("--service-timeout")?.parse().map_err(|_| "--service-timeout".to_string())?,
            "--backend" => args.backend = Backend::parse(&value("--backend")?)?,
            "--ros-args" => break, // ros2 launch appends --ros-args ...; the node reads those itself
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if args.policy.is_empty() {
        return Err("--policy <ws_bridge.yaml> is required".into());
    }
    Ok(args)
}

struct Gate {
    identity: Option<Identity>,
    nonces: Mutex<NonceCache>,
}

impl Gate {
    /// (client id | "open", rejection reason)
    fn check(&self, headers: &[(String, String)]) -> Result<String, String> {
        let Some(identity) = &self.identity else { return Ok("open".into()) };
        let mut nonces = self.nonces.lock().unwrap();
        auth::verify_mac(identity, headers, &mut nonces, auth::now_unix())
    }
}

struct Shared {
    hub: Hub,
    policy: Arc<Policy>,
    logger: String,
    next_id: AtomicU64,
    max_size: usize,
    service_timeout: Duration,
}

async fn serve_listener(listener: TcpListener, shared: Arc<Shared>, gate: Option<Arc<Gate>>) {
    loop {
        let Ok((stream, peer)) = listener.accept().await else { continue };
        no_delay(&stream);
        let shared = shared.clone();
        let gate = gate.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(stream, peer, shared, gate).await {
                // closed connections and the app's plain-TCP reachability
                // probes (no WebSocket handshake) are routine
                if !e.contains("Connection reset") && !e.contains("closed") && !e.contains("Handshake not finished") {
                    eprintln!("[mower_ws_bridge] {peer}: {e}");
                }
            }
        });
    }
}

/// Frames go out at once instead of waiting for the peer's delayed ACK: they
/// are small and latency-bound (the joystick at 10 Hz, the command clock at
/// 20 Hz), and on 4G that wait is a round trip.
fn no_delay(stream: &TcpStream) {
    let _ = stream.set_nodelay(true);
}

async fn handle_connection(stream: TcpStream, peer: SocketAddr, shared: Arc<Shared>, gate: Option<Arc<Gate>>) -> Result<(), String> {
    let client_name = Arc::new(Mutex::new(String::from("local")));
    let callback = {
        let gate = gate.clone();
        let client_name = client_name.clone();
        let logger = shared.logger.clone();
        move |req: &Request, resp: Response| -> Result<Response, ErrorResponse> {
            let Some(gate) = gate.as_ref() else { return Ok(resp) };
            let headers: Vec<(String, String)> = req
                .headers()
                .iter()
                .map(|(k, v)| (k.as_str().to_ascii_lowercase(), v.to_str().unwrap_or("").to_string()))
                .collect();
            match gate.check(&headers) {
                Ok(name) => {
                    r2r::log_info!(&logger, "paired client {} from {}", name, peer);
                    *client_name.lock().unwrap() = name;
                    Ok(resp)
                }
                Err(why) => {
                    r2r::log_warn!(&logger, "rejected {}: {}", peer, why);
                    let mut err = ErrorResponse::new(Some(format!("pairing required: {why}\n")));
                    *err.status_mut() = tokio_tungstenite::tungstenite::http::StatusCode::UNAUTHORIZED;
                    err.headers_mut().insert("Content-Type", "text/plain".parse().unwrap());
                    Err(err)
                }
            }
        }
    };
    let config = WebSocketConfig { max_message_size: Some(shared.max_size), max_frame_size: Some(shared.max_size), ..Default::default() };
    let ws = tokio_tungstenite::accept_hdr_async_with_config(stream, callback, Some(config))
        .await
        .map_err(|e| format!("handshake: {e}"))?;
    let (mut sink, mut source) = ws.split();

    let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
    let (tx, mut rx) = mpsc::channel::<Arc<str>>(512);
    let peer_label = format!("{}@{}", client_name.lock().unwrap(), peer);
    let mut client = Client::new(id, tx, shared.hub.clone(), shared.policy.clone(), shared.logger.clone(), peer_label.clone(), shared.service_timeout);
    r2r::log_info!(&shared.logger, "client {} connected", peer_label);

    // Writer: queued JSON frames plus a keep-alive ping every 20 s.
    let writer = tokio::spawn(async move {
        let mut ping = tokio::time::interval(Duration::from_secs(20));
        ping.tick().await;
        loop {
            tokio::select! {
                item = rx.recv() => match item {
                    Some(payload) => {
                        if sink.send(Message::Text(payload.to_string())).await.is_err() {
                            break;
                        }
                    }
                    None => break,
                },
                _ = ping.tick() => {
                    if sink.send(Message::Ping(Vec::new())).await.is_err() {
                        break;
                    }
                }
            }
        }
        let _ = sink.close().await;
    });

    // One frame at a time, in order; a call to a server only starts its
    // task here (client.rs), so nothing behind it waits for its answer.
    let result = loop {
        match source.next().await {
            Some(Ok(Message::Text(text))) => client.on_text(&text).await,
            Some(Ok(Message::Binary(_))) => client.on_text("").await,
            Some(Ok(Message::Close(_))) | None => break Ok(()),
            Some(Ok(_)) => {} // ping/pong handled by tungstenite
            Some(Err(e)) => break Err(format!("read: {e}")),
        }
    };
    client.close();
    writer.abort();
    r2r::log_info!(&shared.logger, "client {} disconnected", peer_label);
    result
}

/// The WebSocket door. Runs as the `mower_ws_bridge` binary or as a module of
/// `mower_rsd`; the listeners, gate, policy and node name are the same, and
/// `m.args` carries what the launch file used to pass on the command line.
pub async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult {
    let args = parse_args(&m.args).map_err(|e| format!("{e}\nusage: mower_ws_bridge --policy FILE [--address A] [--port P] [--loopback-port P|none] [--state-dir D] [--max-size N] [--service-timeout S] [--backend r2r|bus]"))?;
    let policy = Arc::new(Policy::from_yaml(&std::fs::read_to_string(&args.policy).map_err(|e| format!("{}: {e}", args.policy))?)?);

    let node = r2r::Node::create(ctx, &m.node_name, &m.namespace)?;
    let logger = node.logger().to_string();
    match args.backend {
        Backend::R2r => {
            let hub = Hub::R2r(R2rHub::start(node)?);
            serve(args, policy, hub, logger, m).await
        }
        Backend::Bus => {
            // The node stays for the graph's sake, with no endpoint and no
            // spin: graph_snapshot and the DDS tools keep seeing it.
            let hub = Hub::Bus(BusHub::new(m.bus.clone(), m.clock.clone(), bus_hub::manifest_names(&m)?, &logger));
            let served = serve(args, policy, hub, logger, m).await;
            drop(node);
            served
        }
    }
}

/// The listener tasks. With the bus backend they are aborted however
/// `serve` ends: a fatal bind error of the public listener used to leave the
/// loopback ones running, holding their ports, so that every restart failed
/// to bind them. The r2r backend keeps that behaviour until it goes.
struct Listeners {
    tasks: Vec<tokio::task::JoinHandle<()>>,
    abort_on_drop: bool,
}

impl Drop for Listeners {
    fn drop(&mut self) {
        if self.abort_on_drop {
            for task in &self.tasks {
                task.abort();
            }
        }
    }
}

/// The listeners, the gate and the clients, on `hub`, until the shutdown.
async fn serve(args: Args, policy: Arc<Policy>, hub: Hub, logger: String, m: ModuleCtx) -> ModuleResult {
    let identity = Identity::load(&args.state_dir)?;
    match &identity {
        Some(id) => r2r::log_info!(&logger, "pairing gate for {}", id.robot_id),
        None => r2r::log_warn!(&logger, "no identity.json in {}: gate runs OPEN (development mode)", args.state_dir),
    }
    let gate = Arc::new(Gate { identity, nonces: Mutex::new(NonceCache::default()) });
    let shared = Arc::new(Shared {
        hub,
        policy,
        logger: logger.clone(),
        next_id: AtomicU64::new(1),
        max_size: args.max_size,
        service_timeout: Duration::from_secs(args.service_timeout_s),
    });

    let mut tasks = Listeners { tasks: Vec::new(), abort_on_drop: args.backend == Backend::Bus };
    // The same gate on loopback for the relay agent: `address` is the LAN
    // (or VPN) IP, which vanishes with its uplink, while relayed sessions
    // arrive over whichever uplink is left (4G). 0.0.0.0 already covers it.
    let lan_address = !matches!(args.address.as_str(), "127.0.0.1" | "0.0.0.0" | "::" | "localhost");
    if lan_address {
        let local_gate = TcpListener::bind(("127.0.0.1", args.port)).await.map_err(|e| format!("bind 127.0.0.1:{}: {e}", args.port))?;
        r2r::log_info!(&logger, "listening on ws://127.0.0.1:{} (pairing gate, relay agent)", args.port);
        tasks.tasks.push(tokio::spawn(serve_listener(local_gate, shared.clone(), Some(gate.clone()))));
    }
    if let Some(port) = args.loopback_port {
        if !(args.address == "127.0.0.1" && port == args.port) {
            let local = TcpListener::bind(("127.0.0.1", port)).await.map_err(|e| format!("bind 127.0.0.1:{port}: {e}"))?;
            r2r::log_info!(&logger, "listening on ws://127.0.0.1:{} (loopback, no gate)", port);
            tasks.tasks.push(tokio::spawn(serve_listener(local, shared.clone(), None)));
        }
    }
    // A LAN address may not exist yet (booted with the cable out, on 4G):
    // keep retrying instead of dying, the loopback gate above already serves
    // the relay. Any other bind error (port taken, bad address) is fatal.
    let mut attempts = 0u32;
    let public = loop {
        if m.shutdown.is_triggered() {
            shared.hub.shutdown();
            return Ok(());
        }
        match TcpListener::bind((args.address.as_str(), args.port)).await {
            Ok(l) => break l,
            Err(e) if lan_address && e.kind() == std::io::ErrorKind::AddrNotAvailable => {
                if attempts % 12 == 0 {
                    r2r::log_warn!(&logger, "bind {}:{}: {e}; retrying every 5 s", args.address, args.port);
                }
                attempts += 1;
                tokio::select! {
                    _ = tokio::time::sleep(Duration::from_secs(5)) => {}
                    _ = m.shutdown.wait() => {}
                }
            }
            Err(e) => return Err(format!("bind {}:{}: {e}", args.address, args.port).into()),
        }
    };
    r2r::log_info!(&logger, "listening on ws://{}:{} (pairing gate)", args.address, args.port);
    tasks.tasks.push(tokio::spawn(serve_listener(public, shared.clone(), Some(gate.clone()))));

    m.shutdown.wait().await;
    r2r::log_info!(&logger, "mower_ws_bridge: shutting down");
    for t in std::mem::take(&mut tasks.tasks) {
        t.abort();
    }
    shared.hub.shutdown();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Result<Args, String> {
        parse_args(&s.split_whitespace().map(str::to_string).collect::<Vec<_>>())
    }

    fn argv(s: &str) -> Vec<String> {
        s.split_whitespace().map(str::to_string).collect()
    }

    #[test]
    fn accepted_connections_send_without_delay() {
        let rt = tokio::runtime::Builder::new_current_thread().enable_io().build().unwrap();
        rt.block_on(async {
            let listener = TcpListener::bind(("127.0.0.1", 0)).await.unwrap();
            let addr = listener.local_addr().unwrap();
            let _client = TcpStream::connect(addr).await.unwrap();
            let (stream, _) = listener.accept().await.unwrap();
            assert!(!stream.nodelay().unwrap(), "the default this undoes");
            no_delay(&stream);
            assert!(stream.nodelay().unwrap());
        });
    }

    #[test]
    fn the_backend_is_r2r_unless_told_otherwise() {
        assert_eq!(args("--policy p.yaml").unwrap().backend, Backend::R2r);
        assert_eq!(args("--policy p.yaml --backend r2r").unwrap().backend, Backend::R2r);
        assert_eq!(args("--policy p.yaml --backend bus").unwrap().backend, Backend::Bus);
        assert_eq!(args("--policy p.yaml --backend dds").err().as_deref(), Some("--backend takes r2r or bus, not \"dds\""));
        assert!(args("--policy p.yaml --backend").is_err());
        // launch's ROS block ends what the bridge reads
        assert_eq!(args("--policy p.yaml --ros-args --backend nonsense").unwrap().backend, Backend::R2r);
        // as mower_rsd and main read it, before run does
        for (line, backend) in [
            ("--policy p.yaml", Ok(Backend::R2r)),
            ("--address 10.77.0.2 --port 9090 --policy p.yaml --backend bus", Ok(Backend::Bus)),
            ("--backend r2r", Ok(Backend::R2r)),
            ("--policy p.yaml --ros-args --backend bus", Ok(Backend::R2r)),
            ("--backend dds", Err("--backend takes r2r or bus, not \"dds\"".to_string())),
            ("--backend", Err("--backend needs a value".to_string())),
        ] {
            assert_eq!(backend_of(&argv(line)), backend, "{line}");
        }
    }
}

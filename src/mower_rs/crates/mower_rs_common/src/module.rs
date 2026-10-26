//! The contract every mower_rs node implements so it can run either as its
//! own process or as one module of the `mower_rsd` daemon.
//!
//! A module is `async fn run(ctx: r2r::Context, m: ModuleCtx) -> ModuleResult`.
//! It creates its own `r2r::Node` (keeping its node name, namespace, topics,
//! services and parameters), does its work, returns when `m.shutdown` fires
//! and stops its node threads *before* returning: rmw's destructors abort the
//! process when a thread is still inside `rcl_wait`.
//!
//! `r2r::Context::create()` is a process-wide `OnceLock`, so every node in the
//! process — whether one module or thirteen — shares a single DDS participant,
//! and `Node::create` picks up the parameter overrides of the shared context
//! that are addressed to *its* node name (`--params-file` sections keyed by
//! node name, `/**`, or `-p <node>:<name>:=<value>`). Remapping works the same
//! way: rcl applies `-r <node>:<from>:=<to>` rules per node, so the daemon
//! reproduces launch remappings without any code in the modules.
//!
//! Step E1 of docs/ROS_FREE_PLAN.md adds the in-process bus: `m.bus` is the
//! instance's `mower_bus::Scope`, the bus's counterpart of its `r2r::Node`,
//! and `m.clock` the clock to read time from. No module uses either yet; E2
//! moves their IO from r2r onto them.

use std::future::Future;
use std::pin::Pin;

use tokio::sync::watch;

/// Error type for a module: `Send + Sync` so it can cross a task boundary.
pub type BoxError = Box<dyn std::error::Error + Send + Sync + 'static>;

/// What a module returns. `Ok(())` means "asked to stop and stopped cleanly";
/// `Err` means the module failed and the whole process should exit non-zero
/// so that the launch `respawn` brings it back.
pub type ModuleResult = Result<(), BoxError>;

/// A cooperative, cloneable stop signal shared by every module in a process.
///
/// One `watch` channel rather than a signal handler per module: the daemon's
/// single SIGINT/SIGTERM handler (or a module failure) trips it once and all
/// modules unwind together.
#[derive(Clone, Debug)]
pub struct Shutdown {
    tx: watch::Sender<bool>,
}

impl Default for Shutdown {
    fn default() -> Self {
        Self::new()
    }
}

impl Shutdown {
    pub fn new() -> Self {
        Shutdown { tx: watch::channel(false).0 }
    }

    /// Ask every holder to stop. Idempotent.
    ///
    /// `send_replace`, not `send`: `send` fails and leaves the value untouched
    /// when no one is subscribed at that instant, which would silently lose a
    /// shutdown raised between two `wait()` calls.
    pub fn trigger(&self) {
        self.tx.send_replace(true);
    }

    pub fn is_triggered(&self) -> bool {
        *self.tx.borrow()
    }

    /// Resolves as soon as [`Shutdown::trigger`] has been called (including
    /// before this was awaited).
    pub async fn wait(&self) {
        let mut rx = self.tx.subscribe();
        if *rx.borrow_and_update() {
            return;
        }
        let _ = rx.changed().await;
    }

    /// Trip the signal on SIGINT / SIGTERM. Returns a task handle; dropping it
    /// is fine, the runtime keeps it until shutdown.
    pub fn install_signal_handlers(&self) -> Result<tokio::task::JoinHandle<()>, std::io::Error> {
        let mut sigterm = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        let me = self.clone();
        Ok(tokio::spawn(async move {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {}
                _ = sigterm.recv() => {}
            }
            me.trigger();
        }))
    }
}

/// Everything a module needs that differs between "own process" and "one of
/// many in `mower_rsd`".
#[derive(Clone, Debug)]
pub struct ModuleCtx {
    /// ROS node name. Standalone binaries pass their historical name and let
    /// launch rename them with `-r __node:=`; the daemon passes the final name
    /// directly (a bare `__node:=` rule would rename *every* node in the
    /// process).
    pub node_name: String,
    /// ROS namespace, "" for the default.
    pub namespace: String,
    /// Module-specific argv (what `mower_ws_bridge` and `mower_agent` parse).
    /// Standalone this is `std::env::args().skip(1)`; in the daemon it is the
    /// `--module-args <module>=...` list.
    pub args: Vec<String>,
    pub shutdown: Shutdown,
    /// This instance's view of the in-process bus (mower_bus README,
    /// "Model"). Standalone it is a bus of its own; under `mower_rsd` it is
    /// the instance's scope of the process's one bus, which the supervisor
    /// closes when the instance ends.
    ///
    /// No module uses it yet: their topics and services stay on r2r until
    /// step E2 moves them here.
    pub bus: mower_bus::Scope,
    /// Where modules are to read time from, so that tests and replay can
    /// drive it by hand. Unused until E2, like `bus`: modules still call
    /// `tokio::time`, `Instant::now()` and `SystemTime::now()`.
    pub clock: mower_bus::Clock,
}

impl ModuleCtx {
    /// A context on a private bus and the system clock: what a standalone
    /// binary gets from [`module_main`], where the module is alone on its
    /// bus. `mower_rsd` hands each instance its scope with
    /// [`ModuleCtx::with_bus`].
    pub fn new(node_name: &str, shutdown: Shutdown) -> Self {
        ModuleCtx {
            node_name: node_name.to_string(),
            namespace: String::new(),
            args: Vec::new(),
            shutdown,
            bus: mower_bus::Bus::new().scope(node_name),
            clock: mower_bus::Clock::system(),
        }
    }

    pub fn with_args(mut self, args: Vec<String>) -> Self {
        self.args = args;
        self
    }

    /// Run on `scope` instead of the private bus (`mower_rsd`: a fresh scope
    /// of the process's bus for every start of an instance).
    pub fn with_bus(mut self, scope: mower_bus::Scope) -> Self {
        self.bus = scope;
        self
    }

    /// Read time from `clock` instead of the system clock (tests, replay).
    pub fn with_clock(mut self, clock: mower_bus::Clock) -> Self {
        self.clock = clock;
        self
    }
}

/// A module entry point erased to a function pointer so the daemon can keep a
/// table of them.
pub type ModuleRun =
    fn(r2r::Context, ModuleCtx) -> Pin<Box<dyn Future<Output = ModuleResult> + Send>>;

/// The body of every standalone `main.rs`: one runtime, the shared context,
/// signal handlers, then the module.
///
/// Returns 1 on failure (so launch's `respawn` restarts the process) and 0
/// after a clean stop.
pub fn module_main<F, Fut>(node_name: &str, worker_threads: usize, run: F) -> std::process::ExitCode
where
    F: FnOnce(r2r::Context, ModuleCtx) -> Fut,
    Fut: Future<Output = ModuleResult>,
{
    let runtime = match tokio::runtime::Builder::new_multi_thread()
        .worker_threads(worker_threads)
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("[{node_name}] tokio runtime: {e}");
            return std::process::ExitCode::FAILURE;
        }
    };
    let result = runtime.block_on(async move {
        let ctx = r2r::Context::create()?;
        let shutdown = Shutdown::new();
        shutdown.install_signal_handlers()?;
        let m = ModuleCtx::new(node_name, shutdown)
            .with_args(std::env::args().skip(1).collect());
        run(ctx, m).await
    });
    match result {
        Ok(()) => std::process::ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("[{node_name}] {e}");
            std::process::ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_shutdown_with_nobody_listening_is_still_remembered() {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let s = Shutdown::new();
            s.trigger();
            assert!(s.is_triggered());
            tokio::time::timeout(std::time::Duration::from_secs(1), s.wait()).await.unwrap();
        });
    }

    #[test]
    fn shutdown_is_observable_before_and_after_the_wait() {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let s = Shutdown::new();
            assert!(!s.is_triggered());
            let waiter = { let s = s.clone(); tokio::spawn(async move { s.wait().await }) };
            tokio::task::yield_now().await;
            s.trigger();
            waiter.await.unwrap();
            assert!(s.is_triggered());
            // already-triggered: wait() must return at once
            tokio::time::timeout(std::time::Duration::from_secs(1), s.wait()).await.unwrap();
        });
    }

    #[test]
    fn a_context_is_alone_on_a_bus_of_its_own_until_it_is_given_a_scope() {
        let m = ModuleCtx::new("imu", Shutdown::new());
        assert_eq!(m.bus.name(), "imu");
        assert!(!m.bus.is_closed());
        assert!(!m.clock.is_manual());
        // what mower_rsd does: the context holds the very scope it is given
        let given = mower_bus::Bus::new().scope("imu");
        let m = m.with_bus(given.clone());
        given.close();
        assert!(m.bus.is_closed());
    }
}

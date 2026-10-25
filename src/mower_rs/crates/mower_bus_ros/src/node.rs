//! The bridge's r2r node while it runs: spun on a thread of its own, and
//! changed only there.
//!
//! r2r's `Node` is not `Sync`, and creating or destroying an entity must not
//! race with `spin_once` (vendor/r2r/src/publishers.rs: "keep pub/sub
//! management and polling contained to one thread and send out
//! publishers"). So the thread that spins also runs every change a forwarder
//! asks for, between two spins. Publishing, requests, responses and
//! availability probes need no node and run in the forwarders.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::time::Duration;

use futures::channel::oneshot;

/// How long one `spin_once` waits for DDS, as every mower_rs module does. It
/// bounds how long a change asked for by a forwarder waits, and how long
/// stopping takes.
pub(crate) const SPIN_TIMEOUT: Duration = Duration::from_millis(20);

type Command = Box<dyn FnOnce(&mut r2r::Node) + Send>;

/// The forwarders' way to the node.
#[derive(Clone)]
pub(crate) struct NodeHandle {
    commands: mpsc::Sender<Command>,
}

/// The spin thread's end of the queue.
pub(crate) struct Commands {
    queue: mpsc::Receiver<Command>,
}

pub(crate) fn queue() -> (NodeHandle, Commands) {
    let (commands, queue) = mpsc::channel();
    (NodeHandle { commands }, Commands { queue })
}

impl NodeHandle {
    /// Run `f` on the node, on the spin thread, and return what it returns:
    /// within one spin period while the bridge runs, `None` once it stops.
    pub(crate) async fn call<R: Send + 'static>(&self, f: impl FnOnce(&mut r2r::Node) -> R + Send + 'static) -> Option<R> {
        let (reply, answer) = oneshot::channel();
        self.commands
            .send(Box::new(move |node| {
                let _ = reply.send(f(node));
            }))
            .ok()?;
        answer.await.ok()
    }
}

/// The spin thread: spin, run what the forwarders asked for, until told to
/// stop. The node is dropped here, after its last `spin_once`, so no thread
/// is in `rcl_wait` on it any more (mower_rs_common/src/module.rs: rmw's
/// destructors abort the process otherwise). Commands still queued then
/// are dropped, which their callers see as `None`.
pub(crate) fn spin(mut node: r2r::Node, commands: Commands, running: Arc<AtomicBool>) {
    while running.load(Ordering::Acquire) {
        node.spin_once(SPIN_TIMEOUT);
        while let Ok(command) = commands.queue.try_recv() {
            command(&mut node);
        }
    }
    drop(node);
}

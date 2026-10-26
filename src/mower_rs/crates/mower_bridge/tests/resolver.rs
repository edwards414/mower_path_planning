//! The resolver oracle: rcl names the manifest's endpoints itself.
//!
//! In one r2r context with robot.launch.py's `-r` rules, every instance's
//! node is made under its production name, and every endpoint under the name
//! its code passes. The graph then shows the name rcl resolved it to, which
//! must be the manifest's production name and what
//! `mower_rs_common::names::Remaps::resolve` gives (docs/BUS_TOPICS.md 5.4).
//!
//! - **Topics:** an untyped writer of the endpoint's type; the graph lists it,
//!   with its node, under the resolved name. A reader resolves the same way.
//! - **Services:** an untyped client. A client's request writer shows on the
//!   graph as `rq<name>Request` (rmw_cyclonedds's mangling), which a probe
//!   checks first.
//!
//! rcl reads the rules from argv when the context is made, and libtest
//! refuses arguments it does not know, so this test has a `main` of its own:
//! it runs itself again with the rules in argv, on ROS domain 79 and
//! rmw_cyclonedds_cpp, and that run does the work.

use std::collections::BTreeSet;
use std::process::{Command, ExitCode};
use std::time::{Duration, Instant};

use mower_bridge::types::Iface;
use mower_bridge::{production_remaps, MODULES, PRODUCTION_REMAPS};
use mower_rs_common::Remaps;
use r2r::QosProfile;

const NAME: &str = "resolver_oracle";
/// Set in the run that does the work.
const CHILD: &str = "MOWER_BRIDGE_RESOLVER_ORACLE";
/// Discovery is polled for, within this. A pass never waits this long.
const WAIT: Duration = Duration::from_secs(20);

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if std::env::var_os(CHILD).is_some() {
        return oracle();
    }
    if args.iter().any(|a| a == "--list") {
        println!("{NAME}: test");
        return ExitCode::SUCCESS;
    }
    // a filter that does not match skips it, as libtest would
    let filters: Vec<&String> = args.iter().filter(|a| !a.starts_with('-')).collect();
    if !filters.is_empty() && !filters.iter().any(|f| NAME.contains(f.as_str())) {
        println!("running 0 tests");
        return ExitCode::SUCCESS;
    }
    assert!(
        rmw_installed("rmw_cyclonedds_cpp"),
        "this test runs on the robot's RMW: apt-get install ros-jazzy-rmw-cyclonedds-cpp"
    );
    let mut command = Command::new(std::env::current_exe().expect("the test binary"));
    command.arg("--ros-args");
    for rule in PRODUCTION_REMAPS {
        command.args(["-r", rule]);
    }
    command.env(CHILD, "1").env("ROS_DOMAIN_ID", "79").env("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp");
    println!("running 1 test");
    let status = command.status().expect("the oracle runs");
    if status.success() {
        println!("test {NAME} ... ok");
        ExitCode::SUCCESS
    } else {
        println!("test {NAME} ... FAILED ({status})");
        ExitCode::FAILURE
    }
}

fn rmw_installed(rmw: &str) -> bool {
    let prefixes = std::env::var("AMENT_PREFIX_PATH").unwrap_or_default();
    prefixes.split(':').any(|prefix| std::path::Path::new(prefix).join(format!("lib/lib{rmw}.so")).exists())
}

/// One endpoint to find on the graph.
struct Probe {
    node: &'static str,
    topic: bool,
    code: &'static str,
    /// What rcl must make of `code`.
    expected: String,
}

/// The name a writer of `probe` has on the graph, and whether it is a DDS
/// (mangled) name.
fn graph_name(probe: &Probe) -> (String, bool) {
    if probe.topic {
        (probe.expected.clone(), false)
    } else {
        (format!("rq{}Request", probe.expected), true)
    }
}

fn shown(graph: &r2r::Node, node: &str, name: &str, mangled: bool) -> bool {
    graph.get_publishers_info_by_topic(name, mangled).is_ok_and(|infos| infos.iter().any(|i| i.node_name == node))
}

fn oracle() -> ExitCode {
    let ctx = r2r::Context::create().expect("an rcl context (source /opt/ros/jazzy/setup.bash)");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let remaps = Remaps::from_args(&argv).expect("the rules parse");
    assert_eq!(remaps, production_remaps(), "argv carries the production rules");

    let mut failures = Vec::new();
    let mut probes = Vec::new();
    // Every node and endpoint lives until the process ends.
    let mut nodes = Vec::new();
    let mut publishers = Vec::new();
    let mut clients = Vec::new();
    for module in MODULES {
        for instance in module.instances {
            let mut node = r2r::Node::create(ctx.clone(), instance.node, "").expect("a node");
            assert_eq!(node.name().unwrap(), instance.node, "no rule renames a node");
            let mut made = BTreeSet::new();
            for e in instance.endpoints {
                // one probe per name in each namespace: a reader and a writer
                // of one name resolve alike
                if !made.insert((e.kind.is_topic(), e.code)) {
                    continue;
                }
                let expected = remaps.resolve(e.kind.name_kind(), instance.node, "", e.code).expect("a valid name");
                if expected != e.name {
                    failures.push(format!("{} {}: Remaps::resolve gives {expected}, the manifest {}", instance.node, e.code, e.name));
                }
                match e.iface {
                    Iface::Topic(t, profile) => {
                        let ros_type = t.ros_type();
                        match node.create_publisher_untyped(e.code, &ros_type, profile.ros()) {
                            Ok(p) => publishers.push(p),
                            Err(err) => failures.push(format!("{} {} ({ros_type}): {err}", instance.node, e.code)),
                        }
                    }
                    Iface::Service(s) => {
                        let ros_type = s.ros_type();
                        match node.create_client_untyped(e.code, &ros_type, QosProfile::services_default()) {
                            Ok(c) => clients.push(c),
                            Err(err) => failures.push(format!("{} {} ({ros_type}): {err}", instance.node, e.code)),
                        }
                    }
                }
                probes.push(Probe { node: instance.node, topic: e.kind.is_topic(), code: e.code, expected });
            }
            nodes.push(node);
        }
    }

    // The service mangling, before it is relied on.
    let graph = r2r::Node::create(ctx.clone(), "resolver_oracle", "").expect("a node");
    let mut calibration = r2r::Node::create(ctx, "resolver_calibration", "").expect("a node");
    let _calibration =
        calibration.create_client_untyped("/resolver_oracle/calibrate", "std_srvs/srv/Trigger", QosProfile::services_default());
    let deadline = Instant::now() + WAIT;
    while !shown(&graph, "resolver_calibration", "rq/resolver_oracle/calibrateRequest", true) {
        assert!(Instant::now() < deadline, "the graph shows no client as an rq<name>Request writer");
        std::thread::sleep(Duration::from_millis(20));
    }

    let total = probes.len();
    let deadline = Instant::now() + WAIT;
    while !probes.is_empty() && Instant::now() < deadline {
        probes.retain(|p| {
            let (name, mangled) = graph_name(p);
            !shown(&graph, p.node, &name, mangled)
        });
        std::thread::sleep(Duration::from_millis(50));
    }
    for p in &probes {
        let kind = if p.topic { "topic" } else { "service" };
        failures.push(format!("{} {kind} {}: rcl did not resolve it to {}", p.node, p.code, p.expected));
    }
    drop((publishers, clients, nodes));
    if failures.is_empty() {
        println!("{total} names: rcl resolves every one as the manifest and Remaps::resolve do");
        ExitCode::SUCCESS
    } else {
        eprintln!("{} of {total} names differ:\n{}", failures.len(), failures.join("\n"));
        ExitCode::FAILURE
    }
}

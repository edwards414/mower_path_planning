#!/usr/bin/env python3
"""Drive one base implementation through a scripted run and record it.

Used twice by `base_compare.py`: once against `ros2_control_node` +
`mower_hardware` + `diff_controller` + `joint_state_broadcaster`, once
against the Rust `mower_base` node, with the same `fake_base.py` behind
both. Publishes the same `/drivetrain_guarded_cmd_vel` script and the same
side-channel bursts, and records `/odom`, `/joint_states`, `/tf` and
`/mower_base/telemetry`.

    ros2 run ... base_harness.py --out /tmp/run_a.json --label ros2_control

`--scenario pull --mute-file F` runs PULL_SCRIPT instead: the stick is held
while the harness makes `fake_base.py` go deaf and mute for two seconds (a
pulled UART lead) and back, then released and pushed again. The C++ chain
resumes the held command the moment the lead is back; `mower_base` holds
the wheels until the release (its arm latch).

`--scenario pullpush --mute-file F`: the lead comes out while nothing is
commanded, the stick is pushed while it is still out, and the lead is
re-seated with the stick still pushed (PULLPUSH_SCRIPT). `mower_base` must
not re-arm during the outage, so it stays at 0/0 until the release.

`--scenario live`: a live 0.30 m/s stream at nav2's 20 Hz that is already
running when the driver starts (`base_ab.sh` starts this harness first), as
after a restart under nav2 or teleop. No discovery wait: the stream starts
at once. The driver hears nothing until DDS has matched it with this writer,
and that quiet stretch must not re-arm it (LIVE_SCRIPT).

Nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# The command script, as a *continuous* function of time. Both runs are
# driven by wall clock, so their 25 Hz control loops sample it at a slightly
# different phase (up to one 40 ms cycle apart). A step command would turn
# that phase into a v * 40 ms = 16 mm position difference; a script that is
# C0-continuous and ramps at 0.1 m/s^2 / 0.2 rad/s^2 turns it into
# a * h^2 / 2 per corner instead, which is what keeps the odometry
# comparison meaningful. The command is also back at zero before every
# silent stretch, so the cmd_vel timeout cannot move the robot either.
#
# (t_from, t_to, (lin_from, lin_to), (ang_from, ang_to)) — `None` = publish
# nothing at all, which is what exercises the timeout.
CMD_SCRIPT = [
    (0.0, 1.5, None, None),                    # settle: firmware info, feedback
    (1.5, 5.5, (0.0, 0.40), (0.0, 0.0)),       # accelerate
    (5.5, 7.5, (0.40, 0.40), (0.0, 0.40)),     # cruise, start turning
    (7.5, 9.5, (0.40, 0.40), (0.40, 0.0)),     # stop turning
    (9.5, 13.5, (0.40, 0.0), (0.0, 0.0)),      # decelerate to a stop
    (13.5, 14.5, (0.0, 0.0), (0.0, 0.0)),      # explicit zero
    (14.5, 16.5, None, None),                  # cmd_vel timeout
    (16.5, 18.5, None, None),                  # wheel_override burst (below)
    (18.5, 20.5, (0.0, 0.0), (0.0, 0.0)),      # explicit zero again
    # A quiet tail so the per-process CPU window is a full 30 s.
    (20.5, 32.0, (0.0, 0.0), (0.0, 0.0)),
]
CMD_RATE_HZ = 200.0
RUN_SECONDS = 32.0

# The cable-pull scenario: the lead is out from MUTE[0] to MUTE[1] while the
# stick stays at 0.30 m/s, released at 9.0 s and pushed again at 9.5 s.
PULL_SCRIPT = [
    (0.0, 1.5, None, None),
    (1.5, 3.5, (0.0, 0.30), (0.0, 0.0)),
    (3.5, 9.0, (0.30, 0.30), (0.0, 0.0)),      # held through the pull
    (9.0, 9.5, (0.0, 0.0), (0.0, 0.0)),        # released
    (9.5, 11.5, (0.0, 0.30), (0.0, 0.0)),      # pushed again
    (11.5, 12.5, (0.30, 0.0), (0.0, 0.0)),
    (12.5, 14.0, (0.0, 0.0), (0.0, 0.0)),
]
PULL_MUTE = (4.0, 6.0)
PULL_SECONDS = 14.0

# The lead comes out at 4.0 s with nothing commanded (silence), the stick is
# pushed from 5.0 s while it is still out, the lead is back at 7.0 s with the
# stick still pushed, released at 9.0 s and pushed again from 9.5 s.
PULLPUSH_SCRIPT = [
    (0.0, 5.0, None, None),
    (5.0, 6.0, (0.0, 0.30), (0.0, 0.0)),       # pushed with the lead out
    (6.0, 9.0, (0.30, 0.30), (0.0, 0.0)),      # still pushed after the re-seat
    (9.0, 9.5, (0.0, 0.0), (0.0, 0.0)),        # released
    (9.5, 11.5, (0.0, 0.30), (0.0, 0.0)),      # pushed again
    (11.5, 12.5, (0.30, 0.0), (0.0, 0.0)),
    (12.5, 14.0, (0.0, 0.0), (0.0, 0.0)),
]
PULLPUSH_MUTE = (4.0, 7.0)

# A live stream from t = 0, before the driver exists (base_ab.sh starts the
# driver LIVE_DRIVER_START_S later), released at 12.0 s, pushed again.
LIVE_SCRIPT = [
    (0.0, 12.0, (0.30, 0.30), (0.0, 0.0)),
    (12.0, 12.5, (0.0, 0.0), (0.0, 0.0)),      # released
    (12.5, 14.5, (0.0, 0.30), (0.0, 0.0)),     # pushed again
    (14.5, 15.5, (0.30, 0.0), (0.0, 0.0)),
    (15.5, 17.0, (0.0, 0.0), (0.0, 0.0)),
]
LIVE_RATE_HZ = 20.0
LIVE_SECONDS = 17.0
LIVE_DRIVER_START_S = 2.0  # after the first message (base_ab.sh)

SCENARIOS = {
    # name: (command script, mute window, rate, seconds)
    "default": (CMD_SCRIPT, None, CMD_RATE_HZ, RUN_SECONDS),
    "pull": (PULL_SCRIPT, PULL_MUTE, CMD_RATE_HZ, PULL_SECONDS),
    "pullpush": (PULLPUSH_SCRIPT, PULLPUSH_MUTE, CMD_RATE_HZ, PULL_SECONDS),
    "live": (LIVE_SCRIPT, None, LIVE_RATE_HZ, LIVE_SECONDS),
}

# (t_from, t_to, rate_hz, topic, payload)
SIDE_SCRIPT = [
    (2.0, 2.1, 10.0, "led", '{"mode":6,"r":255,"g":180,"b":0,"period_ms":1600}'),
    (5.0, 6.0, 10.0, "blade", '{"permille":300,"ttl_ms":500}'),
    (7.0, 7.1, 10.0, "servo", '{"pulse_us":1500,"hold_ms":0}'),
    (12.0, 12.1, 10.0, "pid",
     '{"left":{"kp":2.0,"ki":0.6,"kd":0.0},"right":{"kp":2.0,"ki":0.6,"kd":0.0},'
     '"persist":0,"closed_loop":1}'),
    (16.5, 18.5, 10.0, "override", '{"left_permille":400,"right_permille":-400,"ttl_ms":300}'),
]


def command_at(t: float, script=CMD_SCRIPT):
    """(linear.x, angular.z) at `t`, or None where the script is silent."""
    for lo, hi, lin, ang in script:
        if lo <= t < hi:
            if lin is None:
                return None
            u = (t - lo) / (hi - lo)
            return (lin[0] + (lin[1] - lin[0]) * u, ang[0] + (ang[1] - ang[0]) * u)
    return None


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def name_of(policy) -> str:
    """`RELIABLE`, not `1`: Jazzy's QoS policies are IntEnums, whose str() is the number."""
    return getattr(policy, "name", str(policy))


def stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Harness(Node):
    def __init__(self, label: str, scenario: str = "default", mute_file: str = "",
                 streaming_file: str = "") -> None:
        super().__init__("base_harness")
        # touched on the first command published, so base_ab.sh can start
        # the driver only once the live stream really is running
        self.streaming_file = streaming_file
        self.label = label
        self.scenario = scenario
        self.script, self.mute, rate_hz, _ = SCENARIOS[scenario]
        self.side_script = SIDE_SCRIPT if scenario == "default" else []
        self.mute_file = mute_file
        self.t0 = time.monotonic()
        self.rec = {
            "label": label,
            "scenario": scenario,
            "mute": list(self.mute) if self.mute else None,
            # CLOCK_MONOTONIC is shared across processes on Linux, so this
            # is what lines the harness up with fake_base.py's log.
            "t0_monotonic": self.t0,
            "odom": [],
            "joint_states": [],
            "tf": [],
            "telemetry": [],
            "firmware_info": [],
            "cmd_log": [],
            "side_log": [],
        }
        system = QoSProfile(depth=10)
        best_effort_1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.cmd_pub = self.create_publisher(TwistStamped, "/drivetrain_guarded_cmd_vel", system)
        self.side_pubs = {
            "led": self.create_publisher(String, "/mower_base/led_command", latched),
            "pid": self.create_publisher(String, "/mower_base/pid_command", QoSProfile(depth=4)),
            "override": self.create_publisher(String, "/mower_base/wheel_override", best_effort_1),
            "servo": self.create_publisher(String, "/mower_base/servo_command", best_effort_1),
            "blade": self.create_publisher(String, "/mower_base/blade_command", best_effort_1),
        }
        self.create_subscription(Odometry, "/odom", self.on_odom, system)
        self.create_subscription(JointState, "/joint_states", self.on_joints, system)
        self.create_subscription(TFMessage, "/tf", self.on_tf, system)
        self.create_subscription(String, "/mower_base/telemetry", self.on_telemetry, best_effort_1)
        self.create_subscription(String, "/mower_base/firmware_info", self.on_firmware, latched)

        # Nothing is published or recorded until wait_for_discovery() has
        # finished and reset the clock: spinning during discovery would
        # otherwise fire the command timer, drive the robot a few
        # centimetres before t = 0, and leave the two runs at different
        # start poses -- which reads as an odometry difference and is not
        # one.
        self.started = False
        self.create_timer(1.0 / rate_hz, self.on_cmd_tick)
        self.create_timer(0.02, self.on_side_tick)
        self._side_sent = {}

    def wait_for_discovery(self, timeout_s: float = 20.0) -> bool:
        """Do not start the clock until every endpoint has matched.

        Both runs must see the same thing at t = 0. Without this the run
        whose driver started later loses its first second or two of /odom to
        DDS discovery and the two trajectories are shifted, which looks
        exactly like an implementation difference and is not one.
        """
        deadline = time.monotonic() + timeout_s
        needed = ["/odom", "/joint_states", "/mower_base/telemetry"]
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(self.count_publishers(t) > 0 for t in needed) and (
                self.count_subscribers("/drivetrain_guarded_cmd_vel") > 0
            ):
                # matched; give the middleware a moment to finish both ways
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                self.t0 = time.monotonic()
                self.rec["t0_monotonic"] = self.t0
                # not firmware_info: it is latched and arrives once, during
                # discovery, which is exactly what latching is for
                for key in ("odom", "joint_states", "tf", "telemetry",
                            "cmd_log", "side_log"):
                    self.rec[key].clear()
                self.started = True
                return True
        self.t0 = time.monotonic()
        self.rec["t0_monotonic"] = self.t0
        self.started = True
        return False

    # -- clock -----------------------------------------------------------
    def rel(self) -> float:
        return time.monotonic() - self.t0

    # -- publishing ------------------------------------------------------
    def on_cmd_tick(self) -> None:
        if not self.started:
            return
        t = self.rel()
        if self.mute_file and self.mute:
            muted = self.mute[0] <= t < self.mute[1]
            if muted and not os.path.exists(self.mute_file):
                open(self.mute_file, "w").close()
            elif not muted and os.path.exists(self.mute_file):
                os.unlink(self.mute_file)
        command = command_at(t, self.script)
        if command is None:
            return
        lin, ang = command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.cmd_pub.publish(msg)
        if self.streaming_file:
            open(self.streaming_file, "w").close()
            self.streaming_file = ""
        self.rec["cmd_log"].append(
            {"t": round(t, 6), "stamp": stamp_s(msg.header), "lin": lin, "ang": ang}
        )

    def on_side_tick(self) -> None:
        if not self.started:
            return
        t = self.rel()
        for i, (lo, hi, rate, topic, payload) in enumerate(self.side_script):
            if not (lo <= t < hi):
                continue
            last = self._side_sent.get(i)
            if last is not None and t - last < 1.0 / rate:
                continue
            self._side_sent[i] = t
            self.side_pubs[topic].publish(String(data=payload))
            self.rec["side_log"].append({"t": round(t, 6), "topic": topic, "data": payload})

    # -- recording -------------------------------------------------------
    def on_odom(self, msg: Odometry) -> None:
        if not self.started:
            return
        self.rec["odom"].append(
            {
                "t": round(self.rel(), 6),
                "stamp": stamp_s(msg.header),
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": yaw_of(msg.pose.pose.orientation),
                "vx": msg.twist.twist.linear.x,
                "wz": msg.twist.twist.angular.z,
                "pose_cov": list(msg.pose.covariance),
                "twist_cov": list(msg.twist.covariance),
            }
        )

    def on_joints(self, msg: JointState) -> None:
        if not self.started:
            return
        self.rec["joint_states"].append(
            {
                "t": round(self.rel(), 6),
                "stamp": stamp_s(msg.header),
                "frame_id": msg.header.frame_id,
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        )

    def on_tf(self, msg: TFMessage) -> None:
        if not self.started:
            return
        for tr in msg.transforms:
            self.rec["tf"].append(
                {
                    "t": round(self.rel(), 6),
                    "stamp": stamp_s(tr.header),
                    "frame_id": tr.header.frame_id,
                    "child_frame_id": tr.child_frame_id,
                    "x": tr.transform.translation.x,
                    "y": tr.transform.translation.y,
                }
            )

    def on_telemetry(self, msg: String) -> None:
        if not self.started:
            return
        self.rec["telemetry"].append({"t": round(self.rel(), 6), "data": msg.data})

    def on_firmware(self, msg: String) -> None:
        # No `started` gate: this topic is latched and its one message
        # arrives during the discovery wait, which is the point of latching.
        self.rec["firmware_info"].append({"t": round(self.rel(), 6), "data": msg.data})

    # -- graph snapshot --------------------------------------------------
    def snapshot_graph(self) -> None:
        info = {}
        # every topic mower_base owns, both directions
        for topic in [
            "/odom",
            "/joint_states",
            "/tf",
            "/mower_base/telemetry",
            "/mower_base/firmware_info",
            "/drivetrain_guarded_cmd_vel",
            "/mower_base/led_command",
            "/mower_base/pid_command",
            "/mower_base/wheel_override",
            "/mower_base/servo_command",
            "/mower_base/blade_command",
        ]:
            entries = []
            for endpoint in self.get_publishers_info_by_topic(topic) + (
                self.get_subscriptions_info_by_topic(topic)
            ):
                qos = endpoint.qos_profile
                entries.append(
                    {
                        "node": endpoint.node_name,
                        "type": endpoint.topic_type,
                        "kind": name_of(endpoint.endpoint_type),
                        "reliability": name_of(qos.reliability),
                        "durability": name_of(qos.durability),
                        "depth": qos.depth,
                        "history": name_of(qos.history),
                        "liveliness": name_of(qos.liveliness),
                    }
                )
            info[topic] = entries
        self.rec["graph"] = info
        self.rec["nodes"] = sorted(
            f"{ns}{'' if ns.endswith('/') else '/'}{name}"
            for name, ns in self.get_node_names_and_namespaces()
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="default")
    ap.add_argument("--mute-file", default="",
                    help="the file fake_base.py --mute-file watches (pull, pullpush)")
    args = ap.parse_args()
    if args.seconds is None:
        args.seconds = SCENARIOS[args.scenario][3]
    if SCENARIOS[args.scenario][1] and not args.mute_file:
        ap.error(f"--scenario {args.scenario} needs --mute-file")

    rclpy.init()
    node = Harness(args.label, args.scenario, args.mute_file,
                   args.out + ".streaming" if args.scenario == "live" else "")
    if args.scenario == "live":
        # the stream is already running when the driver starts: no waiting
        node.started = True
    elif not node.wait_for_discovery():
        print(f"[{args.label}] WARNING: not every endpoint matched before the run")
    end = time.monotonic() + args.seconds
    graphed = False
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if not graphed and node.rel() > 10.0:
            node.snapshot_graph()
            graphed = True
    if not graphed:
        node.snapshot_graph()
    if args.mute_file and os.path.exists(args.mute_file):
        os.unlink(args.mute_file)
    with open(args.out, "w") as f:
        json.dump(node.rec, f)
    print(
        f"[{args.label}] odom={len(node.rec['odom'])} joints={len(node.rec['joint_states'])} "
        f"tf={len(node.rec['tf'])} telemetry={len(node.rec['telemetry'])} "
        f"cmd={len(node.rec['cmd_log'])}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

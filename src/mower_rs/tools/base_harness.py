#!/usr/bin/env python3
"""Drive one base implementation through a scripted run and record it.

Used twice by `base_compare.py`: once against `ros2_control_node` +
`mower_hardware` + `diff_controller` + `joint_state_broadcaster`, once
against the Rust `mower_base` node, with the same `fake_base.py` behind
both. Publishes the same `/drivetrain_guarded_cmd_vel` script and the same
side-channel bursts, and records `/odom`, `/joint_states`, `/tf` and
`/mower_base/telemetry`.

    ros2 run ... base_harness.py --out /tmp/run_a.json --label ros2_control

Nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
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
]
CMD_RATE_HZ = 200.0
RUN_SECONDS = 21.0

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


def command_at(t: float):
    """(linear.x, angular.z) at `t`, or None where the script is silent."""
    for lo, hi, lin, ang in CMD_SCRIPT:
        if lo <= t < hi:
            if lin is None:
                return None
            u = (t - lo) / (hi - lo)
            return (lin[0] + (lin[1] - lin[0]) * u, ang[0] + (ang[1] - ang[0]) * u)
    return None


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Harness(Node):
    def __init__(self, label: str) -> None:
        super().__init__("base_harness")
        self.label = label
        self.t0 = time.monotonic()
        self.rec = {
            "label": label,
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

        self.create_timer(1.0 / CMD_RATE_HZ, self.on_cmd_tick)
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
                return True
        return False

    # -- clock -----------------------------------------------------------
    def rel(self) -> float:
        return time.monotonic() - self.t0

    # -- publishing ------------------------------------------------------
    def on_cmd_tick(self) -> None:
        t = self.rel()
        command = command_at(t)
        if command is None:
            return
        lin, ang = command
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.cmd_pub.publish(msg)
        self.rec["cmd_log"].append(
            {"t": round(t, 6), "stamp": stamp_s(msg.header), "lin": lin, "ang": ang}
        )

    def on_side_tick(self) -> None:
        t = self.rel()
        for i, (lo, hi, rate, topic, payload) in enumerate(SIDE_SCRIPT):
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
        self.rec["joint_states"].append(
            {
                "t": round(self.rel(), 6),
                "stamp": stamp_s(msg.header),
                "name": list(msg.name),
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
            }
        )

    def on_tf(self, msg: TFMessage) -> None:
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
        self.rec["telemetry"].append({"t": round(self.rel(), 6), "data": msg.data})

    def on_firmware(self, msg: String) -> None:
        self.rec["firmware_info"].append({"t": round(self.rel(), 6), "data": msg.data})

    # -- graph snapshot --------------------------------------------------
    def snapshot_graph(self) -> None:
        info = {}
        for topic in [
            "/odom",
            "/joint_states",
            "/tf",
            "/mower_base/telemetry",
            "/mower_base/firmware_info",
            "/drivetrain_guarded_cmd_vel",
        ]:
            entries = []
            for endpoint in self.get_publishers_info_by_topic(topic) + (
                self.get_subscriptions_info_by_topic(topic)
            ):
                entries.append(
                    {
                        "node": endpoint.node_name,
                        "type": endpoint.topic_type,
                        "kind": str(endpoint.endpoint_type),
                        "reliability": str(endpoint.qos_profile.reliability),
                        "durability": str(endpoint.qos_profile.durability),
                        "depth": endpoint.qos_profile.depth,
                        "history": str(endpoint.qos_profile.history),
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
    ap.add_argument("--seconds", type=float, default=RUN_SECONDS)
    args = ap.parse_args()

    rclpy.init()
    node = Harness(args.label)
    if not node.wait_for_discovery():
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

#!/usr/bin/env python3
"""Compare two `base_harness.py` runs: `ros2_control` vs. Rust `mower_base`.

    base_compare.py --a /tmp/out_a --b /tmp/out_b [--json /tmp/compare.json]

Each directory holds `run.json` (what the harness recorded off the ROS graph)
and `fake.jsonl` (every frame `fake_base.py` saw or sent). The two runs are
lined up on the harness clock: the command script is a function of time, so
the same `t` means the same commanded velocity in both, and CLOCK_MONOTONIC
is shared, which is what lets a frame in `fake.jsonl` be placed on the
harness timeline.

What is compared, and why each tolerance is what it is, is printed with the
numbers. Nothing here runs on the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics

WHEEL_SPEED_COMMAND = 0x01
LAWER_MOTOR_COMMAND = 0x02
WS2812_COMMAND = 0x03
PID_CONFIG_COMMAND = 0x04
INFO_REQUEST = 0x06
SERVO_COMMAND = 0x07

# The window both runs are steady in: after the settle and before the harness
# stops publishing at the very end.
WINDOW = (2.0, 20.0)


def load(directory: str):
    with open(os.path.join(directory, "run.json")) as f:
        run = json.load(f)
    frames = []
    t0_fake = None
    with open(os.path.join(directory, "fake.jsonl")) as f:
        for line in f:
            rec = json.loads(line)
            if rec["dir"] == "meta":
                t0_fake = rec["fields"]["t0_monotonic"]
                continue
            frames.append(rec)
    # Put every frame on the harness timeline.
    shift = (t0_fake - run["t0_monotonic"]) if t0_fake is not None else 0.0
    for rec in frames:
        rec["h"] = rec["t"] + shift
    return run, frames


def rate(samples, key="t", window=WINDOW):
    xs = [s[key] for s in samples if window[0] <= s[key] <= window[1]]
    if len(xs) < 2:
        return 0.0, len(xs)
    return (len(xs) - 1) / (xs[-1] - xs[0]), len(xs)


def interp(series, key, t):
    """Linear interpolation of `key` over a list of {t: ..., key: ...}."""
    if not series:
        return None
    if t <= series[0]["t"]:
        return series[0][key]
    if t >= series[-1]["t"]:
        return series[-1][key]
    lo, hi = 0, len(series) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if series[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = series[lo], series[hi]
    span = b["t"] - a["t"]
    u = 0.0 if span <= 0 else (t - a["t"]) / span
    return a[key] + (b[key] - a[key]) * u


def commands(frames):
    """The 0x01 wheel-speed frames as (harness time, left, right, timeout)."""
    return [
        (f["h"], f["fields"]["left"], f["fields"]["right"], f["fields"]["timeout_ms"])
        for f in frames
        if f["dir"] == "rx" and f["type"] == WHEEL_SPEED_COMMAND and f["fields"]
    ]


def hold(series, t):
    """Zero-order hold: the last command at or before `t`."""
    value = None
    for entry in series:
        if entry[0] > t:
            break
        value = entry
    return value


def section(title):
    print()
    print(title)
    print("-" * len(title))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--a", required=True, help="ros2_control run directory")
    ap.add_argument("--b", required=True, help="Rust mower_base run directory")
    ap.add_argument("--json", help="write the numbers here as well")
    args = ap.parse_args()

    run_a, frames_a = load(args.a)
    run_b, frames_b = load(args.b)
    out = {}

    # ---- 1. the external contract ------------------------------------
    section("graph (topic: endpoint kind, type, reliability/durability/depth)")
    for topic in sorted(set(run_a.get("graph", {})) | set(run_b.get("graph", {}))):
        ea = run_a.get("graph", {}).get(topic, [])
        eb = run_b.get("graph", {}).get(topic, [])

        def fmt(entries):
            return sorted(
                "{} {} {}/{}/{}".format(
                    e["kind"].split(".")[-1],
                    e["type"],
                    e["reliability"].split(".")[-1],
                    e["durability"].split(".")[-1],
                    e["depth"],
                )
                for e in entries
                if e["node"] != "base_harness"
            )

        fa, fb = fmt(ea), fmt(eb)
        print(f"{topic}\n    A: {fa}\n    B: {fb}\n    {'SAME' if fa == fb else 'DIFFERENT'}")
        out.setdefault("graph", {})[topic] = {"a": fa, "b": fb, "same": fa == fb}
    out["nodes"] = {"a": run_a.get("nodes"), "b": run_b.get("nodes")}

    # ---- 2. rates -----------------------------------------------------
    section("publish rates in the %.0f-%.0f s window (Hz, count)" % WINDOW)
    out["rates"] = {}
    for key in ["odom", "joint_states", "tf", "telemetry"]:
        ra, na = rate(run_a[key])
        rb, nb = rate(run_b[key])
        print(f"{key:14s} A {ra:7.3f} ({na:5d})   B {rb:7.3f} ({nb:5d})   d={abs(ra - rb):.3f}")
        out["rates"][key] = {"a": ra, "b": rb, "n_a": na, "n_b": nb}

    # ---- 3. odometry ---------------------------------------------------
    section("odometry (B resampled onto A's stamps, %.0f-%.0f s)" % WINDOW)
    grid = [s["t"] for s in run_a["odom"] if WINDOW[0] <= s["t"] <= WINDOW[1]]
    diffs = {k: [] for k in ["x", "y", "yaw", "vx", "wz"]}
    for t in grid:
        for k in diffs:
            va = interp(run_a["odom"], k, t)
            vb = interp(run_b["odom"], k, t)
            diffs[k].append(abs(va - vb))
    for k, values in diffs.items():
        if values:
            print(f"  {k:4s} max {max(values):.3e}   mean {statistics.fmean(values):.3e}")
            out.setdefault("odom_diff", {})[k] = {"max": max(values), "mean": statistics.fmean(values)}
    if grid:
        dx = interp(run_a["odom"], "x", grid[-1]) - interp(run_b["odom"], "x", grid[-1])
        dy = interp(run_a["odom"], "y", grid[-1]) - interp(run_b["odom"], "y", grid[-1])
        drift = math.hypot(dx, dy)
        travelled = math.hypot(
            interp(run_a["odom"], "x", grid[-1]), interp(run_a["odom"], "y", grid[-1])
        )
        print(f"  final position difference {drift:.3e} m after {travelled:.3f} m travelled")
        out["odom_final_drift_m"] = drift
        out["odom_travelled_m"] = travelled
    for key in ["frame_id", "child_frame_id", "pose_cov", "twist_cov"]:
        va = run_a["odom"][0][key] if run_a["odom"] else None
        vb = run_b["odom"][0][key] if run_b["odom"] else None
        print(f"  {key:16s} {'SAME' if va == vb else f'A={va} B={vb}'}")
        out.setdefault("odom_fields", {})[key] = {"a": va, "b": vb, "same": va == vb}

    # ---- 4. joint states ----------------------------------------------
    section("joint_states")
    for label, run in (("A", run_a), ("B", run_b)):
        first = run["joint_states"][0] if run["joint_states"] else {}
        print(f"  {label} names={first.get('name')} effort={first.get('effort')}")
    out["joint_names"] = {
        "a": run_a["joint_states"][0]["name"] if run_a["joint_states"] else None,
        "b": run_b["joint_states"][0]["name"] if run_b["joint_states"] else None,
    }
    # The wheel positions come from the fake plant, which both runs drive with
    # the same commands, so the travelled wheel angle at the end must agree.
    for i, joint in enumerate(["left", "right"]):
        pa = run_a["joint_states"][-1]["position"][i] if run_a["joint_states"] else float("nan")
        pb = run_b["joint_states"][-1]["position"][i] if run_b["joint_states"] else float("nan")
        print(f"  final {joint} wheel angle A {pa:10.4f} rad  B {pb:10.4f} rad  d={abs(pa - pb):.3e}")
        out.setdefault("joint_final_rad", {})[joint] = {"a": pa, "b": pb}

    # ---- 5. telemetry --------------------------------------------------
    section("telemetry JSON")
    ta = json.loads(run_a["telemetry"][-1]["data"]) if run_a["telemetry"] else {}
    tb = json.loads(run_b["telemetry"][-1]["data"]) if run_b["telemetry"] else {}

    def keys(doc, prefix=""):
        found = set()
        for k, v in doc.items():
            found.add(prefix + k)
            if isinstance(v, dict):
                found |= keys(v, prefix + k + ".")
        return found

    ka, kb = keys(ta), keys(tb)
    print(f"  keys: {len(ka)} vs {len(kb)}; only in A: {sorted(ka - kb)}; only in B: {sorted(kb - ka)}")
    out["telemetry_keys"] = {"only_a": sorted(ka - kb), "only_b": sorted(kb - ka)}
    fa = run_a["firmware_info"][-1]["data"] if run_a["firmware_info"] else None
    fb = run_b["firmware_info"][-1]["data"] if run_b["firmware_info"] else None
    print(f"  firmware_info A {fa}\n                B {fb}\n  {'SAME' if fa == fb else 'DIFFERENT'}")
    out["firmware_info"] = {"a": fa, "b": fb, "same": fa == fb}

    # ---- 6. command frames ---------------------------------------------
    section("0x01 wheel-speed frames seen by the fake base")
    ca, cb = commands(frames_a), commands(frames_b)
    for label, c in (("A", ca), ("B", cb)):
        span = (c[-1][0] - c[0][0]) if len(c) > 1 else 0.0
        hz = (len(c) - 1) / span if span else 0.0
        print(f"  {label}: {len(c)} frames, {hz:.3f} Hz, timeout_ms={sorted({x[3] for x in c})}")
        out.setdefault("wheel_frames", {})[label] = {"n": len(c), "hz": hz}
    # zero-order hold on a 40 ms grid, over the window
    t = WINDOW[0]
    mismatch = worst = 0
    total = 0
    while t <= WINDOW[1]:
        ha, hb = hold(ca, t), hold(cb, t)
        if ha and hb:
            total += 1
            d = max(abs(ha[1] - hb[1]), abs(ha[2] - hb[2]))
            worst = max(worst, d)
            if d:
                mismatch += 1
        t += 0.04
    print(f"  40 ms grid: {total - mismatch}/{total} samples identical, worst |d permille| = {worst}")
    out["wheel_permille"] = {"samples": total, "identical": total - mismatch, "worst": worst}

    # ---- 7. cmd_vel timeout ---------------------------------------------
    section("cmd_vel timeout (script goes silent at 14.5 s, cmd_vel_timeout = 0.25 s)")
    for label, c in (("A", ca), ("B", cb)):
        last_moving = None
        first_zero = None
        for entry in c:
            if entry[0] < 13.0:
                continue
            if entry[1] or entry[2]:
                last_moving = entry[0]
            elif last_moving is not None and first_zero is None and entry[0] > last_moving:
                first_zero = entry[0]
        print(f"  {label}: last non-zero command at {last_moving}, first zero at {first_zero}")
        out.setdefault("timeout", {})[label] = {"last_moving": last_moving, "first_zero": first_zero}

    # ---- 8. the side channels ------------------------------------------
    section("side channels (frames the fake base received)")
    out["side_frames"] = {}
    for frame_type, name in [
        (LAWER_MOTOR_COMMAND, "0x02 blade"),
        (WS2812_COMMAND, "0x03 led"),
        (PID_CONFIG_COMMAND, "0x04 pid"),
        (INFO_REQUEST, "0x06 info"),
        (SERVO_COMMAND, "0x07 servo"),
    ]:
        na = [f for f in frames_a if f["dir"] == "rx" and f["type"] == frame_type]
        nb = [f for f in frames_b if f["dir"] == "rx" and f["type"] == frame_type]
        pa = sorted({json.dumps(f["fields"], sort_keys=True) for f in na})
        pb = sorted({json.dumps(f["fields"], sort_keys=True) for f in nb})
        print(f"  {name:11s} A {len(na):4d} frames {pa}")
        print(f"  {'':11s} B {len(nb):4d} frames {pb}")
        print(f"  {'':11s} payloads {'SAME' if pa == pb else 'DIFFERENT'}")
        out["side_frames"][name] = {"a": len(na), "b": len(nb), "payloads_same": pa == pb}

    section("wheel_override burst (16.5-18.5 s, +-400 permille, ttl 300 ms)")
    for label, c in (("A", ca), ("B", cb)):
        burst = [x for x in c if 16.4 <= x[0] <= 19.0 and x[1] == 400 and x[2] == -400]
        window = (burst[0][0], burst[-1][0]) if burst else None
        print(f"  {label}: {len(burst)} frames at +-400, {window}")
        out.setdefault("override", {})[label] = {"n": len(burst), "window": window}

    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

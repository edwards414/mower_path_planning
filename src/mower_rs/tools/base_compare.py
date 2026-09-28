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
numbers. Exit status 1 if the endpoint QoS of any topic mower_base owns
differs between the two (run both under the robot's RMW,
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp: SystemDefaultsQoS resolves
differently per RMW), or if a scenario's check below fails.

Runs recorded with `base_harness.py --scenario pull` / `pullpush` / `txpull`
/ `txstart` / `guardzero` get the cable-pull report instead of the parity
sections: what each chain commanded after the lead came back with the stick
still pushed (`txpull`: only the LubanCat TX lead was out, the feedback never
stopped; `txstart`: that lead was already out when the driver started;
`guardzero`: as txpull, then one zero injected into the still-held stream
after the recovery, which mower_base must not take for a release). `--scenario
live` gets the restart report: what reached the wheels while a stream that
was running before the driver started was still live. `mower_base` must
send 0/0 in all of them until the release and follow the stick after it;
the C++ chain has no latch and follows at once — if it does not, the run
did not reproduce the fault and proves nothing, which fails too. The release
and the push after it are read from the run (`release`; 9.0 / 9.5 s for runs
recorded before the scripts carried it).
`--scenario autotune` gets the pid_autotune report: the run's states, the
flash save's stall, and whether the second run's steps reached the wheels
after it; both sides must save (`done`), end the second run `idle` without
an error, and drive after the save. Where a run directory has the driver's
log (`base_ab.sh` writes `driver.log`), its arm-latch lines are printed.

Nothing here runs on the robot.
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
    metas = []
    with open(os.path.join(directory, "fake.jsonl")) as f:
        for line in f:
            rec = json.loads(line)
            if rec["dir"] == "meta":
                if "t0_monotonic" in rec["fields"]:
                    t0_fake = rec["fields"]["t0_monotonic"]
                else:
                    metas.append(rec)
                continue
            frames.append(rec)
    # Put every frame on the harness timeline.
    shift = (t0_fake - run["t0_monotonic"]) if t0_fake is not None else 0.0
    for rec in frames + metas:
        rec["h"] = rec["t"] + shift
    # (t, what, on): the fake's lead cuts ("muted", "deaf") and flash stalls
    run["mute_log"] = [(round(m["h"], 3), key, value)
                       for m in metas for key, value in m["fields"].items()
                       if key in ("muted", "deaf", "stalled")]
    run["latch_log"] = latch_lines(directory)
    return run, frames


def latch_lines(directory: str):
    """The driver's arm-latch and board-link log lines, if it left a log."""
    path = os.path.join(directory, "driver.log")
    if not os.path.exists(path):
        return []
    keep = ("arm latch", "STM32 re", "feedback", "COMMAND_TIMEOUT")
    with open(path, errors="replace") as f:
        return [line.rstrip() for line in f if any(k in line for k in keep)]


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
                "{} {} {}/{}/{}/{}/{}".format(
                    e["kind"].split(".")[-1],
                    e["type"],
                    e["reliability"].split(".")[-1],
                    e["durability"].split(".")[-1],
                    e.get("history", "?").split(".")[-1],
                    e["depth"],
                    e.get("liveliness", "?").split(".")[-1],
                )
                for e in entries
                if e["node"] != "base_harness"
            )

        fa, fb = fmt(ea), fmt(eb)
        print(f"{topic}\n    A: {fa}\n    B: {fb}\n    {'SAME' if fa == fb else 'DIFFERENT'}")
        out.setdefault("graph", {})[topic] = {"a": fa, "b": fb, "same": fa == fb}
    out["nodes"] = {"a": run_a.get("nodes"), "b": run_b.get("nodes")}
    differ = sorted(t for t, v in out.get("graph", {}).items() if not v["same"])
    missing = sorted(t for t, v in out.get("graph", {}).items() if not v["a"] or not v["b"])
    out["qos_same"] = not differ and not missing
    print(f"QoS check: {len(out.get('graph', {}))} topics, "
          + ("all SAME" if out["qos_same"] else f"DIFFERENT {differ}, no endpoint {missing}"))

    scenario = run_b.get("scenario") or run_a.get("scenario")
    if scenario in ("pull", "pullpush", "txpull", "txstart", "guardzero"):
        pull_report(run_a, frames_a, run_b, frames_b, out)
        return finish(args, out)
    if scenario == "live":
        live_report(run_a, frames_a, run_b, frames_b, out)
        return finish(args, out)
    if scenario == "autotune":
        autotune_report(run_a, frames_a, run_b, frames_b, out)
        return finish(args, out)

    # ---- 2. rates -----------------------------------------------------
    section("publish rates in the %.0f-%.0f s window (Hz, count)" % WINDOW)
    out["rates"] = {}
    for key in ["odom", "joint_states", "tf", "telemetry"]:
        ra, na = rate(run_a[key])
        rb, nb = rate(run_b[key])
        print(f"{key:14s} A {ra:7.3f} ({na:5d})   B {rb:7.3f} ({nb:5d})   d={abs(ra - rb):.3f}")
        out["rates"][key] = {"a": ra, "b": rb, "n_a": na, "n_b": nb}

    # ---- 3. odometry ---------------------------------------------------
    #
    # The two runs are separate wall-clock runs, so their 25 Hz loops sample
    # the command script at a different phase -- up to one cycle, and in
    # practice one to two, because the loop's own start is arbitrary. That
    # phase is a property of the test, not of either implementation, so the
    # numbers are reported twice: raw, and after shifting B by the single
    # offset that best lines up its velocity with A's. What is left after the
    # shift is what the two implementations actually disagree about.
    section("odometry (B resampled onto A's stamps, %.0f-%.0f s)" % WINDOW)
    grid = [s["t"] for s in run_a["odom"] if WINDOW[0] <= s["t"] <= WINDOW[1]]

    def odom_stats(shift):
        d = {k: [] for k in ["x", "y", "yaw", "vx", "wz"]}
        for t in grid:
            for k in d:
                d[k].append(abs(interp(run_a["odom"], k, t) - interp(run_b["odom"], k, t + shift)))
        return d

    best_shift, best_cost = 0.0, None
    step = 0.001
    shift = -0.200
    while shift <= 0.200 + 1e-9:
        cost = statistics.fmean(odom_stats(shift)["vx"])
        if best_cost is None or cost < best_cost:
            best_shift, best_cost = shift, cost
        shift += step
    out["phase_shift_s"] = best_shift
    print(f"  command-phase offset between the runs: {best_shift * 1e3:+.0f} ms")
    for label, shift in (("raw", 0.0), ("phase-aligned", best_shift)):
        diffs = odom_stats(shift)
        print(f"  [{label}]")
        for k, values in diffs.items():
            if values:
                print(f"    {k:4s} max {max(values):.3e}   mean {statistics.fmean(values):.3e}")
                out.setdefault("odom_diff_" + label.replace("-", "_"), {})[k] = {
                    "max": max(values), "mean": statistics.fmean(values)}
        if grid:
            t = grid[-1]
            dx = interp(run_a["odom"], "x", t) - interp(run_b["odom"], "x", t + shift)
            dy = interp(run_a["odom"], "y", t) - interp(run_b["odom"], "y", t + shift)
            drift = math.hypot(dx, dy)
            travelled = math.hypot(interp(run_a["odom"], "x", t), interp(run_a["odom"], "y", t))
            print(f"    final position difference {drift:.3e} m after {travelled:.3f} m travelled")
            out["odom_final_drift_m_" + label.replace("-", "_")] = drift
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
        print(f"  {label} names={first.get('name')} effort={first.get('effort')} "
              f"frame_id={first.get('frame_id')!r}")
    out["joint_frame_id"] = {
        "a": run_a["joint_states"][0].get("frame_id") if run_a["joint_states"] else None,
        "b": run_b["joint_states"][0].get("frame_id") if run_b["joint_states"] else None,
    }
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
    # `t` is the C++ read() time on the steady clock: seconds of uptime, not
    # epoch seconds. Two runs minutes apart on one host read within an hour.
    t_a, t_b = ta.get("t"), tb.get("t")
    same_clock = t_a is not None and t_b is not None and abs(t_a - t_b) < 3600
    print(f"  t: A {t_a}  B {t_b}  {'same clock' if same_clock else 'DIFFERENT CLOCK'}")
    out["telemetry_t"] = {"a": t_a, "b": t_b, "same_clock": same_clock}
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
    # Zero-order hold on a 40 ms grid, over the window, with the same phase
    # offset the odometry section measured: a ramping command sampled one or
    # two cycles apart differs by the ramp rate times that offset (about
    # 7 permille per cycle here), which says nothing about the two
    # implementations.
    for label, shift in (("raw", 0.0), ("phase-aligned", out["phase_shift_s"])):
        t = WINDOW[0]
        mismatch = worst = total = 0
        residuals = []
        while t <= WINDOW[1]:
            ha, hb = hold(ca, t), hold(cb, t + shift)
            if ha and hb:
                total += 1
                d = max(abs(ha[1] - hb[1]), abs(ha[2] - hb[2]))
                residuals.append(d)
                worst = max(worst, d)
                if d:
                    mismatch += 1
            t += 0.04
        median = statistics.median(residuals) if residuals else 0
        print(f"  [{label}] 40 ms grid: {total - mismatch}/{total} samples identical, "
              f"median |d permille| = {median}, worst = {worst}")
        out.setdefault("wheel_permille", {})[label] = {
            "samples": total, "identical": total - mismatch, "median": median, "worst": worst}

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

    return finish(args, out)


def finish(args, out) -> int:
    failures = out.setdefault("failures", [])
    if not out.get("qos_same"):
        failures.insert(0, "endpoint QoS differs")
    section("verdict")
    for failure in failures:
        print(f"  FAIL: {failure}")
    if not failures:
        print("  PASS")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
        print(f"\nwrote {args.json}")
    return 1 if failures else 0


def check(out, ok: bool, failure: str) -> None:
    """Record a scenario check; `finish` fails the run on any of them."""
    if not ok:
        out.setdefault("failures", []).append(failure)


def print_latch_log(label, run, limit=12) -> None:
    lines = run.get("latch_log") or []
    for line in lines[:limit]:
        # drop the "[INFO] [1790000000.123] [mower_base]: " prefix
        print(f"     {label} log: {line.split(']: ', 1)[-1]}")
    if len(lines) > limit:
        print(f"     {label} log: ... {len(lines) - limit} more")


def pull_report(run_a, frames_a, run_b, frames_b, out) -> None:
    """The cable-pull scenarios: what reached the wheels once the lead was back.

    `pull`: the stick is held at 0.30 m/s through the pull. `pullpush`: the
    lead comes out with nothing commanded and the stick is pushed while it
    is out. `txpull`: as `pull`, but only the LubanCat TX -> STM32 RX lead
    is out, so the feedback keeps coming and the board reports
    COMMAND_TIMEOUT. `txstart`: as `pullpush`, but that lead alone, and
    already out when the driver started. `guardzero`: as `txpull` at 10 Hz,
    and after the recovery one zero in the held stream with a stall around
    it (the 2026-09-29 robot run). Either way it is still pushed when the
    lead is back, released at `release[0]` and pushed again from
    `release[1]`. The C++ chain has no latch and resumes the pushed command
    as soon as frames get through again; mower_base must stay at 0/0 until
    the release and follow the stick again after it.
    """
    scenario = run_b.get("scenario") or run_a.get("scenario")
    tx_only = scenario in ("txpull", "txstart", "guardzero")
    leads = "LubanCat TX lead" if tx_only else "lead"
    rel = run_b.get("release") or run_a.get("release") or (9.0, 9.5)
    section("cable %s (%s out %.1f-%.1f s, stick pushed at the re-seat, released %.1f s, "
            "pushed %.1f s)" % ((scenario, leads)
                                + tuple(run_b.get("mute") or run_a.get("mute") or (0, 0))
                                + tuple(rel)))
    out["pull"] = {"scenario": scenario, "release": list(rel)}
    for label, run, frames in (("A", run_a, frames_a), ("B", run_b, frames_b)):
        mute = run.get("mute") or (4.0, 6.0)
        seen = [(f["h"], f["fields"]["left"], f["fields"]["right"])
                for f in frames
                if f["dir"] == "rx" and f["type"] == WHEEL_SPEED_COMMAND and f["fields"]]
        fb = [(f["h"], f["fields"]["left_measured_rpm"])
              for f in frames if f["dir"] == "tx" and f["type"] == 0x85]
        before = [x for x in seen if mute[0] - 0.5 <= x[0] < mute[0]]
        held = [x for x in seen if mute[1] <= x[0] < rel[0]]
        moving_after = [x for x in held if x[1] or x[2]]
        released = [x for x in seen if rel[1] + 0.1 <= x[0] < rel[1] + 2.0 and (x[1] or x[2])]
        rpm_held = max((abs(r) for h, r in fb if mute[1] + 0.3 <= h < rel[0]), default=0.0)
        status = [(f["h"], f["fields"]["flags"], f["fields"]["command_age_ms"])
                  for f in frames if f["dir"] == "tx" and f["type"] == 0x81 and f["fields"]]
        out_status = [x for x in status if mute[0] <= x[0] < mute[1]]
        timeouts = [x for x in out_status if x[1] & 0x02]
        fb_out = [h for h, _ in fb if mute[0] <= h < mute[1]]
        print(f"  {label}: before the pull {sorted({(l, r) for _, l, r in before})}")
        if tx_only:
            print(f"     lead out: {len(fb_out)} feedback frames still sent, "
                  f"{len(timeouts)}/{len(out_status)} 0x81 with COMMAND_TIMEOUT"
                  + (f", first at {timeouts[0][0]:.3f} s, command_age_ms up to "
                     f"{max(x[2] for x in timeouts)}" if timeouts else ""))
        print(f"     lead back, stick held: {len(held)} frames, {len(moving_after)} non-zero"
              + (f" (first at {moving_after[0][0]:.3f} s: {moving_after[0][1:]})" if moving_after else "")
              + f"; wheel speed up to {rpm_held:.1f} rpm")
        print(f"     after release + push: {len(released)} non-zero frames"
              + (f", first at {released[0][0]:.3f} s" if released else ""))
        zeros = [c for c in run.get("cmd_log", []) if c.get("inject")]
        after_zero = []
        if scenario == "guardzero":
            z = zeros[0]["t"] if zeros else None
            quiet = [c["t"] for c in run.get("cmd_log", []) if z is not None and c["t"] < z]
            after_zero = [x for x in seen if z is not None and z < x[0] < rel[0] and (x[1] or x[2])]
            print(f"     injected zero at {z if z is None else round(z, 3)} s "
                  f"({(z - quiet[-1]) if quiet else float('nan'):.3f} s after the stick's last "
                  f"message); non-zero frames after it until the release: {len(after_zero)}"
                  + (f", first at {after_zero[0][0]:.3f} s: {after_zero[0][1:]}" if after_zero else ""))
        print_latch_log(label, run)
        out["pull"][label] = {
            "held_frames": len(held),
            "held_nonzero": len(moving_after),
            "first_held": list(held[0]) if held else None,
            "held_max_rpm": rpm_held,
            "after_release_nonzero": len(released),
            "status_while_out": len(out_status),
            "timeouts_while_out": len(timeouts),
            "feedback_while_out": len(fb_out),
            "mute_log": run.get("mute_log"),
            "latch_log": run.get("latch_log"),
        }
        out["pull"][label]["after_zero_nonzero"] = len(after_zero)
        if scenario == "guardzero":
            check(out, len(zeros) == 1, f"guardzero {label}: {len(zeros)} injected zeros, not one")
        if label == "A":
            check(out, bool(moving_after),
                  f"{scenario}: the C++ chain did not follow the pushed stick after the re-seat, "
                  "so the run did not reproduce the fault")
            if scenario == "guardzero":
                check(out, bool(after_zero),
                      "guardzero: the C++ chain did not follow the stick after the injected zero")
            continue
        check(out, bool(held), f"{scenario}: no 0x01 reached the fake after the re-seat")
        check(out, not moving_after,
              f"{scenario}: mower_base sent {len(moving_after)} non-zero 0x01 after the re-seat "
              "with the stick still pushed (a lurch)")
        check(out, bool(released),
              f"{scenario}: mower_base did not follow the stick after the release")
        if tx_only:
            check(out, bool(fb_out) and bool(timeouts),
                  f"{scenario}: the fake did not keep sending feedback with COMMAND_TIMEOUT "
                  "while the lead was out")
        if scenario == "guardzero" and run.get("latch_log"):
            check(out, any("not held for" in line for line in run["latch_log"]),
                  "guardzero: mower_base did not log the stop it did not take for a release")


def live_report(run_a, frames_a, run_b, frames_b, out) -> None:
    """A restart under a live stream: 0.30 m/s from before the driver
    started until the release (12.0 s), pushed again after it (13.0 s; 12.5 s
    in runs recorded before the scripts carried `release`)."""
    rel = run_b.get("release") or run_a.get("release") or (12.0, 12.5)
    section("live stream before the driver (0.30 m/s until %.1f s, released, pushed %.1f s)"
            % tuple(rel))
    out["live"] = {"release": list(rel)}
    for label, frames in (("A", frames_a), ("B", frames_b)):
        seen = [(f["h"], f["fields"]["left"], f["fields"]["right"])
                for f in frames
                if f["dir"] == "rx" and f["type"] == WHEEL_SPEED_COMMAND and f["fields"]]
        live = [x for x in seen if x[0] < rel[0]]
        moving = [x for x in live if x[1] or x[2]]
        after = [x for x in seen if rel[1] + 0.1 <= x[0] < rel[1] + 2.0 and (x[1] or x[2])]
        first = seen[0][0] if seen else None
        print(f"  {label}: first 0x01 at {first if first is None else round(first, 3)} s; "
              f"{len(live)} frames while the stream was live, {len(moving)} non-zero"
              + (f" (first at {moving[0][0]:.3f} s: {moving[0][1:]})" if moving else ""))
        print(f"     after release + push: {len(after)} non-zero frames"
              + (f", first at {after[0][0]:.3f} s" if after else ""))
        out["live"][label] = {
            "first_frame": first,
            "live_frames": len(live),
            "live_nonzero": len(moving),
            "after_release_nonzero": len(after),
        }
        if label == "A":
            check(out, bool(moving),
                  "live: the C++ chain did not follow the live stream, so the run did not "
                  "reproduce a restart under it")
            continue
        check(out, bool(live), "live: no 0x01 reached the fake while the stream was live")
        check(out, not moving,
              f"live: mower_base sent {len(moving)} non-zero 0x01 while the old stream was live")
        check(out, bool(after), "live: mower_base did not follow the stick after the release")


def autotune_report(run_a, frames_a, run_b, frames_b, out) -> None:
    """pid_autotune end to end: two runs, the first applied (the flash save
    stalls the fake board like the STM32's sector erase), the second
    discarded. The second run's open-loop steps must reach the wheels: the
    save must not leave the base latched."""
    section("pid_autotune: run 1 applied (flash save), run 2 discarded")
    out["autotune"] = {}
    for label, run, frames in (("A", run_a, frames_a), ("B", run_b, frames_b)):
        at = run.get("autotune") or {}
        states = [x["state"] for x in at.get("states", [])]
        saves = [f["h"] for f in frames
                 if f["dir"] == "rx" and f["type"] == PID_CONFIG_COMMAND
                 and f["fields"].get("persist")]
        stalls = [x for x in run.get("mute_log", []) if x[1] == "stalled"]
        seen = [(f["h"], f["fields"]["left"], f["fields"]["right"])
                for f in frames
                if f["dir"] == "rx" and f["type"] == WHEEL_SPEED_COMMAND and f["fields"]]
        save_t = saves[0] if saves else None
        before = [x for x in seen if save_t is not None and x[0] < save_t and (x[1] or x[2])]
        after = [x for x in seen if save_t is not None and x[0] > save_t and (x[1] or x[2])]
        levels_after = sorted({x[1] for x in after})
        # the board's reports around the save (the stall plus a second);
        # the ones after the drivers stop at the end are not about the save
        stall_end = max((t for t, _, on in stalls if not on), default=save_t)
        timeouts = [(f["h"], f["fields"]["command_age_ms"]) for f in frames
                    if f["dir"] == "tx" and f["type"] == 0x81 and f["fields"]
                    and f["fields"]["flags"] & 0x02 and save_t is not None
                    and save_t < f["h"] <= stall_end + 1.0]
        print(f"  {label}: states {states}; error {at.get('error')}")
        print(f"     flash save 0x04 at {save_t if save_t is None else round(save_t, 3)} s, "
              f"stall log {stalls}")
        print(f"     0x81 COMMAND_TIMEOUT within 1 s of the stall: {len(timeouts)}"
              + (f" (first {timeouts[0][0]:.3f} s, command_age_ms {timeouts[0][1]})" if timeouts else ""))
        print(f"     non-zero 0x01: {len(before)} before the save, {len(after)} after it "
              f"(left permille levels after: {levels_after})")
        print_latch_log(label, run, limit=20)
        out["autotune"][label] = {
            "states": states,
            "error": at.get("error"),
            "calls": at.get("calls"),
            "save_t": save_t,
            "stalls": stalls,
            "timeouts_after_save": len(timeouts),
            "nonzero_before_save": len(before),
            "nonzero_after_save": len(after),
            "levels_after_save": levels_after,
            "latch_log": run.get("latch_log"),
        }
        saved = "saving" in states and "done" in states[states.index("saving"):]
        check(out, saved, f"autotune {label}: the first run did not save (states {states})")
        check(out, bool(states) and states[-1] == "idle" and "done" in states[:-1],
              f"autotune {label}: the second run did not end idle after the save (states {states})")
        check(out, not {"failed", "aborted"} & set(states) and at.get("error") is None,
              f"autotune {label}: failed or aborted (error {at.get('error')})")
        check(out, bool(after),
              f"autotune {label}: no non-zero 0x01 after the flash save: the second run's steps "
              "never reached the wheels")


if __name__ == "__main__":
    raise SystemExit(main())

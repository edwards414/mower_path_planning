#!/usr/bin/env python3
"""The CPU levels of the outdoor harness from idle CPU windows (cpu_window.py).

    calibrate.py cpu_idle_1.json [cpu_idle_2.json ...] > calibration.json

D_stack = the stack's mean idle use in cores (every process in the sim's PID
namespace except mower_sim, the observer's own tools and sleepers);
D_sim = mower_sim's. The robot's idle load ratio L is 0.30 at 1992 MHz, 0.51
at 1008 and 1.25 at 408 (PLAN.md), so the stack gets X = D_stack / L cores
plus what mower_sim itself needs (it stands for hardware, which costs the
robot nothing): quota = X * 10 ms per 10 ms period.
"""
import json
import sys

LOAD = {"T100": 0.30, "T50": 0.51, "T20": 1.25}
OBSERVER = ("cpu_window.py", "observe.py", "faults.py", "sleep", "probe_app.py", "docker-init", "tini")


def split(doc):
    stack = sim = obs = 0.0
    for p in doc["processes"]:
        cores = p["cpu_pct"] / 100.0
        cmd = p.get("cmd", "")
        if p["name"] == "mower_sim" or "/mower_sim " in cmd + " ":
            sim += cores
        elif any(k in cmd or p["name"] == k for k in OBSERVER):
            obs += cores
        else:
            stack += cores
    return stack, sim, obs


def main():
    rows = [split(json.load(open(f))) for f in sys.argv[1:]]
    d_stack = sum(r[0] for r in rows) / len(rows)
    d_sim = sum(r[1] for r in rows) / len(rows)
    docs = [json.load(open(f)) for f in sys.argv[1:]]
    cores = {k: round(d_stack / l + d_sim, 3) for k, l in LOAD.items()}
    out = {
        "windows": [{"file": f, "stack_cores": round(r[0], 3), "mower_sim_cores": round(r[1], 3),
                     "observer_cores": round(r[2], 3), "mower_rsd_pct": d.get("mower_rsd_cpu_pct")}
                    for f, r, d in zip(sys.argv[1:], rows, docs)],
        "D_stack_cores": round(d_stack, 3),
        "D_sim_cores": round(d_sim, 3),
        "L": LOAD,
        "cores": cores,
        "quota_us": {k: round(v * 10000) for k, v in cores.items()},
        "period_us": 10000,
        "top": [[p["name"], p["cpu_pct"]] for p in docs[0]["processes"][:10]],
    }
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()

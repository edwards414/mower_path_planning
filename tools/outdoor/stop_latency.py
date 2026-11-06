#!/usr/bin/env python3
"""How fast a lost sensor stops a running mission: per fault in a mission run
(run_mission.py --faults ...), from the unthrottled observer and the truth
log. Host side; development only.

    stop_latency.py RUN_DIR [RUN_DIR ...] [--json OUT] [--top-speed 0.5]

For each fault that switched on while the wheels were turning:

  last_sample_to_cmd0_s  the source's last sample (the observer's receipt) to
                         the first zero on /drivetrain_guarded_cmd_vel: what
                         the gate's budget comes to end to end
  fault_to_cmd0_s        the fault (faults.py's switch; the sim reads it within
                         50 ms) to that zero
  fault_to_wheels0_s     the fault to the truth log's wheels at rest (10 Hz
                         samples, rpm <= 0.5; includes the base's stop ramp and
                         the sim's motor lag)
  travel_m               how far the truth moved from the fault to rest
  at_top_speed_m         what fault_to_cmd0_s plus the base's 0.2 s ramp come to
                         at --top-speed (the guards' limit): v * t + v * 0.2 / 2

Which source's last sample counts: imu_gone -> /imu/data; gps_gone, a
gps_mode change -> /fix; hub_gone, sensors_mute -> the later of the two.
"""
import argparse
import glob
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from teleop_analyze import load_jsonl, pct  # noqa: E402

R_EARTH = 6378137.0
RAMP_S = 0.2


def truth_rows(run):
    rows = []
    for f in sorted(glob.glob(os.path.join(run, 'sim', 'truth.*.jsonl'))):
        rows += load_jsonl(f)
    rows.sort(key=lambda d: d['t'])
    return rows


def dist(a, b):
    n = math.radians(b['lat'] - a['lat']) * R_EARTH
    e = math.radians(b['lon'] - a['lon']) * math.cos(math.radians(a['lat'])) * R_EARTH
    return math.hypot(n, e)


def rpm(d):
    return max(abs(d['left_rpm']), abs(d['right_rpm']))


def last_before(ts, t):
    lo, hi = 0, len(ts)
    while lo < hi:
        mid = (lo + hi) // 2
        if ts[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    return ts[lo - 1] if lo else None


def events(run, top_speed):
    faults = load_jsonl(os.path.join(run, 'obs', 'faults.jsonl'))
    obs = load_jsonl(os.path.join(run, 'obs', 'mission.jsonl'))
    truth = truth_rows(run)
    by = {}
    for d in obs:
        by.setdefault(d.get('k'), []).append(d)
    fix = sorted(d['t'] for d in by.get('/fix', []))
    imu = sorted(d['t'] for d in by.get('/imu/data', []))
    out = []
    final = by.get('/drivetrain_guarded_cmd_vel', [])
    stop = by.get('/navigation_safety_stop', [])
    for f in faults:
        kind, v = f.get('k'), f.get('v')
        if kind is None or not (v is True or v in ('none', 'nosat', '3d', 'float')):
            continue
        tf = f['t']
        before = [d for d in truth if tf - 0.3 <= d['t'] <= tf]
        if not before or rpm(before[-1]) < 2.0:
            continue  # standing still: nothing to stop
        moving_cmd = [d for d in final if tf - 0.5 <= d['t'] <= tf and abs(float(d['lx'])) > 1e-6]
        if not moving_cmd:
            continue
        zero = next((d['t'] for d in final if d['t'] >= tf and abs(float(d['lx'])) < 1e-9 and abs(float(d['az'])) < 1e-9), None)
        gate = next((d['t'] for d in stop if d['t'] >= tf), None)
        rest = next((d for d in truth if d['t'] >= tf and rpm(d) <= 0.5), None)
        src = {'imu_gone': [imu], 'gps_gone': [fix], 'gps_mode': [fix]}.get(kind, [imu, fix])
        lasts = [x for x in (last_before(s, tf + 0.1) for s in src) if x is not None]
        last = max(lasts) if lasts else None
        at_fault = before[-1]
        row = {'t': round(tf, 3), 'fault': kind if v is True else f'{kind}={v}',
               'gate_after_fault_s': round(gate - tf, 3) if gate else None,
               'fault_to_cmd0_s': round(zero - tf, 3) if zero else None,
               'last_sample_to_cmd0_s': round(zero - last, 3) if zero and last else None,
               'fault_to_wheels0_s': round(rest['t'] - tf, 3) if rest else None,
               'travel_m': round(dist(at_fault, rest), 3) if rest else None}
        if zero:
            row['at_top_speed_m'] = round(top_speed * (zero - tf) + top_speed * RAMP_S / 2, 3)
        out.append(row)
    return out


def summary(rows, key):
    xs = [r[key] for r in rows if r.get(key) is not None]
    return {'n': len(xs), 'p50': pct(xs, 0.5), 'p95': pct(xs, 0.95), 'max': max(xs) if xs else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    ap.add_argument('--top-speed', type=float, default=0.5)
    args = ap.parse_args()
    doc = {}
    for run in args.runs:
        rows = events(run, args.top_speed)
        name = os.path.basename(run.rstrip('/'))
        by_kind = {}
        for r in rows:
            by_kind.setdefault(r['fault'], []).append(r)
        doc[name] = {'events': rows, 'by_fault': {k: {m: summary(v, m) for m in (
            'last_sample_to_cmd0_s', 'fault_to_cmd0_s', 'fault_to_wheels0_s', 'travel_m', 'at_top_speed_m')} for k, v in by_kind.items()}}
        print(f'== {name}: {len(rows)} stops while moving')
        for k, v in sorted(doc[name]['by_fault'].items()):
            line = '  '.join(f"{m} {v[m]['p50']}/{v[m]['p95']}/{v[m]['max']}" for m in v)
            print(f"  {k} (n={v['fault_to_cmd0_s']['n']}): {line}")
    if args.json:
        json.dump(doc, open(args.json, 'w'), indent=1)


if __name__ == '__main__':
    main()

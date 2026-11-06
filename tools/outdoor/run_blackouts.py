#!/usr/bin/env python3
"""The dead link's stop, many times over (row 2's blackout check has one per
cell): on one stack and one CPU level at a time, hold the stick, black the link
out 3 s in, bring it back, and measure each. Host side; development only. Needs
MOWER_PAIR_FILE (outdoor.sh checks it).

    OD=oda OUTDOOR_CPUSET=0-3 run_blackouts.py --cpus T100,T50,T20 [--net typ]
        [--timeout 0.2] [--final-timeout 0.2] [--repeat 12] --tag main --out DIR

Per blackout, from the observer's receipts and the truth log:
  last_cmd_to_final_zero_s  the last command the manual guard forwarded before
                            the link went, to the final guard's first zero: the
                            limit itself, end to end
  blk_to_final_zero_s       the blackout to that zero (the plan's check: no
                            motion more than 0.3 s into a blackout)
  blk_to_wheels_zero_s      the blackout to the wheels commanded to 0 (truth.py's
                            back-projection of the sim's motor lag)
  blk_to_rest_s, travel_m   the truth at rest (rpm <= 0.5), and how far it moved
                            after the blackout started, at the soak's 0.2 m/s
  at_top_speed_m            what blk_to_final_zero_s and the base's 0.2 s ramp come
                            to at 0.5 m/s, the guards' limit
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import safety  # noqa: E402
from run_row2 import cid, od, sh  # noqa: E402
from teleop_analyze import load_jsonl, pct, stop_time  # noqa: E402

R_EARTH = 6378137.0
TOP = 0.5
RAMP_S = 0.2


def dist(a, b):
    n = math.radians(b['lat'] - a['lat']) * R_EARTH
    e = math.radians(b['lon'] - a['lon']) * math.cos(math.radians(a['lat'])) * R_EARTH
    return math.hypot(n, e)


def analyze(d):
    obs = load_jsonl(os.path.join(d, 'obs', 'teleop.jsonl'))
    truth = sorted(load_jsonl(os.path.join(d, 'sim', 'truth.1.jsonl')), key=lambda x: x['t'])
    blks = json.load(open(os.path.join(d, 'blk.json')))
    joy = [x['t'] for x in obs if x.get('k') == '/joy_cmd' and abs(float(x['lx'])) > 0]
    final = [x for x in obs if x.get('k') == '/drivetrain_guarded_cmd_vel']
    rows = []
    for tb in blks:
        before = [x for x in final if tb - 0.4 <= x['t'] <= tb and abs(float(x['lx'])) > 0]
        if not before:
            rows.append({'t_blk': tb, 'moving': False})
            continue
        last = max([t for t in joy if t <= tb + 0.15] or [None], key=lambda x: x or 0)
        zero = next((x['t'] for x in final if x['t'] >= tb and abs(float(x['lx'])) < 1e-9 and abs(float(x['az'])) < 1e-9), None)
        ts, how = stop_time(truth, tb, tb + 3)
        at = next((x for x in reversed(truth) if x['t'] <= tb), None)
        rest = next((x for x in truth if x['t'] >= tb and max(abs(x['left_rpm']), abs(x['right_rpm'])) <= 0.5), None)
        nonzero_after = [x['t'] for x in final if tb <= x['t'] <= tb + 2.5 and abs(float(x['lx'])) > 0]
        rows.append({'t_blk': round(tb, 3), 'moving': True,
                     'last_cmd_to_final_zero_s': round(zero - last, 3) if zero and last else None,
                     'blk_to_final_zero_s': round(zero - tb, 3) if zero else None,
                     'blk_to_wheels_zero_s': round(ts - tb, 3) if ts else None, 'how': how,
                     'blk_to_rest_s': round(rest['t'] - tb, 3) if rest else None,
                     'travel_m': round(dist(at, rest), 3) if rest and at else None,
                     'at_top_speed_m': round(TOP * (zero - tb) + TOP * RAMP_S / 2, 3) if zero else None,
                     'last_final_nonzero_after_blk_s': round(max(nonzero_after) - tb, 3) if nonzero_after else 0.0})
    moved = [r for r in rows if r.get('moving')]

    def s(key):
        xs = [r[key] for r in moved if r.get(key) is not None]
        return [pct(xs, .5), pct(xs, .95), max(xs) if xs else None]
    out = {'run': os.path.basename(d), 'n': len(moved), 'rows': rows, **{k + '_p50_p95_max': s(k) for k in (
        'last_cmd_to_final_zero_s', 'blk_to_final_zero_s', 'blk_to_wheels_zero_s', 'blk_to_rest_s', 'travel_m', 'at_top_speed_m')},
        'pass_0p3s': sum(1 for r in moved if (r.get('blk_to_final_zero_s') or 9) <= 0.3)}
    json.dump(out, open(os.path.join(d, 'blackouts_summary.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != 'rows'}), flush=True)
    return out


def cell(args, cpu, outdir):
    d = os.path.join(outdir, f'{args.tag}_{cpu}')
    for sub in ('client', 'obs', 'sim'):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    od('cpu', cpu)
    od('net', args.net)
    time.sleep(5)
    obs, cl = cid('observer'), cid('client')
    sh('docker', 'exec', obs, 'rm', '-f', '/tmp/obs/teleop.jsonl')
    sh('docker', 'exec', '-d', obs, 'bash', '-c',
       'source /opt/ros/jazzy/setup.bash; source /mower_ws/install/setup.bash; '
       'exec python3 /to/observe.py --set teleop --out /tmp/obs/teleop.jsonl --seconds 900')
    time.sleep(4)
    p = subprocess.Popen(['docker', 'exec', cl, 'python3', '-u', '/to/ws_teleop_soak.py', '--out', '/tmp/client/teleop.json', '--seconds', '0',
                          '--blk-hold', '6', '--blk-repeat', str(args.repeat)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    blks = []
    lines = []
    for line in p.stdout:
        lines.append(line)
        if line.startswith('BLK_HOLD_START'):
            time.sleep(3.0)
            out = od('blk')
            blks.append(float(out.split()[0]))
            time.sleep(3.0)
            od('net', args.net)
    p.wait()
    sh('docker', 'exec', obs, 'pkill', '-INT', '-f', '/to/observe.py', check=False)
    time.sleep(3)
    sh('docker', 'cp', f'{cl}:/tmp/client/teleop.json', os.path.join(d, 'client', 'teleop.json'))
    sh('docker', 'cp', f'{obs}:/tmp/obs/teleop.jsonl', os.path.join(d, 'obs', 'teleop.jsonl'))
    sh('docker', 'cp', f'{obs}:/tmp/obs/vmload.jsonl', os.path.join(d, 'obs', 'vmload.jsonl'), check=False)
    sh('docker', 'cp', f'{cid("sim")}:/tmp/outdoor/truth.1.jsonl', os.path.join(d, 'sim', 'truth.1.jsonl'))
    json.dump(blks, open(os.path.join(d, 'blk.json'), 'w'))
    open(os.path.join(d, 'soak.out'), 'w').writelines(lines)
    return analyze(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cpus', default='T100,T50,T20')
    ap.add_argument('--net', default='typ')
    ap.add_argument('--timeout', type=float, default=0.20)
    ap.add_argument('--final-timeout', type=float, default=0.20)
    ap.add_argument('--repeat', type=int, default=12)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--analyze', help='only re-analyze this cell directory')
    args = ap.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    safety.check()
    if args.timeout != 0.20:
        os.environ['MANUAL_GUARD_TIMEOUT_S'] = str(args.timeout)
    if args.final_timeout != 0.20:
        os.environ['FINAL_GUARD_TIMEOUT_S'] = str(args.final_timeout)
    od('down', check=False)
    od('up', 'FULL', 'clean')
    try:
        for cpu in args.cpus.split(','):
            try:
                cell(args, cpu, args.out)
            except Exception as e:  # noqa: BLE001
                print(f'[{cpu}] failed: {e}', flush=True)
    finally:
        od('down', check=False)


if __name__ == '__main__':
    main()

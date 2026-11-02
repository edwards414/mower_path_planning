#!/usr/bin/env python3
"""Rows 7/8: one coverage mission per call on a fresh stack, with an optional
fault profile, then truth.py's metrics. Host side; development only. Needs
MOWER_PAIR_FILE (outdoor.sh checks it).

    OD=odm1 run_mission.py --cpu T50 --net clean [--faults G1|F-stall|F-hub]
        [--navcore] [--seed 1] --out ~/work/outdoor-runs/row7/<name>

Row 8 adds --navcore with MOWER_RS_TARGET pointing at a mower_rsd build
(the fixed one). cpu_window.py runs 60 s into the coverage (per-thread CPU).
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import safety  # noqa: E402
from run_row2 import od, sh, cid  # noqa: E402

SRC = 'source /opt/ros/jazzy/setup.bash; source /mower_ws/install/setup.bash; '


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cpu', default='T50')
    ap.add_argument('--net', default='clean')
    ap.add_argument('--faults', default='')
    ap.add_argument('--fault-start', type=float, default=45.0)
    ap.add_argument('--navcore', action='store_true')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--zone', type=int, default=1)
    ap.add_argument('--max-s', type=float, default=1500)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    safety.check()
    os.environ['SIM_SEED'] = str(args.seed)
    d = args.out
    for sub in ('client', 'obs', 'sim'):
        os.makedirs(os.path.join(d, sub), exist_ok=True)
    od('down', check=False)
    up = ['up', 'FULL', 'clean'] + (['navcore'] if args.navcore else [])
    od(*up)
    try:
        od('cpu', args.cpu)
        od('net', args.net)
        obs, cl = cid('observer'), cid('client')
        sh('docker', 'exec', '-d', obs, 'bash', '-c',
           SRC + 'exec python3 /to/observe.py --set mission --out /tmp/obs/mission.jsonl --seconds 3600')
        time.sleep(3)
        if args.faults:
            sh('docker', 'exec', '-d', obs, 'python3', '/to/faults.py', args.faults, '--log', '/tmp/obs/faults.jsonl',
               '--seed', str(args.seed), '--start', str(args.fault_start), '--duration', str(args.max_s))
        cpu = subprocess.Popen(['bash', '-c', f'sleep 90; docker exec {obs} python3 /v/cpu_window.py --seconds 60 '
                                f'--out /tmp/obs/cpu_mission.json --label mission'],
                               stdout=open(os.path.join(d, 'cpu_mission.txt'), 'w'), stderr=subprocess.STDOUT)
        t0 = time.time()
        r = subprocess.run(['docker', 'exec', cl, 'python3', '-u', '/to/ws_mission.py', '--out', '/tmp/client/mission.json',
                            '--zone', str(args.zone), '--max-s', str(args.max_s)], capture_output=True, text=True)
        open(os.path.join(d, 'mission.out'), 'w').write(r.stdout + r.stderr)
        print(r.stdout.strip(), flush=True)
        cpu.wait()
        sh('docker', 'exec', obs, 'bash', '-c', SRC + 'python3 /to/zone_paths.py --out /tmp/obs/zone_paths.json', check=False)
        sh('docker', 'exec', obs, 'pkill', '-INT', '-f', '/to/observe.py', check=False)
        sh('docker', 'exec', obs, 'pkill', '-f', '/to/faults.py', check=False)
        time.sleep(3)
        od('collect', d)
        with open(os.path.join(d, 'stack.log'), 'w') as f:
            subprocess.run(['docker', 'logs', '-t', cid('sim')], stdout=f, stderr=subprocess.STDOUT)
        json.dump({'args': vars(args), 't0': t0, 'rs_target': os.environ.get('MOWER_RS_TARGET', 'image'),
                   'sim_bin': os.environ.get('MOWER_SIM_BIN', 'tree')}, open(os.path.join(d, 'run.json'), 'w'))
    finally:
        od('down', check=False)
    r = subprocess.run([sys.executable, os.path.join(HERE, 'truth.py'), 'mission', d, '--zone', str(args.zone)],
                       capture_output=True, text=True)
    s = json.load(open(os.path.join(d, 'mission_summary.json'))) if r.returncode == 0 else {'error': r.stderr[-800:]}
    keys = ('end', 'duration_s', 'resumes', 'coverage_zone_pct', 'coverage_of_plan_pct', 'cross_track_m_p50_p95_max',
            'excursions_gt_0p2', 'error')
    print(json.dumps({k: s.get(k) for k in keys}), flush=True)


if __name__ == '__main__':
    main()

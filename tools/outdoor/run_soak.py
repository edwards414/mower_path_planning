#!/usr/bin/env python3
"""Rows 3 and 4 without a mission: a stack standing still (or a cold start)
under a fault profile, recorded by the observer's `loc` set, then truth.py.
Host side; development only. Needs MOWER_PAIR_FILE.

    OD=odr3a run_soak.py reconnect --cpu T50 --faults R3 --interval 8,20 \
        --duration 700 --out ~/work/outdoor-runs/row3/T50
    OD=odr4a run_soak.py cold --cpu T20 --heading 47 --record-s 60 --out ...
    OD=odr4b run_soak.py gboot --cpu T50 --record-s 150 --out ...

reconnect  up, set the CPU level, faults.py PROFILE for --duration, then
           truth.py reconnect (and loc)
cold       up-cold (throttled from boot) with --heading, record the datum
           and the pose for --record-s once the bridge answers, truth.py loc
gboot      up-cold with the receiver in 3d from boot, fixed after 90 s
           (faults.py writes gps_mode at 90 s), truth.py loc
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


def observe(seconds):
    sh('docker', 'exec', '-d', cid('observer'), 'bash', '-c',
       SRC + f'exec python3 /to/observe.py --set loc --out /tmp/obs/loc.jsonl --seconds {seconds}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['reconnect', 'cold', 'gboot'])
    ap.add_argument('--cpu', default='T50')
    ap.add_argument('--faults', default='R3')
    ap.add_argument('--interval', default='15,60')
    ap.add_argument('--duration', type=float, default=700)
    ap.add_argument('--heading', type=float, default=None)
    ap.add_argument('--record-s', type=float, default=60)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    safety.check()
    d = args.out
    os.makedirs(d, exist_ok=True)
    os.environ['SIM_SEED'] = str(args.seed)
    extra = []
    if args.heading is not None:
        extra += ['--heading', str(args.heading)]
    if args.mode == 'gboot':
        extra += ['--gps-mode', '3d']
    os.environ['SIM_ARGS'] = ' '.join(extra)
    od('down', check=False)
    t_up = time.time()
    try:
        if args.mode == 'reconnect':
            od('up', 'FULL', 'clean')
            od('cpu', args.cpu)
            observe(args.duration + 120)
            time.sleep(5)
            r = subprocess.run(['docker', 'exec', cid('observer'), 'python3', '/to/faults.py', args.faults, '--log',
                                '/tmp/obs/faults.jsonl', '--seed', str(args.seed), '--start', '20', '--interval',
                                args.interval, '--duration', str(args.duration)], capture_output=True, text=True)
            time.sleep(40)  # the last re-plug's RTK reacquisition
        else:
            od('up-cold', args.cpu)
            observe(args.record_s + 400)
            if args.mode == 'gboot':
                # gps_mode file at boot+90 s: the receiver goes from 3d to fixed
                sh('docker', 'exec', '-d', cid('observer'), 'python3', '/to/faults.py', 'G-boot-fixed', '--log',
                   '/tmp/obs/faults.jsonl', '--start', str(max(1.0, 90 - (time.time() - t_up))))
            # until the bridge answers (throttled boot), then record
            od('ready', '400')
            t_ready = time.time() - t_up
            time.sleep(args.record_s)
            json.dump({'ready_after_s': round(t_ready, 1)}, open(os.path.join(d, 'boot.json'), 'w'))
        sh('docker', 'exec', cid('observer'), 'pkill', '-INT', '-f', '/to/observe.py', check=False)
        time.sleep(3)
        od('collect', d)
        json.dump({'args': vars(args), 'sim_args': os.environ['SIM_ARGS'], 'sim_bin': os.environ.get('MOWER_SIM_BIN', 'tree'),
                   't_up': t_up}, open(os.path.join(d, 'run.json'), 'w'))
    finally:
        od('down', check=False)
    whats = ['reconnect', 'loc'] if args.mode == 'reconnect' else ['loc']
    for w in whats:
        r = subprocess.run([sys.executable, os.path.join(HERE, 'truth.py'), w, d], capture_output=True, text=True)
        print(w, r.stdout[:1500] if r.returncode == 0 else r.stderr[-800:], flush=True)


if __name__ == '__main__':
    main()

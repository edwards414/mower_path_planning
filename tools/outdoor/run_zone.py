#!/usr/bin/env python3
"""Row 5: record an 8 x 6 m zone by joystick on a fresh stack, optionally
under a fault profile timed from the start of recording, optionally with a
power kill after the save; then truth.py zone. Host side; development only.
Needs MOWER_PAIR_FILE.

    OD=odz1 run_zone.py --cpu T50 [--faults G1|G2|HUB1 --fault-at 45]
        [--power-kill] --out ~/work/outdoor-runs/row5/<name>
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
ZONES = '/home/mower/.mower/zone_record/zone_list.json'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cpu', default='T50')
    ap.add_argument('--faults', default='')
    ap.add_argument('--fault-at', type=float, default=0.5)
    ap.add_argument('--power-kill', action='store_true')
    ap.add_argument('--seed', type=int, default=5)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    safety.check()
    d = args.out
    os.makedirs(d, exist_ok=True)
    os.environ['SIM_SEED'] = str(args.seed)
    od('down', check=False)
    try:
        od('up', 'FULL', 'clean')
        od('cpu', args.cpu)
        obs, cl = cid('observer'), cid('client')
        sh('docker', 'exec', '-d', obs, 'bash', '-c',
           SRC + 'exec python3 /to/observe.py --set loc --out /tmp/obs/loc.jsonl --seconds 1800')
        time.sleep(3)
        p = subprocess.Popen(['docker', 'exec', cl, 'python3', '-u', '/to/ws_zone_record.py', '--out', '/tmp/client/zone.json'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        lines = []
        for line in p.stdout:
            lines.append(line)
            if line.startswith('RECORD_START') and args.faults:
                sh('docker', 'exec', '-d', obs, 'python3', '/to/faults.py', args.faults, '--log', '/tmp/obs/faults.jsonl',
                   '--seed', str(args.seed), '--start', str(args.fault_at), '--duration', '400')
        p.wait()
        open(os.path.join(d, 'zone.out'), 'w').writelines(lines)
        sh('docker', 'cp', f'{cid("sim")}:{ZONES}', os.path.join(d, 'zone_list.json'))
        if args.power_kill:
            sim = cid('sim')
            t = time.time()
            sh('docker', 'kill', '-s', 'KILL', sim)
            sh('docker', 'start', sim)
            od('ready', '300')
            json.dump({'killed_at': t, 'ready_after_s': round(time.time() - t, 1)}, open(os.path.join(d, 'power_kill.json'), 'w'))
            sh('docker', 'cp', f'{sim}:{ZONES}', os.path.join(d, 'zone_list.after_restart.json'))
            r = subprocess.run(['docker', 'exec', cl, 'python3', '/rt/probe_app.py', 'ws://sim:9090', '8'],
                               capture_output=True, text=True)
            open(os.path.join(d, 'probe_after_restart.txt'), 'w').write(r.stdout + r.stderr)
        else:
            sh('docker', 'exec', obs, 'pkill', '-INT', '-f', '/to/observe.py', check=False)
            sh('docker', 'exec', obs, 'pkill', '-f', '/to/faults.py', check=False)
        time.sleep(3)
        od('collect', d)
    finally:
        od('down', check=False)
    r = subprocess.run([sys.executable, os.path.join(HERE, 'truth.py'), 'zone', d], capture_output=True, text=True)
    print(r.stdout[:1500] if r.returncode == 0 else r.stderr[-800:], flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Row 2 (teleop) on one stack: for one manual-guard timeout, every CPU level
x network profile cell, each a 120 s joystick soak (ws_teleop_soak.py) under
the observer, then teleop_analyze.py. Cells with --blk-in get a held stick
under a 30 s blackout (N-blk) and a new client address after it. Host side;
development only. Needs MOWER_PAIR_FILE (outdoor.sh checks it).

    OD=odA OUTDOOR_CPUSET=0-3 run_row2.py --timeout 0.35 --cpus T50,T20 \
        --nets good,typ,bad --blk-in typ --out ~/work/outdoor-runs/row2
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
OUTDOOR = os.path.join(REPO, 'deploy', 'sim', 'outdoor', 'outdoor.sh')
sys.path.insert(0, HERE)
import safety  # noqa: E402


def sh(*args, check=True, capture=True, env=None):
    r = subprocess.run(list(args), capture_output=capture, text=True, env=env)
    if check and r.returncode != 0:
        raise RuntimeError(f'{args}: {r.returncode} {r.stderr[-800:] if r.stderr else ""}')
    return r.stdout.strip() if capture else ''


def od(*args, check=True):
    return sh(OUTDOOR, *args, check=check)


def cid(service):
    return sh('docker', 'ps', '-qf', f'name=^{os.environ["OD"]}-{service}-1$')


def vm_now():
    return float(sh('docker', 'exec', cid('client'), 'date', '+%s.%N'))


def cell(args, cpu, net, outdir):
    name = f'{args.tag}T{args.timeout:.2f}_{cpu}_{net}'
    d = os.path.join(outdir, name)
    os.makedirs(d, exist_ok=True)
    od('cpu', cpu)
    od('net', net)
    time.sleep(5)
    obs, cl = cid('observer'), cid('client')
    sh('docker', 'exec', obs, 'rm', '-f', '/tmp/obs/teleop.jsonl')
    sh('docker', 'exec', cl, 'rm', '-f', '/tmp/client/teleop.json')
    sh('docker', 'exec', '-d', obs, 'bash', '-c',
       'source /opt/ros/jazzy/setup.bash; source /mower_ws/install/setup.bash; '
       'exec python3 /to/observe.py --set teleop --out /tmp/obs/teleop.jsonl --seconds 900')
    time.sleep(4)
    blk = net in args.blk_in
    cmd = ['docker', 'exec', cl, 'python3', '-u', '/to/ws_teleop_soak.py', '--out', '/tmp/client/teleop.json',
           '--seconds', str(args.seconds), '--hold', str(args.hold), '--pause', str(args.pause)]
    if blk:
        cmd += ['--blk-hold', '45']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    blk_doc = {}
    lines = []
    for line in p.stdout:
        lines.append(line)
        if line.startswith('BLK_HOLD_START'):
            time.sleep(3.0)
            out = od('blk')
            blk_doc['t_blk'] = float(out.split()[0])
            time.sleep(30.0)
            od('ipchange')          # still dark: the sim side drops everything
            od('net', net)          # the link is back, with its new address
            blk_doc['t_unblk'] = vm_now()
    p.wait()
    sh('docker', 'exec', obs, 'pkill', '-INT', '-f', '/to/observe.py', check=False)
    time.sleep(3)
    os.makedirs(os.path.join(d, 'client'), exist_ok=True)
    os.makedirs(os.path.join(d, 'obs'), exist_ok=True)
    os.makedirs(os.path.join(d, 'sim'), exist_ok=True)
    sh('docker', 'cp', f'{cl}:/tmp/client/teleop.json', os.path.join(d, 'client', 'teleop.json'))
    sh('docker', 'cp', f'{obs}:/tmp/obs/teleop.jsonl', os.path.join(d, 'obs', 'teleop.jsonl'))
    sh('docker', 'cp', f'{obs}:/tmp/obs/vmload.jsonl', os.path.join(d, 'obs', 'vmload.jsonl'), check=False)
    sim = cid('sim')
    sh('docker', 'cp', f'{sim}:/tmp/outdoor/truth.1.jsonl', os.path.join(d, 'sim', 'truth.1.jsonl'))
    with open(os.path.join(d, 'stack.log'), 'w') as f:
        subprocess.run(['docker', 'logs', '-t', '--since', f'{int(time.time() - args.seconds - 200)}', sim],
                       stdout=f, stderr=subprocess.STDOUT)
    with open(os.path.join(d, 'soak.out'), 'w') as f:
        f.writelines(lines)
    if blk_doc:
        json.dump(blk_doc, open(os.path.join(d, 'blk.json'), 'w'))
    r = subprocess.run([sys.executable, os.path.join(HERE, 'teleop_analyze.py'), d,
                        '--manual-timeout', str(args.timeout), '--final-timeout', str(args.final_timeout)],
                       capture_output=True, text=True)
    s = json.load(open(os.path.join(d, 'teleop_summary.json'))) if r.returncode == 0 else {'error': r.stderr[-500:]}
    print(f'[{name}] accepted {s.get("accepted_pct")} % fates {s.get("fates")} rt {s.get("receipt_timeouts")} '
          f'stop {s.get("stop_after_last_accept_s_p50_p95_max")} blk {s.get("blackout", {}).get("wheels_commanded_zero_after_blk_s")}',
          flush=True)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, required=True)
    ap.add_argument('--final-timeout', type=float, default=0.20)
    ap.add_argument('--cpus', default='T50,T20')
    ap.add_argument('--nets', default='good,typ,bad')
    ap.add_argument('--blk-in', default='typ')
    ap.add_argument('--seconds', type=float, default=120.0)
    ap.add_argument('--hold', type=float, default=5.0)
    ap.add_argument('--pause', type=float, default=1.5)
    ap.add_argument('--tag', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--keep-up', action='store_true')
    args = ap.parse_args()
    args.blk_in = args.blk_in.split(',')
    safety.check()
    os.environ['MANUAL_GUARD_TIMEOUT_S'] = str(args.timeout)
    if args.final_timeout != 0.20:
        os.environ['FINAL_GUARD_TIMEOUT_S'] = str(args.final_timeout)
    os.makedirs(args.out, exist_ok=True)
    od('down', check=False)
    od('up', 'FULL', 'clean')
    results = []
    try:
        for cpu in args.cpus.split(','):
            for net in args.nets.split(','):
                try:
                    results.append(cell(args, cpu, net, args.out))
                except Exception as e:  # noqa: BLE001
                    print(f'[cell {cpu} {net}] failed: {e}', flush=True)
        stack = os.path.join(args.out, f'{args.tag}T{args.timeout:.2f}_stack')
        od('collect', stack, check=False)
    finally:
        if not args.keep_up:
            od('down', check=False)


if __name__ == '__main__':
    main()

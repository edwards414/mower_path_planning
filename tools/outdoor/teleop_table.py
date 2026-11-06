#!/usr/bin/env python3
"""Row 2's cells side by side: one line per side x CPU x network, each number
over the repeats as min..max. Host side; development only.

    teleop_table.py ROOT SIDE [SIDE ...] [--reanalyze] [--timeouts SIDE=M,F ...]

Cells are ROOT/<SIDE>_<repeat>_T<timeout>_<cpu>_<net> (run_row2.py --tag
<SIDE>_<repeat>_). The columns are the fates of the commands in the holds, the
guards' receipt timeouts, how often the wheels really dipped under a held stick,
the stop after the last accepted command (truth's wheels and the final guard's
first zero, p95 and the worst, holds the VM stalled left out) and the blackout:
the wheels commanded to zero after it started, and the last command before it
to the final guard's zero.
"""
import argparse
import glob
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = re.compile(r'^(?P<side>.+)_(?P<rep>[^_]+)_T(?P<t>[0-9.]+)_(?P<cpu>T\d+)_(?P<net>good|typ|bad)$')


def span(xs, nd=1):
    xs = [x for x in xs if x is not None]
    if not xs:
        return '-'
    a, b = min(xs), max(xs)
    return f'{a:.{nd}f}' if a == b else f'{a:.{nd}f}..{b:.{nd}f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('sides', nargs='+')
    ap.add_argument('--reanalyze', action='store_true')
    ap.add_argument('--final-timeout', type=float, default=0.20)
    args = ap.parse_args()
    cells = {}
    for d in sorted(glob.glob(os.path.join(args.root, '*'))):
        m = CELL.match(os.path.basename(d))
        if not m or m['side'] not in args.sides:
            continue
        if args.reanalyze:
            subprocess.run([sys.executable, os.path.join(HERE, 'teleop_analyze.py'), d, '--manual-timeout', m['t'],
                            '--final-timeout', str(args.final_timeout)], capture_output=True)
        f = os.path.join(d, 'teleop_summary.json')
        if os.path.exists(f):
            cells.setdefault((m['cpu'], m['net'], args.sides.index(m['side'])), []).append(json.load(open(f)))
    print('| CPU | net | side | n | soak accepted % | rejects /min | backward | stale | replay | RT manual /min | RT final /min | wheel dips /min | '
          'stop after last accept p95 / worst s | final zero p95 / worst s | blackout: wheels 0 after s | last cmd to final zero s | pass 0.3 s |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    order = {'T100': 0, 'T50': 1, 'T20': 2}
    nets = {'good': 0, 'typ': 1, 'bad': 2}
    for (cpu, net, side), rows in sorted(cells.items(), key=lambda kv: (order[kv[0][0]], nets[kv[0][1]], kv[0][2])):
        blk = [r['blackout'] for r in rows if r.get('blackout')]
        print('| ' + ' | '.join([
            cpu, net, args.sides[side], str(len(rows)),
            span([r['soak_accepted_pct'] for r in rows]),
            span([r['rejects_per_min'] for r in rows]),
            span([r['fates']['moved_backward'] for r in rows], 0),
            span([r['fates']['stale'] for r in rows], 0),
            span([r['fates']['replay_blocked'] for r in rows], 0),
            span([r['receipt_timeouts_per_min']['manual_velocity_guard'] for r in rows]),
            span([r['receipt_timeouts_per_min']['velocity_command_guard'] for r in rows]),
            span([r['soak_dips_per_min'] for r in rows]),
            span([r['stop_after_last_accept_s_p50_p95_max'][1] for r in rows], 3) + ' / ' + span([r['stop_after_last_accept_s_p50_p95_max'][2] for r in rows], 3),
            span([r['final_zero_after_last_accept_s_p50_p95_max'][1] for r in rows], 3) + ' / ' + span([r['final_zero_after_last_accept_s_p50_p95_max'][2] for r in rows], 3),
            span([b.get('wheels_commanded_zero_after_blk_s') for b in blk], 3),
            span([b.get('final_zero_after_last_command_s') for b in blk], 3),
            '/'.join('Y' if b.get('pass_0p3s') else 'N' for b in blk) or '-',
        ]) + ' |')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Row 2's table: one line per cell (manual guard timeout x CPU x network)
from each cell's teleop_summary.json, with PLAN.md's pass criteria.

    row2_table.py ~/work/outdoor-runs/row2 [--reanalyze]
"""
import glob
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def verdict(s, net, cpu):
    bad = []
    acc = s['soak_accepted_pct']
    if net == 'typ' and cpu == 'T50' and acc < 95:
        bad.append('acc<95')
    if net == 'typ' and cpu == 'T20' and acc < 85:
        bad.append('acc<85')
    if s['fates']['moved_backward'] > 0:
        bad.append('backward>0')
    if net == 'bad' and s['stop_bound_violations'] > 0:
        bad.append('stop>T+0.1')
    b = s.get('blackout')
    if b and not b['pass_0p3s']:
        bad.append('blk>0.3s')
    return 'PASS' if not bad else 'FAIL ' + ','.join(bad)


def main():
    root = sys.argv[1]
    rows = []
    for d in sorted(glob.glob(os.path.join(root, 'T*_T*_*'))):
        name = os.path.basename(d)
        t, cpu, net = name.split('_')
        if '--reanalyze' in sys.argv:
            subprocess.run([sys.executable, os.path.join(HERE, 'teleop_analyze.py'), d, '--manual-timeout', t[1:]],
                           capture_output=True)
        f = os.path.join(d, 'teleop_summary.json')
        if not os.path.exists(f):
            continue
        s = json.load(open(f))
        fa = s['fates']
        b = s.get('blackout') or {}
        rows.append((float(t[1:]), cpu, net, s, fa, b))
    print('| T s | CPU | net | soak acc % | stale | backward | replay | other | lost | RT manual/final per min | '
          'stop after release p50/p95 | stop after last accept p50/p95/max (bound) | blackout: zero after | reconnect | VM busy % | verdict |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for t, cpu, net, s, fa, b in sorted(rows, key=lambda r: (r[1] != 'T50', ['good', 'typ', 'bad'].index(r[2]), r[0])):
        mins = (s['moves_sent'] / 10.0) / 60.0
        rt = s['receipt_timeouts']
        sr = s['stop_after_release_s_p50_p95_max']
        sa = s['stop_after_last_accept_s_p50_p95_max']
        print(f"| {t:.2f} | {cpu} | {net} | {s['soak_accepted_pct']} | {fa['stale']} | {fa['moved_backward']} | "
              f"{fa['replay_blocked']} | {fa['other']} | {fa['not_delivered']} | "
              f"{rt['manual_velocity_guard'] / mins:.1f} / {rt['velocity_command_guard'] / mins:.1f} | "
              f"{sr[0]}/{sr[1]} | {sa[0]}/{sa[1]}/{sa[2]} ({s['stop_bound_s']}) | "
              f"{b.get('wheels_commanded_zero_after_blk_s', '')} | {b.get('reconnected_after_unblk_s', '')} | "
              f"{(s.get('vm_busy_pct_mean_max') or ['', ''])[0]} | {verdict(s, net, cpu)} |")


if __name__ == '__main__':
    main()

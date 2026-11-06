#!/usr/bin/env python3
"""Rows 7/8's table: one line per mission run (run_mission.py + truth.py), so
that two builds can be read side by side. Host side; development only.

    mission_table.py RUN_DIR [RUN_DIR ...] [--json OUT]

Per run: how it ended, the checkpoint's segments and progress at the end,
coverage of the plan, trips (safety-stop episodes) per minute, segment
restarts from the segment's start and resumes inside it, pauses (the gate
closed but the goal stayed), truth.py's tracking, and the VM stall canary
(vmload.py), so that a run a neighbour disturbed can be told apart.
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from truth import log_events  # noqa: E402

SEGMENT = re.compile(r'執行第 (\d+)/(\d+) 段')
RESUME_INSIDE = re.compile(r'續割：從第 (\d+) 段 ([0-9.]+) m')
SHORT = {
    'GPS odometry': 'odom', 'GPS fix': 'fix', 'GPS /fix': 'fix', 'robot pose': 'pose', 'IMU': 'imu',
}


def short_reason(why):
    for k, v in SHORT.items():
        if why.startswith(k):
            return v + ('@stamp' if 'source timestamp' in why else '')
    return why[:24]


def row(run):
    s = json.load(open(os.path.join(run, 'mission_summary.json')))
    m = json.load(open(os.path.join(run, 'client', 'mission.json')))
    a = json.load(open(os.path.join(run, 'run.json')))['args'] if os.path.exists(os.path.join(run, 'run.json')) else {}
    starts = {}
    for _, line in log_events(run, '段'):
        g = SEGMENT.search(line)
        if g:
            starts[int(g.group(1))] = starts.get(int(g.group(1)), 0) + 1
    inside = sum(1 for _, line in log_events(run, '續割') if RESUME_INSIDE.search(line))
    restarts = max(0, sum(n - 1 for n in starts.values()) - inside)
    pauses = len(log_events(run, 'Navigation safety pause:'))
    failed = len(log_events(run, '失敗'))
    uncertain = len(log_events(run, 'cancel confirmation timed out'))
    polls = m['polls']
    last = polls[-1] if polls else [0, {}, {}]
    dur_min = max(m['duration_s'], 1.0) / 60.0
    trips = s.get('safety_stop_episodes', 0)
    reasons = {}
    for why, n in (s.get('safety_stop_reasons') or {}).items():
        k = short_reason(why)
        reasons[k] = reasons.get(k, 0) + n
    t0, t1 = m['t_start'], m['t_start'] + m['duration_s']
    return {
        'run': os.path.basename(run.rstrip('/')),
        'cpu': a.get('cpu'), 'navcore': a.get('navcore'), 'seed': a.get('seed'), 'faults': a.get('faults'),
        'end': m['end'], 'duration_s': m['duration_s'], 'running_s': s.get('running_s'),
        'segments_done': (last[1].get('seg') or [None, None, None])[1], 'segments': (last[1].get('seg') or [None, None, None])[2],
        'overall_pct': round(100.0 * (last[1].get('overall') or 0.0), 1),
        'coverage_of_plan_pct': s.get('coverage_of_plan_pct'),
        'trips': trips, 'trips_per_min': round(trips / dur_min, 2), 'trips_with_vm_stall': s.get('safety_stop_episodes_with_vm_stall'),
        'reasons': reasons, 'resumes': s.get('resumes'), 'segment_restarts': restarts, 'resumed_inside': inside,
        'cross_track_p50_p95_max': s.get('cross_track_m_p50_p95_max'), 'excursions_gt_0p2': s.get('excursions_gt_0p2'),
        'vm_stalls': s.get('vm_stalls'), 'vm_busy_mean_max': s.get('vm_busy_pct_mean_max'), 't0': t0, 't1': t1,
        'pauses': pauses, 'task_failures': failed, 'cancel_unconfirmed': uncertain,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--json')
    args = ap.parse_args()
    rows = []
    for run in args.runs:
        try:
            rows.append(row(run))
        except Exception as e:  # noqa: BLE001
            print(f'{run}: {type(e).__name__}: {e}', file=sys.stderr)
    print('| run | end | s | segs | overall % | plan cov % | trips | /min | pauses | restarts | inside | cross-track p50/p95/max | >0.2 | reasons | stalls n/max |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        ct = r['cross_track_p50_p95_max'] or ['', '', '']
        vs = r['vm_stalls'] or {}
        reasons = ' '.join(f'{k}:{v}' for k, v in sorted(r['reasons'].items(), key=lambda kv: -kv[1]))
        print(f"| {r['run']} | {r['end']} | {r['duration_s']:.0f} | {r['segments_done']}/{r['segments']} | {r['overall_pct']} | "
              f"{r['coverage_of_plan_pct']} | {r['trips']} | {r['trips_per_min']} | {r['pauses']} | {r['segment_restarts']} | {r['resumed_inside']} | "
              f"{ct[0]}/{ct[1]}/{ct[2]} | {r['excursions_gt_0p2']} | {reasons} | {vs.get('n')}/{vs.get('max_s')} |")
    if args.json:
        json.dump(rows, open(args.json, 'w'), indent=1)


if __name__ == '__main__':
    main()

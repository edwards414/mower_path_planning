#!/usr/bin/env python3
"""Inter-arrival gaps and stamp ages of the topics in a harness run, from the
unthrottled observer's receipts, to size a budget against what a source does.
Host side; development only.

    gaps.py RUN_DIR [--file obs/mission.jsonl] [--topic /fix ...] [--over 0.2,0.25,...]
            [--t0 T --t1 T]

Per topic: messages, gaps p50/p99/max, how many gaps exceed each limit (per
minute), and the stamp's age at receipt p50/p99/max. The observer sits behind
the bus/DDS bridge in the same throttled container, so a gap here is at least
what the module in the stack saw, give or take the bridge's own delay.
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from teleop_analyze import load_jsonl, pct  # noqa: E402

DEFAULT = {'obs/mission.jsonl': ['/fix', '/adapter/robot_pose', '/odometry/global', '/imu/data', '/odom'],
           'obs/teleop.jsonl': ['/app_joy_cmd', '/joy_cmd', '/drivetrain_guarded_cmd_vel', '/manual_command_clock']}


def table(rows, topics, over):
    out = {}
    by = {}
    for d in rows:
        by.setdefault(d.get('k'), []).append(d)
    for topic in topics:
        rs = by.get(topic, [])
        ts = [d['t'] for d in rs]
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        mins = max((ts[-1] - ts[0]) / 60.0, 1e-6) if len(ts) > 1 else 1e-6
        ages = [d['t'] - d['s'] / 1e9 for d in rs if d.get('s')]
        out[topic] = {'n': len(rs), 'gap_s_p50_p99_max': [pct(gaps, .5), pct(gaps, .99), round(max(gaps), 3) if gaps else None],
                      'gaps_over_per_min': {str(x): round(sum(1 for g in gaps if g > x) / mins, 2) for x in over},
                      'age_s_p50_p99_max': [pct(ages, .5), pct(ages, .99), round(max(ages), 3) if ages else None]}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--file', default='obs/mission.jsonl')
    ap.add_argument('--topic', action='append')
    ap.add_argument('--over', default='0.2,0.25,0.3,0.35,0.4,0.5,0.6,0.75,1.0')
    ap.add_argument('--t0', type=float)
    ap.add_argument('--t1', type=float)
    args = ap.parse_args()
    over = [float(x) for x in args.over.split(',')]
    rows = load_jsonl(os.path.join(args.run, args.file))
    if args.t0 is not None:
        rows = [d for d in rows if d.get('t', 0) >= args.t0]
    if args.t1 is not None:
        rows = [d for d in rows if d.get('t', 0) <= args.t1]
    topics = args.topic or DEFAULT.get(args.file, sorted({d.get('k') for d in rows if d.get('k')}))
    print(json.dumps(table(rows, topics, over), indent=1))


if __name__ == '__main__':
    main()

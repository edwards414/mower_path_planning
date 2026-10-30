#!/usr/bin/env python3
"""Compare a fresh recording of the two node scenarios with the committed one.

usage: compare_scenarios.py COMMITTED_DIR FRESH_DIR

Both directories hold controller_server.json and navigator.json
(mower_nav_core's tests/data). The real controller_server and bt_navigator
run on the wall clock, so two recordings in the same image never agree line
for line. This compares only what a run of the real nodes does not change:
- the nav2 version, and the same trees and scenarios;
- per scenario the same goals, each with the same acceptance, status, error
  code and cancel answers, except a cancel that races a send (below);
- where the robot ends, within POSITION_TOLERANCE_M and
  HEADING_TOLERANCE_RAD.

A cancel event at the same `t` as a send event of its scenario (the
scenario script, tests/oracle/<node>/scenarios.json) races it: in
cancel_running_while_pending, goal 1 is sent and goal 0 cancelled at
t = 1.0, and controller_server accepts the cancel only when it handles it
before its next loop preempts goal 0 with goal 1. The goal ends the same
either way. Its answer is printed, not compared (two arm runs of run
41056625955 answered False, the committed recording and a Mac run True).

Result times, command counts, plan and follow_path counts and the recovery
count in the last feedback vary from run to run (the 7-round wall scenario
took 35.05 s, 37.04 s and 39.5 s on nav2 itself;
mower_nav_core/README.md). They are printed, never compared.

The port is held to the committed recording, on every field and with its own
tolerances, by its tests (controller_server_oracle, navigator_oracle) in
build.yml's mower_rs tests. With this check, the committed recording's
outcomes are what the pinned nav2 still does.

Exits 1 if a compared field differs, and says which.
"""

import json
import math
import os
import sys

POSITION_TOLERANCE_M = 0.05
HEADING_TOLERANCE_RAD = 0.1
GOAL_FIELDS = ('accepted', 'status', 'error_code')
ORACLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                      'src', 'mower_rs', 'crates', 'mower_nav_core', 'tests', 'oracle')
COUNTED = ('cmds', 'plans', 'compute_goals', 'follow_goals')


def scenarios(doc):
    """(name, scenario) of a controller_server or navigator recording."""
    if 'trees' in doc:
        for tree, by_name in doc['trees'].items():
            for name, s in by_name.items():
                yield f'{tree} {name}', s
    else:
        yield from doc['scenarios'].items()


def counts(s):
    out = []
    for key in COUNTED:
        if key in s:
            v = s[key]
            out.append(f'{key} {v if isinstance(v, int) else len(v)}')
    return out


def recoveries(goal):
    feedback = goal.get('feedback')
    return None if not feedback else feedback.get('number_of_recoveries')


def racing_cancels(script):
    """{scenario name: {goal: [the t of each of its cancels that shares its t with a send, else None]}}"""
    out = {}
    for s in script['scenarios']:
        sends = {e['t'] for e in s['events'] if e['op'] == 'send'}
        goals = {}
        for e in s['events']:
            if e['op'] == 'cancel':
                goals.setdefault(e['goal'], []).append(e['t'] if e['t'] in sends else None)
        out[s['name']] = goals
    return out


def compare_cancels(where, i, a, b, races, problems, drift):
    """cancel_accepted, one answer per cancel of the goal: a racing one is reported."""
    if a == b:
        return
    if a is None or b is None or len(a) != len(b):
        problems.append(f'{where} goal {i}: cancel_accepted {b!r}, committed {a!r}')
        return
    for k, (x, y) in enumerate(zip(a, b)):
        if x == y:
            continue
        t = races[k] if k < len(races) else None
        name = 'cancel_accepted' if len(a) == 1 else f'cancel_accepted[{k}]'
        if t is None:
            problems.append(f'{where} goal {i}: {name} {y!r}, committed {x!r}')
        else:
            drift.append(f'goal {i} {name} {y} (committed {x}; the cancel races a send at t={t})')


def compare(committed, fresh, label, races):
    problems, report = [], []
    for key in ('nav2', 'nav2_controller'):
        if committed.get(key) != fresh.get(key):
            problems.append(f'{label}: {key} {fresh.get(key)!r}, committed {committed.get(key)!r}')
    ours, theirs = dict(scenarios(committed)), dict(scenarios(fresh))
    if list(ours) != list(theirs):
        problems.append(f'{label}: scenarios {list(theirs)}, committed {list(ours)}')
    for name in ours:
        if name not in theirs:
            continue
        a, b = ours[name], theirs[name]
        where = f'{label} {name}'
        if len(a['goals']) != len(b['goals']):
            problems.append(f'{where}: {len(b["goals"])} goals, committed {len(a["goals"])}')
        drift = []
        for i, (ga, gb) in enumerate(zip(a['goals'], b['goals'])):
            for field in GOAL_FIELDS:
                if ga.get(field) != gb.get(field):
                    problems.append(f'{where} goal {i}: {field} {gb.get(field)!r}, committed {ga.get(field)!r}')
            compare_cancels(where, i, ga.get('cancel_accepted'), gb.get('cancel_accepted'),
                            races.get(name.split()[-1], {}).get(i, []), problems, drift)
            if 'result_t' in ga and 'result_t' in gb:
                drift.append(f'goal {i} at {gb["result_t"]:.3f} s ({gb["result_t"] - ga["result_t"]:+.3f})')
            if recoveries(ga) != recoveries(gb):
                drift.append(f'goal {i} recoveries {recoveries(gb)} (committed {recoveries(ga)})')
        (xa, ya, ha), (xb, yb, hb) = a['final_pose'], b['final_pose']
        off = math.hypot(xb - xa, yb - ya)
        turn = abs(math.remainder(hb - ha, math.tau))
        if off > POSITION_TOLERANCE_M or turn > HEADING_TOLERANCE_RAD:
            problems.append(f'{where}: ends at ({xb:.3f}, {yb:.3f}, {hb:.3f}), '
                            f'{off:.3f} m and {turn:.3f} rad from the committed end')
        fresh_counts, committed_counts = counts(b), counts(a)
        changed = [f'{f} (committed {c.split()[-1]})' for f, c in zip(fresh_counts, committed_counts) if f != c]
        report.append(f'  {where}: ends {off * 1000:.0f} mm off; ' + '; '.join(drift + changed))
    return problems, report


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__.split('\n\n')[1])
    committed_dir, fresh_dir = sys.argv[1:]
    problems = []
    for name in ('controller_server', 'navigator'):
        with open(os.path.join(committed_dir, f'{name}.json')) as f:
            committed = json.load(f)
        with open(os.path.join(fresh_dir, f'{name}.json')) as f:
            fresh = json.load(f)
        with open(os.path.join(ORACLE, name, 'scenarios.json')) as f:
            races = racing_cancels(json.load(f))
        p, report = compare(committed, fresh, name, races)
        problems += p
        print(f'{name}: the fresh recording against the committed one (times, counts and recoveries: report only)')
        print('\n'.join(report))
    if problems:
        print('the fresh recording differs from the committed one where a run of the real nodes does not:')
        print('\n'.join(f'  {p}' for p in problems))
        return 1
    print('every outcome, error code and end pose of the committed node scenarios reproduced')
    return 0


if __name__ == '__main__':
    sys.exit(main())

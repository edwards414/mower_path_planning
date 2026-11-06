#!/usr/bin/env python3
"""Row 2: each joystick command's fate, stop latencies and the blackout, from
one teleop run's files (outdoor.sh collect + the runner's blk.json).

    teleop_analyze.py RUN_DIR [--manual-timeout 0.2] [--final-timeout 0.2]

Fates, per move command the client sent:
  accepted         the manual guard forwarded it (/joy_cmd carried its
                   angular.z)
  stale            delivered, rejected: echoed clock older than max_input_age_s
  moved_backward   delivered, rejected: stamp behind the guard's barrier
  replay_blocked   delivered, rejected: a repeated stamp after the receipt
                   timeout stopped the motion ("replayed velocity cannot
                   resume stopped motion")
  other            delivered, rejected for another reason, or a reject the
                   replay did not predict
  not_delivered    never reached /app_joy_cmd (link down)
Rejection reasons come from replaying velocity_command_guard/src/core.rs on
the observer's receipt times of /app_joy_cmd (same kernel clock as the
guard; the replay's agreement with the forwarded set is reported).
Receipt timeouts are events (the guards' log lines, not rate-limited),
counted per guard. The ticks the app itself skipped (clock older than 0.8 s)
are counted apart: the app sends nothing then.

Wheels: the truth log's rpm (10 Hz) with the sim's 0.15 s first-order lag;
the moment the wheels were commanded to 0 is taken from the first decaying
sample (t - tau ln(rpm_before / rpm)).
"""
import argparse
import glob
import json
import math
import os
import re
import sys
from datetime import datetime

TAU = 0.15
MAX_AGE = 0.8
SKEW = 0.05
MAX_LX, MAX_AZ = 0.5, 1.0


def load_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def vm_busy(run, t0, t1):
    """The VM's CPU use while [t0, t1] ran (vmload.py), [mean, max] %."""
    for name in ('obs/vmload.jsonl', 'vmload.jsonl'):
        rows = [d['busy_pct'] for d in load_jsonl(os.path.join(run, name)) if 'busy_pct' in d and t0 <= d['t'] <= t1]
        if rows:
            return [round(sum(rows) / len(rows), 1), max(rows)]
    return None


def vm_stalls(run):
    """(time, seconds) of the canary's VM stalls (vmload.py)."""
    for name in ('obs/vmload.jsonl', 'vmload.jsonl'):
        rows = [(d['t'], d['stall_s']) for d in load_jsonl(os.path.join(run, name)) if 'stall_s' in d]
        if rows:
            return rows
    return []


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    return round(xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))], 3)


class GuardReplay:
    """velocity_command_guard core.rs on observed receipt times. keeps: the
    rejections that leave the barrier where it is: one behind the barrier
    always did, and a replay (and, in a variant, a stale one) may too."""

    def __init__(self, timeout, keeps=('moved_backward',)):
        self.keeps = keeps
        self.T = max(timeout, 0.05)
        self.period = min(0.05, self.T / 2)
        self.active = False
        self.last_rx = None
        self.barrier = None
        self.watchdog_fires = []

    def tick_to(self, t):
        if self.active and self.last_rx is not None and t > self.last_rx + self.T + self.period / 2:
            self.watchdog_fires.append(self.last_rx + self.T + self.period / 2)
            self.active = False
            self.last_rx = None

    def on(self, t, stamp_ns, lx, az, session_ok=True):
        self.tick_to(t)
        ros_ns = int(t * 1e9)
        reason = None
        is_new = False
        if stamp_ns is not None and stamp_ns > 0:
            age = (ros_ns - stamp_ns) / 1e9
            if age < -SKEW:
                reason = 'future'
            elif self.barrier is not None and stamp_ns < self.barrier:
                reason = 'moved_backward'
            else:
                is_new = self.barrier is None or stamp_ns > self.barrier
                if is_new:
                    self.barrier = stamp_ns
        moving = abs(lx) > 1e-9 or abs(az) > 1e-9
        if reason is None:
            if not session_ok:
                reason = 'session'
            elif abs(lx) > MAX_LX or abs(az) > MAX_AZ:
                reason = 'limit'
            elif (ros_ns - stamp_ns) / 1e9 > MAX_AGE:
                reason = 'stale'
            elif moving and not is_new:
                expired = self.last_rx is None or (t - self.last_rx) > self.T
                if not self.active or expired:
                    reason = 'replay_blocked'
        if reason is not None:
            if reason not in self.keeps and (self.barrier is None or ros_ns > self.barrier):
                self.barrier = ros_ns
            self.active = False
            self.last_rx = None
            return reason
        if not moving:
            self.active = False
            self.last_rx = None
        else:
            self.active = True
            if is_new:
                self.last_rx = t
        return None


def wheel_events(truth, t_from, t_to):
    """Truth samples in a window as (t, max |rpm|)."""
    return [(d['t'], max(abs(d['left_rpm']), abs(d['right_rpm']))) for d in truth if t_from <= d['t'] <= t_to]


def stop_time(truth, t_from, t_to, still=0.5):
    """When the wheels were commanded to 0 after t_from: the first sample
    whose rpm decays from a moving one, back-projected with the lag."""
    rows = wheel_events(truth, t_from - 0.3, t_to)
    prev = None
    for t, r in rows:
        if prev is not None and t >= t_from and prev[1] > 2.0 and r < prev[1] * 0.97:
            if r < still:
                return prev[0], 'bound'
            return t - TAU * math.log(prev[1] / r), 'decay'
        if prev is not None and t >= t_from and prev[1] <= still and r <= still:
            return (t_from, 'already')
        prev = (t, r)
    return None, 'moving'


def analyze(run, manual_t, final_t):
    client = json.load(open(os.path.join(run, 'client', 'teleop.json')))
    obs = load_jsonl(os.path.join(run, 'obs', 'teleop.jsonl'))
    truth = []
    for f in sorted(glob.glob(os.path.join(run, 'sim', 'truth.*.jsonl'))):
        truth += load_jsonl(f)
    truth.sort(key=lambda d: d['t'])
    blk = json.load(open(os.path.join(run, 'blk.json'))) if os.path.exists(os.path.join(run, 'blk.json')) else None

    by = {}
    for d in obs:
        by.setdefault(d.get('k'), []).append(d)
    app_in = by.get('/app_joy_cmd', [])
    joy_out = by.get('/joy_cmd', [])
    final_out = by.get('/drivetrain_guarded_cmd_vel', [])
    sessions = {d.get('f') for d in by.get('/manual_command_clock', [])}

    moves = [s for s in client['sends'] if s[0]]
    az_of = {round(s[5], 4): s for s in moves}
    delivered = {}
    for d in app_in:
        az = round(float(d['az']), 4)
        if az in az_of and az not in delivered:
            delivered[az] = d
    # forwarded = its angular.z on /joy_cmd, or downstream of it (twist_mux
    # and the final guard pass the values through): any of the three, so a
    # sample the best-effort observer missed on one topic still counts
    accepted = {}
    for topic in ('/joy_cmd', '/cmd_vel_guard_input', '/drivetrain_guarded_cmd_vel'):
        for d in by.get(topic, []):
            az = round(float(d['az']), 4)
            if az in az_of and abs(float(d['lx'])) > 0 and (az not in accepted or d['t'] < accepted[az]['t']):
                accepted[az] = d

    # the guard, replayed on every delivered frame (moves and zeros), with
    # each barrier rule: the one that explains the run names the fates
    best = None
    for keeps in (('moved_backward',), ('moved_backward', 'replay_blocked'), ('moved_backward', 'replay_blocked', 'stale')):
        g = GuardReplay(manual_t, keeps)
        predicted = {}
        for d in app_in:
            lx, az = float(d['lx']), float(d['az'])
            r = g.on(d['t'], d.get('s'), lx, az, session_ok=(d.get('f') in sessions) if sessions else True)
            if abs(lx) > 0:
                predicted[round(az, 4)] = r
        fates = {'accepted': 0, 'stale': 0, 'moved_backward': 0, 'replay_blocked': 0, 'other': 0, 'not_delivered': 0}
        agree = disagree = 0
        for s in moves:
            az = round(s[5], 4)
            if az in accepted:
                fates['accepted'] += 1
                agree += predicted.get(az, 'x') is None
                disagree += predicted.get(az, 'x') is not None
            elif az not in delivered:
                fates['not_delivered'] += 1
            else:
                r = predicted.get(az)
                if r in ('stale', 'moved_backward', 'replay_blocked'):
                    fates[r] += 1
                    agree += 1
                else:
                    fates['other'] += 1
                    disagree += r is None
        if best is None or agree > best[0]:
            best = (agree, disagree, fates, g, keeps)
    agree, disagree, fates, g, keeps = best

    final_zeros = [d['t'] for d in final_out if abs(float(d['lx'])) < 1e-9 and abs(float(d['az'])) < 1e-9]

    # receipt timeouts from the stack log, in the soak's window
    t0 = min(s[1] for s in client['sends']) - 1 if client['sends'] else 0
    t1 = max(s[1] for s in client['sends']) + 3 if client['sends'] else 0
    rt = {'manual_velocity_guard': 0, 'velocity_command_guard': 0}
    rej_lines = {}
    log_path = os.path.join(run, 'stack.log')
    if os.path.exists(log_path):
        for line in open(log_path, errors='replace'):
            if 'receipt timeout' not in line and 'Rejected drivetrain command' not in line:
                continue
            m = re.match(r'(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.(\d+)Z', line)
            if not m:
                continue
            ts = datetime.strptime(m.group(1), '%Y-%m-%dT%H:%M:%S').replace(tzinfo=__import__('datetime').timezone.utc).timestamp() + float('0.' + m.group(2))
            if not (t0 <= ts <= t1):
                continue
            node = 'manual_velocity_guard' if 'manual_velocity_guard' in line else 'velocity_command_guard'
            if 'receipt timeout' in line:
                rt[node] += 1
            else:
                reason = line.split('Rejected drivetrain command:', 1)[1].strip()
                rej_lines[f'{node}: {reason}'] = rej_lines.get(f'{node}: {reason}', 0) + 1

    # holds: duty, stop latencies
    ev = client['events']
    holds = []
    starts = [e for e in ev if e['e'] == 'hold_start']
    rels = [e for e in ev if e['e'] == 'release']
    for hs, rl in zip(starts, rels):
        tag = hs['tag']
        hm = [s for s in moves if hs['t'] - 0.01 <= s[1] <= rl['t'] + 0.01]
        acc = [accepted[round(s[5], 4)]['t'] for s in hm if round(s[5], 4) in accepted]
        sent_n = len(hm)
        # the client's own 100 ms tick slipping means the whole VM stalled
        # (Docker Desktop), not the stack: such holds are kept out of the
        # latency figures and counted
        times = [s[1] for s in client['sends'] if hs['t'] - 0.01 <= s[1] <= rl['t'] + 0.2]
        gap = max([b - a for a, b in zip(times, times[1:])] + [0.0])
        row = {'tag': tag, 'sent': sent_n, 'accepted': len(acc), 'host_stall': gap > 0.35 and tag != 'blk',
               'max_send_gap_s': round(gap, 3),
               'ticks_skipped': sum(1 for k in client['skipped'] if hs['t'] <= k[0] <= rl['t'] and k[1] == 'move')}
        if acc:
            t_last = max(acc)
            t_first = min(acc)
            tw = wheel_events(truth, t_first, rl['t'])
            row['duty'] = round(sum(1 for _, r in tw if r > 5.0) / max(1, len(tw)), 3)
            # dips: the wheels fell under half of the hold's speed and came
            # back, with the stick still held (what a driver feels); 10 Hz
            # samples, from 0.5 s in
            turning = sorted(r for _, r in tw)
            if turning:
                top = turning[int(0.9 * (len(turning) - 1))]
                inner = [r for t, r in tw if t >= t_first + 0.5]
                row['dips'] = sum(1 for a, b in zip(inner, inner[1:]) if a >= 0.5 * top > b)
            # the first zero the final guard sent after the last accepted command
            fz = next((t for t in final_zeros if t >= t_last), None)
            row['final_zero_after_last_accept_s'] = round(fz - t_last, 3) if fz else None
            ts, how = stop_time(truth, t_last, t_last + (40 if tag == 'blk' else 5))
            row['stop_after_last_accept_s'] = round(ts - t_last, 3) if ts else None
            row['stop_how'] = how
            row['stop_after_release_s'] = round(ts - rl['t'], 3) if ts and tag != 'blk' else None
            row['t_last_accept'] = round(t_last, 3)
        holds.append(row)

    soak = [h for h in holds if h['tag'].startswith('soak')]
    clean = [h for h in soak if not h['host_stall']]
    lat_acc = [h['stop_after_last_accept_s'] for h in clean if h.get('stop_after_last_accept_s') is not None]
    lat_rel = [h['stop_after_release_s'] for h in clean if h.get('stop_after_release_s') is not None]
    fz_acc = [h['final_zero_after_last_accept_s'] for h in clean if h.get('final_zero_after_last_accept_s') is not None]
    n_moves_soak = sum(h['sent'] for h in soak)
    out = {
        'run': os.path.basename(run.rstrip('/')),
        'manual_timeout_s': manual_t, 'final_timeout_s': final_t,
        'moves_sent': len(moves), 'fates': fates,
        'accepted_pct': round(100.0 * fates['accepted'] / max(1, len(moves)), 2),
        'soak_accepted_pct': round(100.0 * sum(h['accepted'] for h in soak) / max(1, n_moves_soak), 2),
        'app_ticks_skipped': len([k for k in client['skipped'] if k[1] == 'move']),
        'replay_agreement': round(agree / max(1, agree + disagree), 4),
        'receipt_timeouts': rt,
        'replay_watchdog_fires': len(g.watchdog_fires),
        'reject_log_lines_sampled': rej_lines,
        'soak_duty_mean': round(sum(h.get('duty', 0) for h in soak) / max(1, len(soak)), 3),
        'stop_after_release_s_p50_p95_max': [pct(lat_rel, 0.5), pct(lat_rel, 0.95), max(lat_rel) if lat_rel else None],
        'stop_after_last_accept_s_p50_p95_max': [pct(lat_acc, 0.5), pct(lat_acc, 0.95), max(lat_acc) if lat_acc else None],
        'final_zero_after_last_accept_s_p50_p95_max': [pct(fz_acc, 0.5), pct(fz_acc, 0.95), max(fz_acc) if fz_acc else None],
        'barrier_rule_keeps': list(keeps),
        'rejects_delivered': sum(fates[k] for k in ('stale', 'moved_backward', 'replay_blocked', 'other')),
        'rejects_per_min': round(sum(fates[k] for k in ('stale', 'moved_backward', 'replay_blocked', 'other')) / max(len(moves) / 600.0, 1e-6), 1),
        'receipt_timeouts_per_min': {k: round(v / max(len(moves) / 600.0, 1e-6), 1) for k, v in rt.items()},
        'soak_dips_per_min': round(sum(h.get('dips', 0) for h in soak) / max(n_moves_soak / 600.0, 1e-6), 1),
        'stop_bound_s': round(min(manual_t, final_t) + 0.1, 3),
        'stop_bound_violations': sum(1 for x in lat_acc if x > manual_t + 0.1),
        'connections': client['connections'],
        'vm_busy_pct_mean_max': vm_busy(run, t0, t1),
        'host_stall_holds': sum(1 for h in soak if h['host_stall']),
        'stop_after_last_accept_s_incl_stalls_max': max([h['stop_after_last_accept_s'] for h in soak
                                                         if h.get('stop_after_last_accept_s') is not None] or [None]),
        'holds': holds,
    }
    if blk:
        tb = blk['t_blk']
        moving = [t for t, r in wheel_events(truth, tb, tb + 30) if r > 0.5]
        cmd_moving = [d['t'] for d in final_out if tb <= d['t'] <= tb + 30 and abs(float(d['lx'])) > 0]
        acc_in = [d['t'] for d in joy_out if tb <= d['t'] <= tb + 30 and abs(float(d['lx'])) > 0]
        ts, how = stop_time(truth, tb, tb + 30)
        back = [e for e in ev if e['e'] == 'connected' and e['t'] > tb]
        # the link: the last command the guards took before the blackout, to
        # the first zero the final guard sent after it
        last_in = max([d['t'] for d in joy_out if d['t'] <= tb + 0.1 and abs(float(d['lx'])) > 0] or [None], key=lambda x: x or 0)
        zero_after = next((t for t in final_zeros if last_in and t >= last_in), None)
        after = [h for h in holds if h['tag'].startswith('after')]
        out['blackout'] = {
            't_blk': tb,
            'wheels_commanded_zero_after_blk_s': round(ts - tb, 3) if ts else None, 'how': how,
            'final_zero_after_last_command_s': round(zero_after - last_in, 3) if zero_after else None,
            'final_zero_after_blk_s': round(zero_after - tb, 3) if zero_after else None,
            'last_rpm_above_0p5_after_blk_s': round(max(moving) - tb, 3) if moving else 0.0,
            'last_final_guard_nonzero_after_blk_s': round(max(cmd_moving) - tb, 3) if cmd_moving else 0.0,
            'accepted_after_blk': len(acc_in),
            'pass_0p3s': (ts is None and not moving) or (ts is not None and ts - tb <= 0.3),
            'reconnected_after_unblk_s': round(back[0]['t'] - blk['t_unblk'], 2) if back and blk.get('t_unblk') else None,
            'after_holds_accepted': [[h['accepted'], h['sent']] for h in after],
        }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--manual-timeout', type=float, default=0.20)
    ap.add_argument('--final-timeout', type=float, default=0.20)
    args = ap.parse_args()
    out = analyze(args.run, args.manual_timeout, args.final_timeout)
    with open(os.path.join(args.run, 'teleop_summary.json'), 'w') as f:
        json.dump(out, f, indent=1)
    brief = {k: v for k, v in out.items() if k != 'holds'}
    print(json.dumps(brief, indent=1))


if __name__ == '__main__':
    main()

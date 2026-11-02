#!/usr/bin/env python3
"""Seeded fault schedules for the outdoor harness (I4). Runs in the observer
sidecar and writes mower_sim's control files through the shared volume
(/ctl = the sim's /tmp/mower_sim), never by an exec into the throttled sim.
Every switch is one JSON line in --log (wall time, what, on/off or mode).
Development only.

    faults.py PROFILE[,PROFILE...] --log /tmp/obs/faults.jsonl [--seed 1]
              [--start 10] [--duration 600] [--ctl /ctl]

Profiles (PLAN.md "Stress profiles"):
  G1      gps_mode fixed 60 s, float 30, none 15, nosat 20, 3d 10, float 15,
          fixed (once, from --start)
  G2      fixed/float flapping, each state U(2, 5) s, for --duration
  G3      nosat for 120 s, then fixed
  F-hub   hub_gone (IMU and F9P unplugged) for U(1.5, 2.5) s every U(15, 60) s
  F-imu   imu_gone, the same timing
  F-gps   gps_gone, the same timing
  F-stall sensors_mute (a stall, not an unplug), the same timing
  F-mix   F-hub, F-imu and F-gps drawn at random each time
  R3      row 3: F-hub / F-imu / F-gps drawn at random, with an F-stall as
          the 5th, 14th, 23rd, 32nd and 41st event; never two at once
  G23     G2 for 180 s, then G3 (row 4's degraded drive in one run)
  G-boot-fixed  gps_mode fixed at --start (G-boot: the sim boots in 3d)
  HUB1    one hub reset of 2.0 s at --start (row 5, mid-record)
--interval A,B replaces U(15, 60) between events (row 3 uses 8,20 to get
about 45 flaps into 12 minutes instead of 30).
nosat, imu_gone, gps_gone and hub_gone need a mower_sim with the new
control interface (`mower_sim --help` lists --reacquire); an older one
ignores them (its log says so).
"""
import argparse
import json
import os
import random
import time

FLAGS = {'F-hub': 'hub_gone', 'F-imu': 'imu_gone', 'F-gps': 'gps_gone', 'F-stall': 'sensors_mute'}


def schedule(profile, rng, start, duration, gap=(15, 60)):
    ev = []  # (t_rel, kind, value)
    if profile == 'G1':
        t = start
        for mode, s in (('fixed', 60), ('float', 30), ('none', 15), ('nosat', 20), ('3d', 10), ('float', 15)):
            ev.append((t, 'gps_mode', mode))
            t += s
        ev.append((t, 'gps_mode', 'fixed'))
    elif profile == 'G2':
        t, mode = start, 'float'
        while t < start + duration:
            ev.append((t, 'gps_mode', mode))
            t += rng.uniform(2, 5)
            mode = 'fixed' if mode == 'float' else 'float'
        ev.append((t, 'gps_mode', 'fixed'))
    elif profile == 'G3':
        ev += [(start, 'gps_mode', 'nosat'), (start + 120, 'gps_mode', 'fixed')]
    elif profile == 'HUB1':
        ev += [(start, 'hub_gone', True), (start + 2.0, 'hub_gone', False)]
    elif profile == 'G-boot-fixed':
        ev.append((start, 'gps_mode', 'fixed'))
    elif profile == 'G23':
        ev += [e for e in schedule('G2', rng, start, 180) if e[0] < start + 180]
        ev += [(start + 180, 'gps_mode', 'fixed')] + schedule('G3', rng, start + 200, 0)
    elif profile == 'R3':
        t, n = start + rng.uniform(*gap), 0
        while t < start + duration:
            n += 1
            flag = 'sensors_mute' if n in (5, 14, 23, 32, 41) else rng.choice(['hub_gone', 'imu_gone', 'gps_gone'])
            out = rng.uniform(1.5, 2.5)
            ev += [(t, flag, True), (t + out, flag, False)]
            t += out + rng.uniform(*gap)
    elif profile in FLAGS or profile == 'F-mix':
        t = start + rng.uniform(*gap)
        while t < start + duration:
            flag = FLAGS[profile] if profile != 'F-mix' else rng.choice(['hub_gone', 'imu_gone', 'gps_gone'])
            out = rng.uniform(1.5, 2.5)
            ev += [(t, flag, True), (t + out, flag, False)]
            t += out + rng.uniform(*gap)
    else:
        raise SystemExit(f'unknown profile {profile}')
    return ev


def apply(ctl, kind, value):
    path = os.path.join(ctl, kind)
    if kind == 'gps_mode':
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(value + '\n')
        os.chmod(tmp, 0o644)
        os.replace(tmp, path)
    elif value:
        open(path, 'w').close()
    else:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('profiles')
    ap.add_argument('--log', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--start', type=float, default=10.0)
    ap.add_argument('--duration', type=float, default=600.0)
    ap.add_argument('--ctl', default='/ctl')
    ap.add_argument('--interval', default='15,60')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    rng = random.Random(args.seed)
    ev = []
    for p in args.profiles.split(','):
        gap = tuple(float(x) for x in args.interval.split(','))
        ev += [(t, k, v, p) for t, k, v in schedule(p, rng, args.start, args.duration, gap)]
    ev.sort(key=lambda e: e[0])
    os.makedirs(os.path.dirname(args.log) or '.', exist_ok=True)
    log = open(args.log, 'a', buffering=1)
    t0 = time.time()
    log.write(json.dumps({'t': t0, 'start': True, 'profiles': args.profiles, 'seed': args.seed,
                          'plan': [[round(t, 2), k, v] for t, k, v, _ in ev]}) + '\n')
    if args.dry_run:
        print(json.dumps([[round(t, 2), k, v] for t, k, v, _ in ev]))
        return
    for t, kind, value, profile in ev:
        d = t0 + t - time.time()
        if d > 0:
            time.sleep(d)
        apply(args.ctl, kind, value)
        log.write(json.dumps({'t': round(time.time(), 4), 'k': kind, 'v': value, 'p': profile}) + '\n')
    log.write(json.dumps({'t': round(time.time(), 4), 'end': True}) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Ground-truth metrics for the outdoor harness (I9), from mower_sim's truth
log (10 Hz: x, y, yaw, wheel rpm, lat/lon) and the observer's recordings.
Development only; host side (python3 + numpy).

    truth.py mission RUN_DIR [--zone 1]   coverage, cross-track, excursions,
                                          stops at gate closures, resumes
    truth.py loc RUN_DIR                  pose and heading error by GNSS mode,
                                          recovery after fixed returns, NaN
                                          poses, the datum against the truth

The map frame is reached from the truth's lat/lon through the stack's own
datum (/adapter/map_datum: origin and the bearing of map +X), as the app
overlays it; the datum's own error against the truth start is reported apart.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from teleop_analyze import load_jsonl, pct, stop_time, vm_busy, vm_stalls  # noqa: E402

R_EARTH = 6378137.0
REPO = os.path.dirname(os.path.dirname(HERE))


def truth_rows(run):
    rows = []
    for f in sorted(glob.glob(os.path.join(run, 'sim', 'truth.*.jsonl'))):
        rows += load_jsonl(f)
    rows.sort(key=lambda d: d['t'])
    return rows


def obs_by(run, name):
    by = {}
    for d in load_jsonl(os.path.join(run, 'obs', name)):
        by.setdefault(d.get('k'), []).append(d)
    return by


def datum_of(by):
    ds = [d['j'] for d in by.get('/adapter/map_datum', []) if isinstance(d.get('j'), dict)]
    return ds[-1] if ds else None


def to_map(lat, lon, datum):
    lat0, lon0, b = datum['origin_lat'], datum['origin_lon'], datum['bearing_rad']
    e = np.radians(np.asarray(lon) - lon0) * math.cos(math.radians(lat0)) * R_EARTH
    n = np.radians(np.asarray(lat) - lat0) * R_EARTH
    return e * math.sin(b) + n * math.cos(b), -e * math.cos(b) + n * math.sin(b)


def seg_dist(px, py, path):
    """Distance from points to a polyline."""
    p = np.asarray(path, dtype=float)
    a, b = p[:-1], p[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    best = np.full(len(px), np.inf)
    for i in range(0, len(a), 256):
        A, AB, l2 = a[i:i + 256], ab[i:i + 256], L2[i:i + 256]
        dx = px[:, None] - A[None, :, 0]
        dy = py[:, None] - A[None, :, 1]
        t = np.clip((dx * AB[None, :, 0] + dy * AB[None, :, 1]) / l2[None, :], 0, 1)
        ex = dx - t * AB[None, :, 0]
        ey = dy - t * AB[None, :, 1]
        best = np.minimum(best, np.sqrt(ex * ex + ey * ey).min(1))
    return best


def in_poly(x, y, poly):
    poly = np.asarray(poly)[:, :2]
    inside = np.zeros(x.shape, bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        inside ^= cond
        j = i
    return inside


def swept(gx, gy, path, half):
    out = np.zeros(gx.shape, bool)
    p = np.asarray(path, float)
    for i in range(len(p) - 1):
        (x0, y0), (x1, y1) = p[i], p[i + 1]
        m = (gx >= min(x0, x1) - half) & (gx <= max(x0, x1) + half) & (gy >= min(y0, y1) - half) & (gy <= max(y0, y1) + half)
        if not m.any():
            continue
        dx, dy = x1 - x0, y1 - y0
        l2 = max(dx * dx + dy * dy, 1e-12)
        sx, sy = gx[m] - x0, gy[m] - y0
        t = np.clip((sx * dx + sy * dy) / l2, 0, 1)
        out[m] |= (sx - t * dx) ** 2 + (sy - t * dy) ** 2 <= half * half
    return out


def zone_poly(zone):
    for path in (os.path.join(REPO, 'deploy', 'sim', 'fixtures', 'zone_record', 'zone_list.json'),):
        for z in json.load(open(path)):
            if z['id'] == zone:
                return z['points']
    return None


def layer_path(by, zone):
    """The zone's planned path from the app's coverage-path layer (the last
    /adapter/marker_layers/coverage_path): its markers carry no zone id, so
    the one with the most points inside the zone's polygon."""
    docs = [d['j'] for d in by.get('/adapter/marker_layers/coverage_path', []) if isinstance(d.get('j'), dict)]
    poly = zone_poly(zone)
    if not docs or not poly:
        return None
    best, score = None, 0
    for mk in docs[-1].get('markers', []):
        pts = [[p['x'], p['y']] if isinstance(p, dict) else [p[0], p[1]] for p in mk.get('points', [])]
        if len(pts) < 2:
            continue
        a = np.asarray(pts)
        n = int(in_poly(a[:, 0], a[:, 1], poly).sum())
        if n > score:
            best, score = pts, n
    return best


def running_windows(polls):
    """[start, end] wall times where /coverage_progress_status said running."""
    wins, cur = [], None
    for t, p, _ in polls:
        if p.get('status') == 2:
            cur = [t, t] if cur is None else [cur[0], t]
        elif cur is not None:
            wins.append(cur)
            cur = None
    if cur:
        wins.append(cur)
    return wins


def safety_stops(run, out):
    """Every safety stop the nav server logged (its own ROS stamp), grouped
    into episodes (2 s apart), each marked when the VM stall canary fired
    within 1 s before it."""
    import re
    stalls = vm_stalls(run)
    eps = []
    for _, line in log_events(run, 'Navigation safety stop:'):
        mm = re.search(r'\[(\d+\.\d+)\] \[nav_action_server\]: Navigation safety stop: (.*)$', line)
        if not mm:
            continue
        ts, why = float(mm.group(1)), mm.group(2)
        if eps and ts - eps[-1]['t_last'] < 2.0:
            eps[-1]['t_last'] = ts
            eps[-1]['why'].add(why)
        else:
            eps.append({'t': ts, 't_last': ts, 'why': {why}})
    for e in eps:
        e['vm_stall'] = any(e['t'] - 1.0 <= t <= e['t'] + 0.2 for t, _ in stalls)
        e['why'] = sorted(e['why'])
    out['safety_stop_episodes'] = len(eps)
    out['safety_stop_episodes_with_vm_stall'] = sum(1 for e in eps if e['vm_stall'])
    out['safety_stop_reasons'] = {}
    for e in eps:
        for w in e['why']:
            out['safety_stop_reasons'][w] = out['safety_stop_reasons'].get(w, 0) + 1
    out['vm_stalls'] = {'n': len(stalls), 'max_s': max([x for _, x in stalls] or [0])}


def mission(run, zone):
    m = json.load(open(os.path.join(run, 'client', 'mission.json')))
    by = obs_by(run, 'mission.jsonl')
    truth = truth_rows(run)
    zp = json.load(open(os.path.join(run, 'obs', 'zone_paths.json')))
    datum = datum_of(by)
    out = {'run': os.path.basename(run.rstrip('/')), 'end': m['end'], 'duration_s': m['duration_s'],
           'resumes': m['resumes'], 'datum': datum}
    safety_stops(run, out)
    out['vm_busy_pct_mean_max'] = vm_busy(run, m['t_start'], m['t_start'] + m['duration_s'])
    path = (zp['zones'].get(str(zone)) or {}).get('path') or layer_path(by, zone)
    out['path_points'] = len(path or [])
    if not datum or not path or len(path) < 2 or not truth:
        out['error'] = 'no datum, path or truth'
        return out
    half = (zp.get('strip_width_m') or 0.3) / 2.0
    tt = np.array([d['t'] for d in truth])
    tx, ty = to_map([d['lat'] for d in truth], [d['lon'] for d in truth], datum)
    wins = running_windows(m['polls'])
    sel = np.zeros(len(tt), bool)
    for a, b in wins:
        sel |= (tt >= a) & (tt <= b)
    out['running_s'] = round(sum(b - a for a, b in wins), 1)
    if sel.sum() < 5:
        out['error'] = 'no running samples'
        return out
    ct = seg_dist(tx[sel], ty[sel], path)
    out['cross_track_m_p50_p95_max'] = [round(float(np.percentile(ct, 50)), 3), round(float(np.percentile(ct, 95)), 3),
                                        round(float(ct.max()), 3)]
    over = ct > 0.2
    out['excursions_gt_0p2'] = int(((over[1:]) & (~over[:-1])).sum() + (1 if over[0] else 0))
    out['samples_gt_0p2'] = int(over.sum())
    # coverage on a 5 cm grid
    poly = zone_poly(zone)
    xs = np.arange(min(p[0] for p in poly) - 0.5, max(p[0] for p in poly) + 0.5, 0.05)
    ys = np.arange(min(p[1] for p in poly) - 0.5, max(p[1] for p in poly) + 0.5, 0.05)
    gx, gy = np.meshgrid(xs, ys)
    zmask = in_poly(gx, gy, poly)
    plan = swept(gx, gy, path, half) & zmask
    traj = list(zip(tx[sel], ty[sel]))
    # split the driven trajectory where the running windows break
    cov = np.zeros(gx.shape, bool)
    idx = np.where(sel)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    start = 0
    for b in list(breaks) + [len(idx) - 1]:
        seg = [(tx[i], ty[i]) for i in idx[start:b + 1]]
        if len(seg) > 1:
            cov |= swept(gx, gy, seg, half)
        start = b + 1
    out['strip_width_m'] = round(2 * half, 3)
    out['coverage_zone_pct'] = round(100.0 * (cov & zmask).sum() / max(1, zmask.sum()), 2)
    out['plan_zone_pct'] = round(100.0 * plan.sum() / max(1, zmask.sum()), 2)
    out['coverage_of_plan_pct'] = round(100.0 * (cov & plan).sum() / max(1, plan.sum()), 2)
    del traj
    # gate closures: each fault switching on while the mission ran
    faults = load_jsonl(os.path.join(run, 'obs', 'faults.jsonl'))
    stops = []
    stop_topic = by.get('/navigation_safety_stop', [])
    for f in faults:
        if 'k' not in f or not (f['v'] is True or f['v'] in ('none', 'nosat', '3d', 'float')):
            continue
        tf = f['t']
        if not any(a - 1 <= tf <= b + 1 for a, b in wins):
            continue
        moving = [r for t, r in [(d['t'], max(abs(d['left_rpm']), abs(d['right_rpm']))) for d in truth] if tf - 0.25 <= t <= tf]
        gate = [d['t'] for d in stop_topic if tf <= d['t'] <= tf + 5]
        ts, how = stop_time(truth, tf, tf + 6)
        stops.append({'t': tf, 'fault': f['k'], 'v': f['v'], 'was_moving': bool(moving and max(moving) > 2),
                      'gate_closed_after_s': round(gate[0] - tf, 3) if gate else None,
                      'wheels_zero_after_fault_s': round(ts - tf, 3) if ts else None,
                      'wheels_zero_after_gate_s': round(ts - gate[0], 3) if ts and gate else None, 'how': how})
    out['fault_stops'] = stops
    out['resume_events'] = [e for e in m['events'] if e['e'] == 'resume']
    out['last_poll'] = m['polls'][-1] if m['polls'] else None
    return out


def interp(t, tt, v):
    return np.interp(t, tt, v)


def loc(run, obs_name='loc.jsonl'):
    by = obs_by(run, obs_name)
    truth = truth_rows(run)
    datum = datum_of(by)
    out = {'run': os.path.basename(run.rstrip('/')), 'datum': datum}
    if not datum or not truth:
        out['error'] = 'no datum or truth'
        return out
    tt = np.array([d['t'] for d in truth])
    tx, ty = to_map([d['lat'] for d in truth], [d['lon'] for d in truth], datum)
    # map +X is at compass bearing b, so map yaw = ENU yaw - (pi/2 - b)
    rot = math.pi / 2 - datum['bearing_rad']
    tyaw = np.unwrap(np.radians([d['yaw_deg'] for d in truth])) - rot
    est = [d for d in by.get('/odometry/global', []) if isinstance(d.get('x'), float)]
    et = np.array([d['t'] for d in est])
    ex, ey, eyaw = (np.array([d[k] for d in est]) for k in ('x', 'y', 'yaw'))
    err = np.hypot(ex - interp(et, tt, tx), ey - interp(et, tt, ty))
    herr = np.abs((eyaw - interp(et, tt, tyaw) + math.pi) % (2 * math.pi) - math.pi)
    # GNSS mode at each estimate, from the fault log (default fixed)
    faults = [f for f in load_jsonl(os.path.join(run, 'obs', 'faults.jsonl')) if f.get('k') == 'gps_mode']
    mode = np.array(['fixed'] * len(et), dtype=object)
    for f in faults:
        mode[et >= f['t']] = f['v'].split(':')[0]
    gone = [f for f in load_jsonl(os.path.join(run, 'obs', 'faults.jsonl')) if f.get('k') in ('imu_gone', 'hub_gone', 'sensors_mute', 'gps_gone')]
    out['pos_err_m_by_mode'] = {}
    for md in sorted(set(mode)):
        s = err[mode == md]
        if len(s):
            out['pos_err_m_by_mode'][md] = {'n': int(len(s)), 'p50': round(float(np.percentile(s, 50)), 3),
                                            'p95': round(float(np.percentile(s, 95)), 3), 'max': round(float(s.max()), 3)}
    rec = []
    for i, f in enumerate(faults):
        if f['v'].split(':')[0] != 'fixed' or i == 0:
            continue
        after = (et >= f['t'])
        ok = np.where(after & (err <= 0.05))[0]
        # sustained: the first time after which 2 s stay under 5 cm
        t_ok = None
        for j in ok:
            w = (et >= et[j]) & (et <= et[j] + 2.0)
            if (err[w] <= 0.05).all():
                t_ok = et[j]
                break
        rec.append({'t': f['t'], 'back_to_5cm_s': round(t_ok - f['t'], 2) if t_ok else None})
    out['recovery_after_fixed'] = rec
    hg = []
    for f in gone:
        if f['v'] is False and f['k'] in ('imu_gone', 'hub_gone', 'sensors_mute'):
            w = (et >= f['t']) & (et <= f['t'] + 5)
            if w.any():
                hg.append(round(float(np.degrees(herr[w].max())), 3))
    out['heading_err_deg_after_imu_gap_max'] = hg
    out['heading_err_deg_p95'] = round(float(np.degrees(np.percentile(herr, 95))), 3)
    poses = by.get('/adapter/robot_pose', [])
    out['app_pose_nonfinite'] = sum(1 for d in poses if not all(isinstance(d.get(k), float) for k in ('x', 'y', 'yaw')))
    out['app_pose_n'] = len(poses)
    t0 = truth[0]
    dx, dy = to_map([t0['lat']], [t0['lon']], datum)
    out['datum_vs_truth_start'] = {'start_in_map_m': [round(float(dx[0]), 3), round(float(dy[0]), 3)],
                                   'bearing_minus_east_rad': round(datum['bearing_rad'] - math.pi / 2, 4)}
    return out


def log_events(run, needle):
    """(time, line) of stack.log lines containing needle (docker logs -t)."""
    import re
    from datetime import datetime, timezone
    out = []
    path = os.path.join(run, 'stack.log')
    if not os.path.exists(path):
        return out
    for line in open(path, errors='replace'):
        if needle not in line:
            continue
        m = re.match(r'(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.(\d+)Z', line)
        if m:
            t = datetime.strptime(m.group(1), '%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc).timestamp()
            out.append((t + float('0.' + m.group(2)), line.strip()))
    return out


def reconnect(run, obs_name):
    """Row 3: each unplug/stall, how fast /imu/data and /fix came back."""
    by = obs_by(run, obs_name)
    imu = np.array([d['t'] for d in by.get('/imu/data', [])])
    fix = by.get('/fix', [])
    ft = np.array([d['t'] for d in fix])
    fixed_t = np.array([d['t'] for d in fix if d.get('st') == 2])
    faults = load_jsonl(os.path.join(run, 'obs', 'faults.jsonl'))
    on = {}
    events = []
    for f in faults:
        if f.get('k') not in ('imu_gone', 'gps_gone', 'hub_gone', 'sensors_mute'):
            continue
        if f['v'] is True:
            on[f['k']] = f['t']
        elif f['k'] in on:
            events.append((on.pop(f['k']), f['t'], f['k']))
    restarts = log_events(run, '[mower_rsd] restarting')
    rows = []
    for i, (t_on, t_off, kind) in enumerate(events):
        nxt = events[i + 1][0] if i + 1 < len(events) else t_off + 60
        r = {'t_on': round(t_on, 2), 'kind': kind, 'outage_s': round(t_off - t_on, 2)}
        if kind in ('imu_gone', 'hub_gone', 'sensors_mute'):
            a = imu[imu > t_off]
            # the gap must have shown: no sample in the middle of the outage
            r['imu_gap_seen'] = not ((imu > t_on + 0.5) & (imu < t_off - 0.2)).any()
            r['imu_back_s'] = round(float(a[0] - t_off), 2) if len(a) and a[0] - t_off < 30 else None
        if kind in ('gps_gone', 'hub_gone', 'sensors_mute'):
            a = ft[ft > t_off]
            r['fix_back_s'] = round(float(a[0] - t_off), 2) if len(a) and a[0] - t_off < 30 else None
            b = fixed_t[fixed_t > t_off]
            r['fixed_back_s'] = round(float(b[0] - t_off), 2) if len(b) and b[0] < nxt else None
        allowed = math.ceil((t_off - t_on) / 2.0) + 1
        for mod in ('imu', 'gps'):
            n = sum(1 for t, line in restarts if t_on <= t < nxt and f'restarting {mod}/' in line)
            r[f'restarts_{mod}'] = n
            r[f'restarts_{mod}_over'] = n > allowed
        rows.append(r)
    sim_log = ''.join(open(f, errors='replace').read() for f in sorted(glob.glob(os.path.join(run, 'sim', 'sim.*.log'))))
    gps_replugs = sum(1 for e in events if e[2] in ('gps_gone', 'hub_gone'))

    def stats(key, kinds):
        xs = [r[key] for r in rows if r['kind'] in kinds and r.get(key) is not None]
        miss = sum(1 for r in rows if r['kind'] in kinds and key in r and r[key] is None)
        return {'n': len(xs), 'missing': miss, 'p50': pct(xs, 0.5), 'p95': pct(xs, 0.95), 'max': max(xs) if xs else None}
    unplug = ('imu_gone', 'gps_gone', 'hub_gone')
    out = {
        'run': os.path.basename(run.rstrip('/')),
        'events': len(events), 'by_kind': {k: sum(1 for e in events if e[2] == k) for k in sorted({e[2] for e in events})},
        'imu_back_s_unplug': stats('imu_back_s', unplug), 'fix_back_s_unplug': stats('fix_back_s', unplug),
        'imu_back_s_stall': stats('imu_back_s', ('sensors_mute',)), 'fix_back_s_stall': stats('fix_back_s', ('sensors_mute',)),
        'fixed_back_s_gps': stats('fixed_back_s', ('gps_gone', 'hub_gone')),
        'recovered_pct': round(100.0 * sum(1 for r in rows if r.get('imu_back_s', 0) is not None and r.get('fix_back_s', 0) is not None)
                               / max(1, len(rows)), 1),
        'gps_replugs': gps_replugs, 'gps_cfg_bursts': sim_log.count('gps cfg acks='),
        'imu_backlog_lines': len(log_events(run, 'oversized IMU serial backlog')),
        'restart_over_limit': sum(1 for r in rows if r['restarts_imu_over'] or r['restarts_gps_over']),
        'restarts_total': {m: sum(r[f'restarts_{m}'] for r in rows) for m in ('imu', 'gps')},
        'vm_busy_pct_mean_max': vm_busy(run, events[0][0] - 20, events[-1][1] + 20) if events else None,
        'rows': rows,
    }
    return out


def hausdorff(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d1 = seg_dist(a[:, 0], a[:, 1], np.vstack([b, b[:1]]))
    d2 = seg_dist(b[:, 0], b[:, 1], np.vstack([a, a[:1]]))
    return float(max(d1.max(), d2.max())), float(np.percentile(d1, 95))


def area(poly):
    p = np.asarray(poly, float)[:, :2]
    return 0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], 1)) - np.dot(p[:, 1], np.roll(p[:, 0], 1)))


def zone(run):
    """Row 5: the recorded zone against the perimeter the robot truly drove."""
    c = json.load(open(os.path.join(run, 'client', 'zone.json')))
    by = obs_by(run, 'loc.jsonl')
    datum = datum_of(by)
    truth = truth_rows(run)
    zl = json.load(open(os.path.join(run, 'zone_list.json')))
    fixture_ids = {z['id'] for z in json.load(open(os.path.join(REPO, 'deploy', 'sim', 'fixtures', 'zone_record', 'zone_list.json')))}
    new = [z for z in zl if z['id'] not in fixture_ids]
    out = {'run': os.path.basename(run.rstrip('/')), 'calls': [x[1:4] for x in c.get('calls', [])], 'new_zones': len(new)}
    if not new or not datum:
        out['error'] = 'no recorded zone' if not new else 'no datum'
        return out
    rec = [p[:2] for p in new[-1]['points']]
    t0, t1 = c['t_record_start'], c['t_record_end']
    sel = [d for d in truth if t0 <= d['t'] <= t1]
    tx, ty = to_map([d['lat'] for d in sel], [d['lon'] for d in sel], datum)
    drv = np.column_stack([tx, ty])[::2]
    h, h95 = hausdorff(rec, drv)
    a_rec, a_true = area(rec), area(drv)
    gaps = np.hypot(*np.diff(np.asarray(rec + rec[:1], float), axis=0).T)
    out.update({'points': len(rec), 'hausdorff_m': round(h, 3), 'rec_to_truth_p95_m': round(h95, 3),
                'area_rec_m2': round(a_rec, 3), 'area_truth_m2': round(a_true, 3),
                'area_err_pct': round(100.0 * (a_rec - a_true) / a_true, 2),
                'max_point_gap_m': round(float(gaps.max()), 3), 'median_point_gap_m': round(float(np.median(gaps)), 3),
                'record_s': round(t1 - t0, 1)})
    faults = [f for f in load_jsonl(os.path.join(run, 'obs', 'faults.jsonl')) if 'k' in f]
    out['faults'] = [[round(f['t'] - t0, 1), f['k'], f['v']] for f in faults]
    reload = os.path.join(run, 'zone_list.after_restart.json')
    if os.path.exists(reload):
        out['reload_identical'] = json.load(open(reload)) == zl
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('what', choices=['mission', 'loc', 'reconnect', 'zone'])
    ap.add_argument('--obs', default=None, help='the observer file (default <what>.jsonl)')
    ap.add_argument('run')
    ap.add_argument('--zone', type=int, default=1)
    args = ap.parse_args()
    if args.what == 'mission':
        out = mission(args.run, args.zone)
    elif args.what == 'loc':
        out = loc(args.run, args.obs or 'loc.jsonl')
    elif args.what == 'zone':
        out = zone(args.run)
    else:
        out = reconnect(args.run, args.obs or 'loc.jsonl')
    with open(os.path.join(args.run, f'{args.what}_summary.json'), 'w') as f:
        json.dump(out, f, indent=1, default=str)
    print(json.dumps(out, indent=1, default=str)[:4000])


if __name__ == '__main__':
    main()

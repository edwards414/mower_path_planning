#!/usr/bin/env python3
"""The A/B comparison (tools/ab/README.md): each side's raw outputs become
sorted, normalised lines, one group per check; the diff of A's lines against
B's is the result. Everything that is noise by nature (counts, rates,
timings, CPU) goes to the report instead. Stdlib only.

    compare.py DIR     same as `ab.py compare DIR`
"""
import calendar
import difflib
import gzip
import hashlib
import json
import os
import re
import sys

CHECKS = ('contract', 'lists', 'services', 'topics', 'frames', 'log')

# --- topics: what is not the same from one sample to the next -----------------
# Contents of these are not compared, only their shape: they carry live values
# (ages, iTOW, the EKF's pose).
DYNAMIC = {'/robot/telemetry', '/gps/status', '/adapter/robot_pose'}
# Fields masked before contents are compared: a clock, an uptime, a random
# session id; the bundled firmware's CRC and size (the firmware embeds the
# build identity, and whether a constant takes one Thumb-2 instruction or two
# depends on its value); and the map datum's bearing, which is the EKF's yaw
# at the moment navsat locks the datum, so it depends on how far the filter
# had converged by then.
# None masks the whole value, a pattern only what it matches.
MASKS = {
    '/robot/info': {'data<json>.uptime_s': None, 'data<json>.firmware.bundled.crc32': None,
                    'data<json>.firmware.bundled.size': None},
    '/pid_autotune/status': {'data<json>.updated_at': None},
    '/adapter/map_datum': {'data<json>.bearing_rad': None},
    '/manual_command_clock': {'frame_id': re.compile(r'[0-9a-f]{32}')},   # manual-session-v1:<id>
}
CONTENT_LINES_MAX = 40   # a longer content is compared by its digest

# the parameter files launch writes for each run (launch_params_XXXXXXXX,
# robot.launch.py's mower_rsd_<node>_XXXXXXXX.yaml)
TMP = re.compile(r'/tmp/(launch_params_|mower_rsd_[a-z_]*?_)[a-z0-9_]{8}(?=\.yaml|\s|$)')

# --- log --------------------------------------------------------------------------
LOG_SUBS = [
    (re.compile(r'\x1b\[[0-9;]*m'), ''),                                  # colours
    (re.compile(r'\[([A-Za-z_][\w.]*)-\d+\]'), r'[\1]'),                 # launch's process number
    (re.compile(r'pid \[\d+\]'), 'pid [PID]'),
    (re.compile(r'\[\d{10}\.\d+\]'), '[T]'),                              # ROS log stamps
    (re.compile(r'(@|from )(\d+\.\d+\.\d+\.\d+):\d+'), r'\1\2:PORT'),    # client ports
    (re.compile(r'/home/mower/\.ros/log/\S+'), '/home/mower/.ros/log/<run>'),
    (TMP, r'/tmp/\1<tmp>'),
    (re.compile(r'(?<![\w.])-?\d+\.\d+(?![\w.])'), 'N'),                 # measured values
    (re.compile(r'\((\d+) so far'), '(N so far'),
    (re.compile(r'command_age_ms \d+'), 'command_age_ms N'),
]
# Lines that say the host stalled a thread, and what such a stall leads to: a
# message that arrived too late, or a heartbeat tick before the first
# odometry (at start, robot_status's first tick 0.5 s in races mower_base's
# first /odom_slow). How many there are depends on the machine's load, not on
# the image: listed in the report, not diffed.
TIMING = re.compile('|'.join([
    r'\[mower_base\]: (no wheel feedback for|feedback resumed|control cycle overrun)',
    r'\[mower_base\]: arm latch: (the control loop did not run|wheel feedback lost)',
    r'\[mower_base\]: arm latch: .* after the feedback loss',
    r'\[mower_base\]: STM32 receiving commands again',
    r'\[mower_base\]: Ignoring the received message .* older than the current time',
    r'\[imu\]: Discarded IMU serial backlog after a host timing gap',
    r'\[robot_status\]: /robot/online -> false',
    r'\[velocity_command_guard\]: Rejected drivetrain command: velocity timestamp is stale',
]))

# side.sh's own processes in procs.txt
HARNESS = re.compile(r'side\.sh|incontainer\.py|fake_(base|sensors)\.py|ps -eo')

FRAME_NAMES = {1: 'WHEEL_SPEED', 2: 'LAWER_MOTOR', 3: 'WS2812', 4: 'PID_CONFIG', 5: 'POWER',
               6: 'INFO_REQUEST', 7: 'SERVO'}


def load(path, default=None):
    try:
        with open(path, errors='replace') as f:
            return json.load(f) if path.endswith('.json') else f.read()
    except (OSError, ValueError):
        return default


def split_log(text, t_stop):
    """`docker logs --timestamps` output -> (lines before t_stop, lines after),
    each without the timestamp, stdout and stderr merged in time order."""
    rows = []
    for i, line in enumerate(text.splitlines(keepends=True)):
        stamp, _, rest = line.partition(' ')
        m = re.match(r'(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d)(?:\.(\d+))?Z$', stamp)
        if not m:
            rows.append((rows[-1][0] if rows else 0.0, i, line))
            continue
        y, mo, d, h, mi, s, frac = m.groups()
        t = calendar.timegm((int(y), int(mo), int(d), int(h), int(mi), int(s))) + float('0.' + (frac or '0'))
        rows.append((t, i, rest if rest.endswith('\n') else rest + '\n'))
    rows.sort()
    return [r[2] for r in rows if r[0] < t_stop], [r[2] for r in rows if r[0] >= t_stop]


class Identity:
    """Each side's own build identity (the image's MOWER_* env), replaced by a
    placeholder wherever the app or the log shows it: a side that reports
    its own version is fine, one that reports something else shows. A value
    equal to one is replaced whatever its length. Inside a longer string, a
    value of 7 characters or more is replaced where it stands alone (a commit
    name, a build time, the image); a short one is not, since a release name
    like 0.6.0 is also the fake STM32's firmware version."""

    KEYS = ('MOWER_GIT_SHA', 'MOWER_VERSION', 'MOWER_BUILD_UNIX', 'MOWER_IMAGE')

    def __init__(self, env):
        self.exact = {v: f'<{k}>' for k, v in env.items() if k in self.KEYS and v}
        self.subs = [(re.compile(r'(?<![0-9A-Za-z])' + re.escape(v) + r'(?![0-9A-Za-z])'), n)
                     for v, n in sorted(self.exact.items(), key=lambda kv: -len(kv[0])) if len(v) >= 7]

    def text(self, s):
        for rx, name in self.subs:
            s = rx.sub(name, s)
        return s

    def value(self, v):
        if isinstance(v, bool) or not isinstance(v, (str, int)):
            return v
        if str(v) in self.exact:
            return self.exact[str(v)]
        return self.text(v) if isinstance(v, str) else v


def flatten(value, path='', out=None):
    """{'a': {'b': [1, 2]}} -> {'a.b[0]': 1, 'a.b[1]': 2}; a JSON document
    inside a string is under '<json>' (probe.py's norm)."""
    out = {} if out is None else out
    if isinstance(value, dict):
        if not value:
            out[path] = {}
        for k, v in value.items():
            flatten(v, path + k if k == '<json>' else (f'{path}.{k}' if path else k), out)
    elif isinstance(value, list):
        if not value:
            out[path] = []
        for i, v in enumerate(value):
            flatten(v, f'{path}[{i}]', out)
    else:
        out[path] = value
    return out


def masked(value, mask):
    return '<masked>' if mask is None else mask.sub('<id>', str(value))


def content_lines(prefix, value, ident, masks=None):
    flat, masks = flatten(value), masks or {}
    rows = [f'{prefix} {k} = ' + json.dumps(masked(flat[k], masks[k]) if k in masks else ident.value(flat[k]),
                                             ensure_ascii=False, sort_keys=True)
            for k in sorted(flat)]
    if len(rows) > CONTENT_LINES_MAX:
        digest = hashlib.sha1('\n'.join(rows).encode()).hexdigest()[:16]
        return [f'{prefix} sha1 {digest} ({len(rows)} values)']
    return rows


# --- per check: one side's lines --------------------------------------------------
CONTRACT_RUNS = (('contract', '1 client'), ('contract_2a', '2 clients, a'), ('contract_2b', '2 clients, b'))


def contract(d, ident):
    """probe_app.py's verdict against app_contract_baseline.json, with one
    client and with two at once, and the topics each delivered."""
    lines = []
    for name, label in CONTRACT_RUNS:
        text = load(os.path.join(d, name + '.txt'), '')
        if not text:
            lines.append(f'contract [{label}] did not run')
            continue
        for row in text.splitlines():
            if row.startswith(('APP_CONTRACT', 'content=', 'services=', 'topics_gone_from_rosapi=', 'Traceback')):
                lines.append(f'contract [{label}] {row}')
            elif row.startswith('connect_s='):
                lines.append(f'contract [{label}] {row.split(" ", 1)[-1]}')
        saved = load(os.path.join(d, name + '.json'), {})
        lines += [f'contract [{label}] delivered {t}' for t in saved.get('delivered', [])]
    return lines


def lists(d, ident):
    """What the app can list and is allowed to use: /rosapi/topics with types,
    and the bridge's policy (the ws_bridge.yaml mower_rsd was given)."""
    lines = []
    doc = load(os.path.join(d, 'services.json'), {})
    topics = (doc.get('/rosapi/topics') or {}).get('topics')
    if topics is None:
        lines.append('rosapi /rosapi/topics: no answer')
    lines += [f'rosapi topic {t} {ty}' for t, ty in sorted((topics or {}).items())]
    policy = load(os.path.join(d, 'rsd_files', 'ws_bridge.yaml'))
    if policy is None:
        lines.append('policy: no ws_bridge.yaml in mower_rsd\'s argv')
    for row in (policy or '').splitlines():
        row = row.split('#')[0].rstrip()
        if row.strip():
            lines.append(f'policy {row}')
    return lines


def services(d, ident):
    """The read-only services' answers on the loopback port."""
    doc = load(os.path.join(d, 'services.json'), {})
    if not doc:
        return ['services: no answers recorded']
    lines = []
    for name in sorted(doc):
        if name == '/rosapi/topics':
            continue
        e = doc[name]
        lines.append(f'service {name} result {json.dumps(e.get("result"))}')
        if 'values' in e:
            lines += content_lines(f'service {name}', e['values'], ident)
    return lines


def topics(d, ident):
    """Every listed topic through the gate: delivered or not, the JSON shape,
    and (but for DYNAMIC) the distinct contents."""
    doc = load(os.path.join(d, 'topics.json'), {})
    if not doc:
        return ['topics: nothing recorded']
    lines = [f'topics: bridge status message {m}' for m in sorted(set(doc.get('errors') or []))]
    for name, e in sorted(doc['topics'].items()):
        lines.append(f'topic {name} ' + ('delivered' if e['count'] else f'nothing in {doc["seconds"]} s'))
        lines += [f'topic {name} shape {s}' for s in e['shape']]
        if name in DYNAMIC or not e['contents']:
            continue
        variants = sorted({'\n'.join(content_lines(f'topic {name} content', c, ident, MASKS.get(name)))
                           for c in e['contents']})
        if len(variants) > 1:
            lines.append(f'topic {name} {len(variants)} different contents')
        for i, v in enumerate(variants):
            lines += [row.replace(' content ', f' content#{i + 1} ', 1) if len(variants) > 1 else row
                      for row in v.split('\n')]
    return lines


def frame_rows(d):
    rows = []
    try:
        with gzip.open(os.path.join(d, 'fake_base.jsonl.gz'), 'rt') as f:
            for line in f:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        return None
    return [r for r in rows if r.get('dir') == 'rx']


def frames(d, ident):
    """What the host sent the fake STM32 (fake_base.py's rx log), per frame
    type: the payloads in order, consecutive repeats collapsed."""
    rx = frame_rows(d)
    if rx is None:
        return ['frames: no fake_base log']
    seqs = {}
    for r in rx:
        seq = seqs.setdefault(FRAME_NAMES.get(r['type'], hex(r['type'])), [])
        if not seq or seq[-1] != r['payload']:
            seq.append(r['payload'])
    lines = []
    for name, seq in sorted(seqs.items()):
        shown = ' -> '.join(seq[:12]) + (' -> ...' if len(seq) > 12 else '')
        digest = hashlib.sha1(' '.join(seq).encode()).hexdigest()[:12]
        lines.append(f'frames {name}: {len(seq)} payload run(s), sha1 {digest}: {shown}')
    return lines


def log_lines(path, ident):
    out = []
    for line in (load(path) or '').splitlines():
        if not line.strip():
            continue
        line = ident.text(line)
        for rx, sub in LOG_SUBS:
            line = rx.sub(sub, line)
        out.append(line.rstrip())
    return out


def log(d, ident):
    """The distinct normalised lines the stack printed while it ran (the
    container's log up to `docker stop`), host-stall lines left out."""
    lines = log_lines(os.path.join(d, 'stack.log'), ident)
    if not lines:
        return ['log: the stack printed nothing']
    return sorted({f'log {line}' for line in lines if not TIMING.search(line)})


def summarize(d, ident):
    return {'contract': contract(d, ident), 'lists': lists(d, ident), 'services': services(d, ident),
            'topics': topics(d, ident), 'frames': frames(d, ident), 'log': log(d, ident)}


# --- informational ----------------------------------------------------------------
def info(d, side, ident):
    out = {'times': side.get('times', {}), 'stop': side.get('stop', {})}
    out['rates_hz'] = load(os.path.join(d, 'contract.json'), {}).get('rates_hz', {})
    doc = load(os.path.join(d, 'topics.json'), {})
    out['topic_counts'] = {t: e['count'] for t, e in (doc.get('topics') or {}).items()}
    rx = frame_rows(d) or []
    out['frames'] = {}
    if rx:
        span = max(rx[-1]['t'] - rx[0]['t'], 1e-9)
        for r in rx:
            k = FRAME_NAMES.get(r['type'], hex(r['type']))
            out['frames'][k] = out['frames'].get(k, 0) + 1
        out['frames'] = {k: f'{n} ({n / span:.2f} Hz)' for k, n in sorted(out['frames'].items())}
    lines = log_lines(os.path.join(d, 'stack.log'), ident)
    out['timing'] = {}
    for x in lines:
        if TIMING.search(x):
            out['timing'][x] = out['timing'].get(x, 0) + 1
    out['log'] = {'lines': len(lines), 'timing_lines': sum(out['timing'].values()),
                  'stop_lines': len(log_lines(os.path.join(d, 'stack_stop.log'), ident))}
    out['cpu'] = load(os.path.join(d, 'cpu_idle.json'), {})
    out['argv'] = [ident.text(TMP.sub(r'/tmp/\1<tmp>', a))
                   for a in (load(os.path.join(d, 'rsd_argv.txt')) or '').splitlines()]
    out['procs'] = sorted(ident.text(TMP.sub(r'/tmp/\1<tmp>', p))
                          for p in (load(os.path.join(d, 'procs.txt')) or '').splitlines()[1:]
                          if not HARNESS.search(p))
    return out


# --- the two sides ----------------------------------------------------------------
def render(out, gate=CHECKS):
    """Write DIR/{A,B}.summary.txt, diff.txt and report.md; True when the
    checks in `gate` show no difference (the diff holds every check)."""
    meta = load(os.path.join(out, 'run.json'), {})
    sides = meta.get('sides', {})
    summary, infos = {}, {}
    for label in ('A', 'B'):
        side = sides.get(label, {})
        ident = Identity((side.get('image') or {}).get('mower_env') or {})
        d = os.path.join(out, label)
        summary[label] = summarize(d, ident)
        infos[label] = info(d, side, ident)
        with open(os.path.join(out, f'{label}.summary.txt'), 'w') as f:
            for c in CHECKS:
                f.writelines(f'{line}\n' for line in summary[label][c])
    diff, per_check = [], {}
    for c in CHECKS:
        d = list(difflib.unified_diff(summary['A'][c], summary['B'][c], f'A {c}', f'B {c}', n=0, lineterm=''))
        per_check[c] = sum(1 for x in d if x[:1] in '+-' and not x.startswith(('+++', '---')))
        diff += d
    with open(os.path.join(out, 'diff.txt'), 'w') as f:
        f.writelines(f'{line}\n' for line in diff)
    report(out, meta, summary, per_check, diff, infos, gate)
    print(f'[ab] diff {"empty" if not diff else "NOT EMPTY"} ('
          + ', '.join(f'{c} {n}' for c, n in per_check.items()) + f'): {os.path.join(out, "report.md")}')
    return not any(per_check[c] for c in gate)


def report(out, meta, summary, per_check, diff, infos, gate):
    sides = meta.get('sides', {})
    w = ['# Image A/B\n', '| | A | B |', '|---|---|---|']
    for key, get in (('image', lambda s: s['image']['ref']),
                     ('id', lambda s: s['image']['id'][:19]),
                     ('MOWER_GIT_SHA', lambda s: s['image'].get('git_sha', '')[:12]),
                     ('ROS_DOMAIN_ID', lambda s: s.get('ros_domain_id'))):
        w.append(f'| {key} | ' + ' | '.join(str(get(sides[x])) if x in sides else '' for x in 'AB') + ' |')
    w.append(f'\nRun {meta.get("run_id")}, started {meta.get("started")}, took {meta.get("seconds")} s.\n')
    w += ['## Checks\n', '| check | lines A | lines B | differing | gates the result |', '|---|---|---|---|---|']
    w += [f'| {c} | {len(summary["A"][c])} | {len(summary["B"][c])} | {per_check[c] or "none"} | '
          f'{"yes" if c in gate else "no"} |' for c in CHECKS]
    w.append('\n## Diff\n')
    if diff:
        w.append('```diff')
        w += diff[:400]
        if len(diff) > 400:
            w.append(f'... {len(diff) - 400} more lines in diff.txt')
        w.append('```')
    else:
        w.append('Empty: everything compared is the same.')
    w += ['\n## Not compared (informational)\n', '| | A | B |', '|---|---|---|']
    a, b = infos['A'], infos['B']
    rows = [('bridge up after (s)', a['times'].get('bridge_up_s'), b['times'].get('bridge_up_s')),
            ('app contract OK after (s)', a['times'].get('contract_ok_s'), b['times'].get('contract_ok_s')),
            ('docker stop: seconds, exit code',
             f'{a["stop"].get("seconds")}, {a["stop"].get("exit_code")}',
             f'{b["stop"].get("seconds")}, {b["stop"].get("exit_code")}'),
            ('log lines / host-stall lines / lines while stopping',
             '{lines} / {timing_lines} / {stop_lines}'.format(**a['log']),
             '{lines} / {timing_lines} / {stop_lines}'.format(**b['log'])),
            ('mower_rsd argv', 'same as B' if a['argv'] == b['argv'] else 'differs (rsd_argv.txt)', ''),
            ('processes', 'same as B' if a['procs'] == b['procs'] else 'differ (procs.txt)', '')]
    rows += [(f'rate {t} (Hz)', a['rates_hz'].get(t), b['rates_hz'].get(t))
             for t in sorted(set(a['rates_hz']) | set(b['rates_hz']))]
    rows += [(f'frames {k}', a['frames'].get(k), b['frames'].get(k))
             for k in sorted(set(a['frames']) | set(b['frames']))]
    rows += [(f'messages {t}', a['topic_counts'].get(t), b['topic_counts'].get(t))
             for t in sorted(set(a['topic_counts']) | set(b['topic_counts']))]
    ca, cb = a['cpu'].get('by_name', {}), b['cpu'].get('by_name', {})
    top = lambda p: -max(ca.get(p, {}).get('cpu_pct', 0), cb.get(p, {}).get('cpu_pct', 0))  # noqa: E731
    rows += [(f'CPU % {p}', ca.get(p, {}).get('cpu_pct'), cb.get(p, {}).get('cpu_pct'))
             for p in sorted(set(ca) | set(cb), key=top)]
    if ca or cb:
        rows.append(('CPU window (s), load average',
                     f'{a["cpu"].get("window_s")}, {" ".join(a["cpu"].get("loadavg", []))}',
                     f'{b["cpu"].get("window_s")}, {" ".join(b["cpu"].get("loadavg", []))}'))
    w += [f'| {k} | {"" if x is None else x} | {"" if y is None else y} |' for k, x, y in rows]
    ta, tb = a['timing'], b['timing']
    if ta or tb:
        w += ['\nHost-stall lines while running (not diffed):\n', '| A | B | line |', '|---|---|---|']
        w += [f'| {ta.get(x, 0)} | {tb.get(x, 0)} | `{x[:200]}` |' for x in sorted(set(ta) | set(tb))]
    with open(os.path.join(out, 'report.md'), 'w') as f:
        f.write('\n'.join(w) + '\n')


if __name__ == '__main__':
    sys.exit(0 if render(os.path.abspath(sys.argv[1])) else 1)

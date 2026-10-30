#!/usr/bin/env python3
"""Vectors for mower_pyfmt, written by the real Python and PyYAML.

Run in the robot image (Python 3.12, PyYAML 6.0.1), from the repository root,
with the E5-0 recordings (deploy/test/golden/recorder) mounted at /golden:

    docker run --rm --network none -v "$PWD/src/mower_rs/crates/mower_pyfmt":/x \
      -v "$PWD/deploy/test/golden/recorder":/golden:ro \
      --entrypoint python3 mower_path_planning:ros-free-test -I \
      /x/tests/oracle/pyfmt_gen.py --out /x/tests/vectors/pyfmt.json

Every case is a value in a typed JSON encoding (floats as repr strings, so
NaN, infinities and -0.0 survive) and the exact text of each output combo.
Three groups:

- edge: hand-written values, every YAML 1.1 look-alike string, the
  characters that change PyYAML's quoting, folding, keys, floats, ints;
- random: seeded documents from a pool of tricky characters and words;
- real: what the Python recorder wrote in the E5-0 fixture runs. Each file or
  message is parsed, written again with its writer's exact call, checked to
  be byte-identical to what the recorder wrote, and kept with that combo.

For every value the generator also checks that yaml.dump (Dumper) writes the
same text as yaml.safe_dump (SafeDumper) with the same keywords, which is
why the crate's `yaml::dump` is `safe_dump`.
"""
import argparse
import glob
import json
import math
import os
import random
import struct
import sys

import yaml

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--random', type=int, default=150, help='random documents (default 150)')
ap.add_argument('--seed', type=int, default=5)
ap.add_argument('--all-combos', action='store_true', help='every combo for every random document')
ap.add_argument('--no-real', action='store_true', help='leave out the recorder documents')
a = ap.parse_args()

COMBOS = {
    # json.dumps keywords
    'j': {},
    'ju': {'ensure_ascii': False},
    'js': {'sort_keys': True},
    'ji2u': {'ensure_ascii': False, 'indent': 2},
    'ji1s': {'indent': 1, 'sort_keys': True},
    'ji0': {'indent': 0},
    # yaml.safe_dump keywords
    'y': {},
    'yu': {'allow_unicode': True, 'sort_keys': False},
    'yus': {'allow_unicode': True},
    'ya': {'sort_keys': False},
}


def render(combo, v):
    kw = COMBOS[combo]
    if combo.startswith('j'):
        return json.dumps(v, **kw)
    text = yaml.safe_dump(v, **kw)
    other = yaml.dump(v, default_flow_style=False, **kw)
    if other != text:
        raise AssertionError(f'yaml.dump != yaml.safe_dump for {v!r} {kw}')
    return text


def enc(v):
    if v is None:
        return ['N']
    if v is True or v is False:
        return ['B', v]
    if isinstance(v, int):
        if not -2**63 <= v < 2**63:
            raise ValueError(f'int out of i64: {v}')
        return ['I', str(v)]
    if isinstance(v, float):
        return ['F', repr(v)]
    if isinstance(v, str):
        return ['S', v]
    if isinstance(v, list):
        return ['L', [enc(x) for x in v]]
    if isinstance(v, dict):
        for k in v:
            if not isinstance(k, str):
                raise ValueError(f'non-str key {k!r}')
        return ['D', [[k, enc(x)] for k, x in v.items()]]
    raise ValueError(f'unsupported {type(v).__name__}: {v!r}')


CASES = []


def add(group, cid, v, combos=None, expect=None):
    """expect: {combo: text the recorder wrote}; checked, then stored."""
    out = {}
    for c in (combos or COMBOS):
        out[c] = render(c, v)
    for c, text in (expect or {}).items():
        got = render(c, v)
        if got != text:
            raise AssertionError(f'{cid}: {c} does not reproduce the file\n{got!r}\n{text!r}')
        out[c] = got
    CASES.append({'id': f'{group}/{cid}', 'v': enc(v), 'out': out})


# ---- edge cases ----------------------------------------------------------------
LOOKALIKES = [
    'yes', 'Yes', 'YES', 'no', 'No', 'NO', 'true', 'True', 'TRUE', 'false', 'False', 'FALSE',
    'on', 'On', 'ON', 'off', 'Off', 'OFF', 'y', 'n', 'Y', 'N', 'oN', 'tRue',
    '~', 'null', 'Null', 'NULL', 'nULL', '', ' ',
    '0', '-0', '+0', '00', '07', '08', '0o17', '0b101', '0b', '0x1F', '0xG', '0x_', '1_000', '_1',
    '1', '-1', '+1', '123', '1e3', '1E3', '1e+3', '1.0e+3', '1.e-3', '1.', '.5', '.5e+1', '-.5', '+.5',
    '1.5', '-1.5', '1_0.5_', '12:30:00', '190:20:30.15', '1:2', '1:60', '1:59', '0:30', '-1:30',
    '1:2:3.', '+12:00:00.5',
    '.inf', '.Inf', '.INF', '-.inf', '+.inf', '.nan', '.NaN', '.NAN', '-.nan', 'inf', 'nan', 'NaN',
    'Infinity', '.', '..', '...', '---', '--- x', '... x', '-', '--', '- x', '-x', '?', '? x', '?x',
    ':', ': x', ':x', 'x:', 'x: y', 'x:y', 'a #b', 'a#b', '#a', 'a,b', 'a[b]', 'a{b}', '[a]', '{a}',
    ',a', '&a', '*a', '!a', '|a', '>a', "'a", '"a', '%a', '@a', '`a', 'a\'b', 'a"b', "it's",
    '2026-10-27', '2026-1-2', '2026-10-27T10:15:00.123456', '2026-10-27 10:15:00',
    '2026-10-27t10:15:00Z', '2026-10-27 10:15:00 +08:00', '2026-10-27  1:02:03.5-7',
    '2026-10-27T10:15', '20261027T021500', 'MW-0CFP37_20261027T021500',
    '<<', '<', '=', '==', '!', '&', '*', ' a', 'a ', '  ', 'a  b', 'a\tb', '\t', 'a\n', '\na', 'a\nb',
    'a\n\nb', 'a \nb', 'a\n b', '\n', '\r', 'a\rb', '\x85', 'a\x85b', '\u2028', 'a\u2029b', '\ufeff',
    'a\ufeffb', '\x00', '\x07', '\x1b', '\x7f', '\x80', '\x9f', '\xa0', 'é', '中文', '錄製中',
    '\U0001F600', 'a\U0001F600b', '\ud7ff', '\ue000', '\ufffd', '\ufffe', '\uffff', '\U0010FFFF',
    '\U0010FFFE', 'C:\\path\\x', 'back\\slash', '/mower_recorder/status', 'pkg/msg/Type',
    'RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18',
]
for i, s in enumerate(LOOKALIKES):
    add('edge', f'str{i}', s)
    add('edge', f'key{i}', {s: s, 'k': [s]}, combos=['ju', 'y', 'yu'])

WORDS = ('alpha beta gamma delta epsilon zeta eta theta iota kappa lambda mu nu xi omicron pi rho '
         'sigma tau upsilon phi chi psi omega').split()
LONG = ['ju', 'y', 'yu', 'ya']
for n in (79, 80, 81, 82, 120, 250):
    words, k = [], 0
    while len(' '.join(words)) < n:
        words.append(WORDS[k % len(WORDS)])
        k += 1
    s = ' '.join(words)[:n].rstrip()
    add('edge', f'long{n}', s, combos=LONG)
    add('edge', f'longval{n}', {'pipeline': s, 'nested': {'deeper': [s]}}, combos=LONG)
    add('edge', f'longsq{n}', {'q': "it's " + s}, combos=LONG)
    add('edge', f'longdq{n}', {'q': s + '\x07 ' + s}, combos=LONG)
    add('edge', f'longuni{n}', {'q': '錄製 ' + s.replace('a', 'é')}, combos=LONG)
    add('edge', f'longnl{n}', {'q': s + '\n' + s}, combos=LONG)
    add('edge', f'longsp{n}', {'q': s.replace(' ', '  ')}, combos=LONG)
    add('edge', f'nospace{n}', 'x' * n, combos=LONG)
add('edge', 'longkey127', {'k' * 127: 1})
add('edge', 'longkey128', {'k' * 128: 1})
add('edge', 'longkey200', {'k' * 200: {'a': 1}})
add('edge', 'mlkey', {'a\nb': 1, '': 2, ' ': 3})
add('edge', 'pipeline', {'pipeline': 'v4l2src device=/dev/video0 extra-controls="c,focus_automatic_continuous=0,'
                         'focus_auto=0,exposure_dynamic_framerate=0,exposure_auto_priority=0" ! '
                         'image/jpeg,width=1280,height=720,framerate=25/1 ! videorate drop-only=true max-rate=10'})
FLOATS = [0.0, -0.0, 1.0, -1.0, 0.5, 0.1, 0.2, 0.3, 1e-05, 1.5e-05, 0.0001, 0.001, 1e16, 1e17, 1.5e16,
          1e15, 123456789.0, 9007199254740993.0, 1e22, 1e23, 1e-07, 5e-324, 2.2250738585072014e-308,
          1.7976931348623157e308, float('nan'), float('inf'), float('-inf'), 12.3, 20.0, 1.0e-5,
          3.141592653589793, -273.15, 0.585, 17.5, 24.62312345, 121.53456789012]
for i, f in enumerate(FLOATS):
    add('edge', f'float{i}', f)
add('edge', 'floats', FLOATS)
add('edge', 'floatmap', {repr(f): f for f in FLOATS})
add('edge', 'ints', [0, 1, -1, 2**31, -2**31, 2**63 - 1, -2**63, 10**18])
add('edge', 'bools', [True, False, None, {'t': True, 'f': False, 'n': None}])
add('edge', 'empty', [[], {}, [[]], [{}], {'a': []}, {'b': {}}, {'c': [[], {}]}])
add('edge', 'emptyroot_list', [])
add('edge', 'emptyroot_dict', {})
add('edge', 'nested', {'a': [{'b': [1, 2, {'c': [3, [4, [5]]]}]}, [6, [7]], {'d': {'e': {'f': 'g'}}}],
                       'z': [[[]]], 'm': {'x': [{'y': None}]}})
add('edge', 'sortorder', {'b': 1, 'a': 2, 'B': 3, 'é': 4, '中': 5, '_': 6, '1': 7, '': 8, ' ': 9})
add('edge', 'rootlist', ['a', 1, 1.5, None, True, [], {}, ['x'], {'k': 'v'}])

# random doubles: the repr digits
rng = random.Random(20261030)
bits = [struct.unpack('<d', struct.pack('<Q', rng.getrandbits(64)))[0] for _ in range(300)]
bits = [b for b in bits if not math.isnan(b)]
small = [rng.uniform(-1000, 1000) for _ in range(150)] + [rng.uniform(-1, 1) * 10 ** rng.randint(-30, 30)
                                                       for _ in range(150)]
add('edge', 'randbits', bits, combos=['j', 'y'])
# exact decimal ties between two shortest strings (dtoa rounds half to even):
# multiples of 1/4 between 2**49 and 2**50, where the spacing is 1/8
ties = [rng.randrange(2**51, 2**52) / 4 for _ in range(200)]
add('edge', 'ties', ties, combos=['j', 'y'])
add('edge', 'randfloat', small, combos=['j', 'y'])

# ---- random documents ----------------------------------------------------------------
POOL = list('abcxyzABC019 \t\n\r#:-?[]{},&*!|>\'"%@`\\.~_=<') + [
    '\x00', '\x07', '\x1b', '\x7f', '\x85', '\xa0', '\u2028', '\u2029', '\ufeff', 'é', '中', '文', '錄',
    '\U0001F600', '\ud7ff', '\ue000', '\ufffd', '\U0010FFFF']
TOKENS = LOOKALIKES[:80] + WORDS


def rstr(r):
    k = r.random()
    if k < 0.35:
        return r.choice(TOKENS)
    if k < 0.45:
        return ' '.join(r.choice(WORDS) for _ in range(r.randint(10, 30)))
    return ''.join(r.choice(POOL) for _ in range(r.randint(0, 12)))


def rval(r, depth):
    k = r.random()
    if depth > 3 or k < 0.55:
        t = r.random()
        if t < 0.5:
            return rstr(r)
        if t < 0.65:
            return r.randint(-10**12, 10**12)
        if t < 0.8:
            return r.choice([r.uniform(-1e6, 1e6), r.choice(FLOATS), r.uniform(-1, 1) * 10 ** r.randint(-20, 20)])
        if t < 0.9:
            return r.choice([True, False])
        return None
    if k < 0.75:
        return [rval(r, depth + 1) for _ in range(r.randint(0, 4))]
    return {rstr(r): rval(r, depth + 1) for _ in range(r.randint(0, 4))}


r = random.Random(a.seed)
for i in range(a.random):
    add('random', f'doc{i}', rval(r, 0), combos=None if a.all_combos else ['j', 'ji2u', 'y', 'yu', 'ya'])

# ---- real documents: what the recorder wrote ------------------------------------------
real_yaml = real_json = 0
seen = set()
for path in [] if a.no_real else sorted(glob.glob('/golden/F*/MW-*/**/*.yaml', recursive=True)):
    rel = path.split('/golden/', 1)[1]
    name = os.path.basename(path)
    if name == 'metadata.yaml':
        continue  # rosbag2's, written by yaml-cpp
    # the default-profile runs all write the same shapes: F1's and F3's files
    # are kept (F3 adds the data_collection fields, calib and the EKF dumps),
    # one EKF dump of the two
    if rel.split('/')[0] not in ('F1', 'F3') or name == 'ekf_filter_node_odom.yaml':
        continue
    text = open(path, encoding='utf-8').read()
    if name == 'run_metadata.yaml' or name == 'extrinsics.yaml':
        # yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False);
        # calib/extrinsics.yaml has a comment line in front (record_profile.py)
        body = text.split('\n', 1)[1] if text.startswith('#') else text
        combo = 'yu'
    elif '/params/' in path:
        # ros2param verb/dump.py: print(yaml.dump(yaml_output, default_flow_style=False))
        assert text.endswith('\n\n'), rel
        body = text[:-1]
        combo = 'y'
    elif name == 'camera_front.yaml':
        # record_run.sh's calibration: yaml.safe_dump(doc, f, sort_keys=False)
        body = text
        combo = 'ya'
    else:
        raise SystemExit(f'unknown recorder yaml {rel}')
    if body in seen:
        continue
    seen.add(body)
    add('real', rel, yaml.safe_load(body), combos=[], expect={combo: body})
    real_yaml += 1

# listen.jsonl: the recorder's String messages verbatim
SITES = {
    '/mower_recorder/status': 'ju',          # recorder_manager_node.py:303
    '/mower_recorder/bags': 'ju',            # bag_store_node.py:208
    '/mower_recorder/command_result': 'ju',  # bag_store_node.py:236
    '/mower_base/rear_light': 'j',           # recorder_manager_node.py:307 (defaults)
    '/graph_snapshot': 'ju',                 # graph_snapshot_node.py:87
    '/graph_events': 'ju',                   # graph_snapshot_node.py:104
}
# a /graph_snapshot message is ~300 kB of node and topic lists: none are kept
# (the random documents cover the shapes), every other kind is
LIMIT = {'/graph_snapshot': 0, '/mower_recorder/bags': 6, '/mower_recorder/status': 10}
per_topic = {}
for path in [] if a.no_real else sorted(glob.glob('/golden/F*/listen.jsonl.zst')):
    run = path.split('/')[-2]
    import zstandard
    raw = zstandard.ZstdDecompressor().decompressobj().decompress(open(path, 'rb').read())
    for n, line in enumerate(raw.decode('utf-8').splitlines()):
        m = json.loads(line)
        t, data = m['topic'], m['data']
        if data in seen:
            continue
        seen.add(data)
        if per_topic.get(t, 0) >= LIMIT.get(t, 1000):
            continue
        per_topic[t] = per_topic.get(t, 0) + 1
        add('real', f'{run}{t}#{n}', json.loads(data), combos=[], expect={SITES[t]: data})
        real_json += 1

doc = {
    'generator': 'tests/oracle/pyfmt_gen.py',
    'python': sys.version.split()[0],
    'pyyaml': yaml.__version__,
    'combos': COMBOS,
    'note': 'yaml.dump(v, default_flow_style=False, **kw) == yaml.safe_dump(v, **kw) checked for every '
            'yaml output; params/*.yaml = combo y + "\\n" (ros2 param dump prints the dump)',
    'cases': CASES,
}
with open(a.out, 'w', encoding='ascii') as f:
    json.dump(doc, f, ensure_ascii=True, separators=(',', ':'))
    f.write('\n')
outputs = sum(len(c['out']) for c in CASES)
print(f'{len(CASES)} cases, {outputs} outputs ({real_yaml} recorder yaml files, '
      f'{real_json} recorder messages: {per_topic}); {os.path.getsize(a.out)} bytes')

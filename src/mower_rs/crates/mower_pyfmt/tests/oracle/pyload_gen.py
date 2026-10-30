#!/usr/bin/env python3
"""Load vectors for mower_pyfmt: what the real yaml.safe_load (PyYAML 6.0.1)
and json.loads (CPython 3.12) make of a set of texts.

Run in the robot image from the repository root, with the crate at /x, the
repository at /repo and the E5-0 recordings at /golden:

    docker run --rm --network none -v "$PWD/src/mower_rs/crates/mower_pyfmt":/x \\
      -v "$PWD":/repo:ro -v "$PWD/deploy/test/golden/recorder":/golden:ro \\
      --entrypoint python3 mower_path_planning:ros-free-test -I \\
      /x/tests/oracle/pyload_gen.py --out /x/tests/vectors/pyload.json

Each YAML case holds the text and one of: the value (typed: floats as their
IEEE bits, dates and datetimes by field), `err` (what Python raised), or
`gap` (Python returned a value mower_pyfmt has no type for: a non-string key,
an int outside i64, bytes, a set, a tuple, a recursive structure; the Rust
loader must refuse those). Where the value has no shared collections, `dump`
is `yaml.safe_dump(value, allow_unicode=True, sort_keys=False)` of it, which
tests the dumper on loaded values (dates, datetimes) too.

`refs` point at the outputs of each case of tests/vectors/pyfmt.json instead
of copying them (the dumper's texts loaded back): the combos in `same` load
as the very value the case was dumped from, those in `sorted` as that value
with every dict's keys sorted (a `sort_keys` dump), `other` holds the rest.

JSON cases hold the text and the value, or `err` = str() of the
JSONDecodeError.
"""
import argparse
import datetime
import glob
import json
import os
import struct
import sys

import yaml
import zstandard

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--pyfmt', default='/x/tests/vectors/pyfmt.json')
a = ap.parse_args()


class Gap(Exception):
    pass


# set by enc() when a str held a lone surrogate, which it wrote as U+FFFD
# (a Rust string cannot hold one; both loaders give U+FFFD instead)
LOSSY = [False]


def enc(v, stack=()):
    if any(v is s for s in stack):
        raise Gap('a recursive structure')
    if v is None:
        return ['N']
    if v is True or v is False:
        return ['B', v]
    if isinstance(v, int):
        if not -2**63 <= v < 2**63:
            raise Gap('an int outside i64')
        return ['I', str(v)]
    if isinstance(v, float):
        return ['F', struct.pack('>d', v).hex()]
    if isinstance(v, str):
        if any(0xD800 <= ord(c) <= 0xDFFF for c in v):
            LOSSY[0] = True
            v = ''.join('\ufffd' if 0xD800 <= ord(c) <= 0xDFFF else c for c in v)
        return ['S', v]
    if isinstance(v, datetime.datetime):
        off = None
        if v.tzinfo is not None:
            secs = v.utcoffset().total_seconds()
            assert secs % 60 == 0, v
            off = int(secs // 60)
        return ['Dt', [v.year, v.month, v.day, v.hour, v.minute, v.second, v.microsecond, off]]
    if isinstance(v, datetime.date):
        return ['Da', [v.year, v.month, v.day]]
    if isinstance(v, list):
        return ['L', [enc(x, stack + (v,)) for x in v]]
    if isinstance(v, dict):
        for k in v:
            if not isinstance(k, str):
                raise Gap(f'a {type(k).__name__} key')
        return ['D', [[k, enc(x, stack + (v,))] for k, x in v.items()]]
    raise Gap(f'a {type(v).__name__}')


def shared(v):
    """True when a list or dict appears twice (safe_dump would write &id001)."""
    seen = set()

    def walk(x):
        if isinstance(x, (list, dict)):
            if id(x) in seen:
                return True
            seen.add(id(x))
            items = x.values() if isinstance(x, dict) else x
            return any(walk(y) for y in items)
        return False
    return walk(v)


def strict_eq(x, y):
    if type(x) is not type(y):
        return False
    if isinstance(x, float):
        return struct.pack('>d', x) == struct.pack('>d', y)
    if isinstance(x, list):
        return len(x) == len(y) and all(strict_eq(p, q) for p, q in zip(x, y))
    if isinstance(x, dict):
        return list(x) == list(y) and all(strict_eq(x[k], y[k]) for k in x)
    return x == y


def dec(t):
    """The pyfmt.json typed encoding (floats as repr) back to a value."""
    tag = t[0]
    if tag == 'N':
        return None
    if tag == 'B':
        return t[1]
    if tag == 'I':
        return int(t[1])
    if tag == 'F':
        return float(t[1])
    if tag == 'S':
        return t[1]
    if tag == 'L':
        return [dec(x) for x in t[1]]
    if tag == 'D':
        return {k: dec(x) for k, x in t[1]}
    raise ValueError(tag)


YAML, JSON, REFS = [], [], []
counts = {}


def ycase(group, cid, text, dump=True):
    rec = {'id': f'{group}/{cid}', 'text': text}
    try:
        v = yaml.safe_load(text)
    except Exception as e:  # noqa: BLE001
        rec['err'] = f'{type(e).__name__}: {e}'.splitlines()[0][:200]
    else:
        LOSSY[0] = False
        try:
            rec['v'] = enc(v)
        except Gap as g:
            rec['gap'] = str(g)
        else:
            if LOSSY[0]:
                rec['lossy'] = 'a lone surrogate, as U+FFFD'
            elif dump and not shared(v):
                rec['dump'] = yaml.safe_dump(v, allow_unicode=True, sort_keys=False)
    YAML.append(rec)
    kind = 'v' if 'v' in rec else 'err' if 'err' in rec else 'gap'
    counts[f'yaml {kind}'] = counts.get(f'yaml {kind}', 0) + 1


def jcase(group, cid, text):
    rec = {'id': f'{group}/{cid}', 'text': text}
    try:
        v = json.loads(text)
    except json.JSONDecodeError as e:
        rec['err'] = str(e)
    else:
        LOSSY[0] = False
        try:
            rec['v'] = enc(v)
        except Gap as g:
            rec['gap'] = str(g)
        if LOSSY[0]:
            rec['lossy'] = 'a lone surrogate, as U+FFFD'
    JSON.append(rec)
    kind = 'v' if 'v' in rec else 'err' if 'err' in rec else 'gap'
    counts[f'json {kind}'] = counts.get(f'json {kind}', 0) + 1


# ---- YAML: every style, by hand ---------------------------------------------------------
STYLES = [
    '', ' ', '\n', '# only a comment\n', '---\n', '--- \n...\n', '...\n', '---\n...\n---\n', 'a\n...\n',
    '--- a\n', '---\na: 1\n', 'a: 1\n---\nb: 2\n', '--- |\n  x\n', '--- >\n  x\n  y\n',
    'a: 1\nb: 2\n', 'a:\n  b:\n    c: d\n', 'a: {b: c, d: [e, f]}\n', '{a: 1, b: [2, 3]}', '[1, [2, [3]], {a: b}]',
    '[a, b, ]', '{a: b, }', '[a: b, c: d]', '[? a : b]', '{? a : b, ? c}', '{a, b: c}', '[]', '{}', '[ ]', '{ }',
    '- a\n- b\n', '- - a\n  - b\n- c\n', '- a: 1\n  b: 2\n- c: 3\n', 'a:\n- b\n- c\nd: e\n', 'a:\n  - b\n  -\n  - c\n',
    '-\n-\n', '- \n- x\n', 'a:\nb:\n', 'a: \nb: ~\n', '? a\n: b\n', '? a\n', '? - a\n  - b\n: c\n', '? |\n  k\n: v\n',
    'a: b # c\n', 'a: b#c\n', 'a: "b # c"\n', "a: 'b'' c'\n", 'a: "b\\"c\\\\d"\n', 'a: "\\x41\\u00e9\\U0001F600"\n',
    'a: "\\0\\a\\b\\t\\n\\v\\f\\r\\e\\ \\/\\N\\_\\L\\P"\n', 'a: "\\q"\n', 'a: "\\x4"\n', 'a: "\\ud800"\n',
    'a: "line1\n  line2"\n', 'a: "line1\n\n  line2"\n', "a: 'line1\n  line2'\n", "a: 'line1\n\n\n  line2'\n",
    'a: "x \\\n  y"\n', 'a: "trailing   \n  next"\n', 'a: plain\n  continued\n', 'a: plain\n\n  continued\n',
    'a: plain\n  # not a comment? \n', 'a: x\n b\n', 'a: x\n  b: c\n', '- x\n  y\n', 'a b c\nd e\n', 'x\n\n\ny\n',
    'a: |\n  one\n  two\n', 'a: |-\n  one\n  two\n', 'a: |+\n  one\n  two\n\n\n', 'a: |\n  one\n\n  two\n',
    'a: |2\n    indented\n   less\n', 'a: |1\n  x\n', 'a: >\n  folded\n  text\n\n  para\n', 'a: >-\n  a\n   b\n  c\n',
    'a: >+\n  a\n\n', 'a: >\n\n  lead\n', 'a: |\n\n  lead\n', 'a: |\nb: 1\n', 'a: |0\n  x\n', 'a: |x\n',
    'a: > # comment\n  x\n', '- |\n  a\n- >\n  b\n', 'a: |\n  tab\there\n', 'a: |\n   deeper\n  shallow\n',
    'a: &x 1\nb: *x\n', 'a: &x [1, 2]\nb: *x\n', 'a: &x {k: v}\nb: *x\n', 'a: *x\n', 'a: &x 1\nb: &x 2\n',
    '&a [*a]', '&a {k: *a}', 'a: &e\nb: *e\n', '- &a !!str 1\n- *a\n', '- !!str &a 1\n- *a\n',
    'base: &b {x: 1, y: 2}\nd:\n  <<: *b\n  y: 3\n', 'b1: &b1 {x: 1}\nb2: &b2 {x: 2, z: 2}\nd:\n  <<: [*b1, *b2]\n  w: 0\n',
    'd:\n  y: 3\n  <<: {x: 1, y: 2}\n', 'd:\n  <<: 1\n', 'd:\n  <<: [1]\n', 'a: <<\n', "'<<': 1\n", 'd: {<<: {a: 1}, b: 2}\n',
    'a: =\n', '=: 1\n', "'=': 1\n", 'd:\n  <<: {a: {<<: {b: 1}}}\n',
    'a: 1\na: 2\nb: 3\n', '{a: 1, b: 2, a: 3}', 'a: 1\n<<: {a: 9, c: 3}\n',
    'a: !!str 1\nb: !!int "2"\nc: !!float "3"\nd: !!bool "yes"\ne: !!null "x"\n', 'a: !!int 0x1F\nb: !!int 0o17\nc: !!int 0B1\n',
    'a: !!int " 12 "\nb: !!int +-1\nc: !!int ""\n', 'a: !!float 1e400\nb: !!float "-0"\nc: !!float nan\n', 'a: !!bool y\n',
    'a: !!timestamp 2026-1-2\nb: !!timestamp "2026-10-30 1:02:03"\n', 'a: !!timestamp x\n', 'a: !!seq [1]\nb: !!map {k: v}\n',
    'a: !!seq {k: v}\n', 'a: !!map [1]\n', 'a: !!str [1]\n', 'a: !!binary aGVsbG8=\n', 'a: !!set {x, y}\n',
    'a: !!omap [{x: 1}]\n', 'a: !!pairs [{x: 1}]\n', 'a: !foo 1\n', 'a: !<tag:yaml.org,2002:str> 1\n', 'a: ! 12\n',
    'a: ! "12"\n', 'a: !\nb: !!str\n', 'a: !!python/str x\n', 'a: !!int\n', 'a: !!null\n',
    '%YAML 1.1\n---\na: 1\n', '%YAML 1.2\n--- a\n', '%YAML 2.0\n--- a\n', '%YAML 1.1\n%YAML 1.1\n--- a\n',
    '%TAG !e! tag:example.com,2000:\n--- !e!x 1\n', '%TAG !! tag:yaml.org,2002:\n--- !!int "5"\n', '%FOO bar\n--- a\n',
    '%YAML 1.1\na: 1\n', 'a: !e!x 1\n', '\ufeffa: 1\n', 'a: 1\r\nb: 2\r\n', 'a: |\r\n  x\r\n  y\r\n', 'a: 1\rb: 2\r',
    'a: x\u2028y\n', 'a: "x\u2028y"\n', 'a: x\x85y\n', 'a:\tb\n', '\ta: b\n', 'a: b\t\n', 'a:  \t b\n', 'a: [b,\tc]\n',
    'a: 1\n b: 2\n', 'a:\n  b: 1\n c: 2\n', '- a\nb: c\n', 'a: b: c\n', 'a: - b\n', 'a: [b\n', 'a: {b: c\n', 'a: ]\n',
    ']\n', '}\n', '[a, b]: c\n', '{a: b}: c\n', '? [a, b]\n: c\n', '"a": b\n', "'a': b\n", '"a\nb": c\n',
    'a: "unterminated\n', "a: 'unterminated\n", 'a: @x\n', 'a: `x\n', 'a: %x\n', 'a: x%\n', 'a: x@y\n', 'a: -x\n',
    'a: - \n', 'a: ?x\n', 'a: :x\n', 'a: x:y\n', 'a: x: y\n', '[a:b]\n', '[a: b]\n', '{a:b}\n', '{"a":b}\n', '[x, ?y]\n',
    'a: 1\n...\n', 'a: 1\n... # end\n', 'a: "x\n---\ny"\n', 'a: x\n---\n', 'a: b\n...\nc\n', '--- &a\n', '--- !!str\n',
    'a: ' + 'x' * 2000 + ': y\n', 'k' * 1100 + ': v\n', 'a: ' + ' '.join(['w'] * 300) + '\n',
    '1: a\n', 'null: a\n', '~: a\n', 'yes: a\n', '1.5: a\n', '2026-10-30: a\n', '[1]: a\n', '? {a: b}\n: c\n',
    'a: 123456789012345678901234567890\n', 'a: -9223372036854775808\n', 'a: 9223372036854775807\n', 'a: 9223372036854775808\n',
    'a: 0x7fffffffffffffff\n', 'a: 0x8000000000000000\n', 'a: 1:0:0:0:0:0:0:0:0:0:0:0\n', 'a: 1_000_000\n', 'a: 1__0\n',
    'a: 00\n', 'a: 08\n', 'a: 0_1\n', 'a: +0\n', 'a: -0\n', 'a: -0.0\n', 'a: 0.\n', 'a: .5\n', 'a: 1.5e3\n', 'a: 1e3\n',
    'a: 6.8523015e+5\n', 'a: 685.230_15e+03\n', 'a: 685_230.15\n', 'a: 190:20:30.15\n', 'a: -1:30.5\n', 'a: 1:60\n',
    'a: .inf\nb: -.Inf\nc: +.INF\nd: .NaN\n', 'a: 0.1\nb: 0.30000000000000004\nc: 5e-324\nd: 1.7976931348623157e+308\n',
    'a: 1e309\n', 'a: 1.0e-400\n', 'a: 2.2250738585072014e-308\n',
]
for i, t in enumerate(STYLES):
    ycase('style', str(i), t)

# ---- the dumper's look-alikes, in five places --------------------------------------
from_fmt = json.load(open(a.pyfmt))
LOOK = []
for c in from_fmt['cases']:
    if c['id'].startswith('edge/str') and c['v'][0] == 'S':
        LOOK.append(c['v'][1])
for i, s in enumerate(LOOK):
    for j, fmt in enumerate(('{}', 'k: {}\n', '- {}\n', '[{}]', '{{k: {}}}')):
        ycase('look', f'{i}.{j}', fmt.format(s), dump=False)

# ---- explicit tags on a set of values ---------------------------------------------
VALUES = ['1', '-1', '0b11', '0x1f', '017', '1_0', '1:30', '1.5', '1e3', '.inf', '-.nan', 'yes', 'No', 'on', 'x', '',
          '2026-10-30', '2026-10-30 10:00:00', '~', 'null']
for t in ('str', 'int', 'float', 'bool', 'null', 'timestamp'):
    for i, s in enumerate(VALUES):
        ycase('tag', f'{t}.{i}', f'!!{t} "{s}"\n', dump=False)

# ---- timestamps -------------------------------------------------------------------------
TIMES = [
    '2026-10-30', '2024-02-29', '2026-02-29', '2026-13-01', '2026-00-10', '2026-10-00', '2026-10-32', '0000-01-01',
    '0001-01-01', '9999-12-31', '2026-10-30T10:15:00', '2026-10-30t10:15:00', '2026-10-30 10:15:00',
    '2026-10-30\t10:15:00', '2026-10-30   10:15:00', '2026-1-2T3:04:05', '2026-10-30T10:15:00.', '2026-10-30T10:15:00.1',
    '2026-10-30T10:15:00.123456', '2026-10-30T10:15:00.1234567', '2026-10-30T10:15:00.000000', '2026-10-30T10:15:00Z',
    '2026-10-30T10:15:00 Z', '2026-10-30T10:15:00+8', '2026-10-30T10:15:00+08', '2026-10-30T10:15:00-05:30',
    '2026-10-30T10:15:00 -05:30', '2026-10-30T10:15:00+00:00', '2026-10-30T10:15:00-00:00', '2026-10-30T10:15:00+23:59',
    '2026-10-30T10:15:00+24:00', '2026-10-30T10:15:00+05:75', '2026-10-30T10:15:00.5+0530', '2026-10-30T24:00:00',
    '2026-10-30T23:60:00', '2026-10-30T23:59:60', '2026-10-30T10:15', '2026-10-30T10:15:00 ', '2001-12-14t21:59:43.10-05:00',
    '2001-12-14 21:59:43.10 -5', '2001-12-15 2:59:43.10',
]
for i, t in enumerate(TIMES):
    ycase('time', f'{i}', t + '\n')
    ycase('time', f'{i}.k', f'when: {t}\nl: [{t}]\n')
    ycase('time', f'{i}.q', f'when: "{t}"\n')

# ---- real files: the repository's YAML and the recordings' -----------------------------
paths = sorted(glob.glob('/repo/src/mower_recorder/config/*.yaml')) + sorted(glob.glob('/repo/src/mower_bringup/config/*.yaml'))
for p in sorted(glob.glob('/golden/F1/*/**/*.yaml', recursive=True)) + sorted(glob.glob('/golden/F3/*/**/*.yaml', recursive=True)):
    name = os.path.basename(p)
    # rosbag2's metadata.yaml: the small ones (F1's snapshots, F3's camera)
    if name == 'metadata.yaml' and os.path.getsize(p) > 5000:
        continue
    paths.append(p)
for p in paths:
    rel = p.replace('/repo/', '').replace('/golden/', 'golden/')
    ycase('file', rel, open(p, encoding='utf-8').read())

# ---- the dumper's outputs loaded back ------------------------------------------------
def sorted_keys(v):
    if isinstance(v, dict):
        return {k: sorted_keys(v[k]) for k in sorted(v)}
    if isinstance(v, list):
        return [sorted_keys(x) for x in v]
    return v


same = differ = 0
for c in from_fmt['cases']:
    orig = dec(c['v'])
    rec = {'case': c['id'], 'same': [], 'sorted': [], 'other': {}}
    for combo, text in c['out'].items():
        try:
            v = yaml.safe_load(text) if combo.startswith('y') else json.loads(text)
        except Exception as e:  # noqa: BLE001
            rec['other'][combo] = {'err': f'{type(e).__name__}: {e}'.splitlines()[0][:200]}
            differ += 1
            continue
        if strict_eq(v, orig):
            rec['same'].append(combo)
            same += 1
        elif strict_eq(v, sorted_keys(orig)):
            # a sort_keys dump loads back sorted
            rec['sorted'].append(combo)
            same += 1
        else:
            try:
                rec['other'][combo] = {'v': enc(v)}
            except Gap as g:
                rec['other'][combo] = {'gap': str(g)}
            differ += 1
    for k in ('same', 'sorted', 'other'):
        if not rec[k]:
            del rec[k]
    REFS.append(rec)

# ---- JSON ------------------------------------------------------------------------------------
JSON_EDGE = [
    '', ' ', '{}', '[]', 'null', 'true', 'false', 'NaN', 'Infinity', '-Infinity', '-NaN', 'nan', 'infinity', '1', '-1', '0',
    '-0', '00', '01', '-01', '1.5', '1.', '.5', '1e5', '1E+5', '1e-5', '1e', '1e+', '-', '--1', '+1', '1.5e400', '-1e400',
    '9223372036854775807', '-9223372036854775808', '9223372036854775808', '123456789012345678901234567890', '1_0',
    '"a"', '"\\u00e9"', '"\\ud83d\\ude00"', '"\\ud83d"', '"\\ude00"', '"\\ud83dx"', '"\\u12"', '"\\u12G4"', '"\\x41"',
    '"\\/\\b\\f\\n\\r\\t\\"\\\\"', '"a\nb"', '"a\tb"', '"\x7f"', '"é中"', '"unterminated', '"a\\', '"\\', '["a", ]', '[1,]',
    '[1 2]', '{"a" 1}', '{"a": 1,}', '{a: 1}', "{'a': 1}", '{"a": 1, "b": 2, "a": 3}', '{"a": {"b": [1, {"c": null}]}}',
    '[1, [2, [3, [4]]]]', ' [1] ', '[1] x', '1 2', '\ufeff[1]', '\ufeff', '[\n1,\n2', '{"a":\n', 'tru', 'nul', 'falsey',
    '[NaN, Infinity, -Infinity]', '{"": ""}', '{"a": 1}{"b": 2}', '\n\n  x', '[1, 2,\n 3, @]',
]
for i, t in enumerate(JSON_EDGE):
    jcase('edge', str(i), t)
LIMIT = {'/graph_snapshot': 0, '/mower_recorder/bags': 4, '/mower_recorder/status': 6}
per = {}
for p in sorted(glob.glob('/golden/F*/listen.jsonl.zst')):
    raw = zstandard.ZstdDecompressor().decompressobj().decompress(open(p, 'rb').read()).decode('utf-8')
    run = p.split('/')[-2]
    for n, line in enumerate(raw.splitlines()):
        m = json.loads(line)
        if per.get(m['topic'], 0) >= LIMIT.get(m['topic'], 4):
            continue
        per[m['topic']] = per.get(m['topic'], 0) + 1
        jcase('real', f'{run}{m["topic"]}#{n}', m['data'])

doc = {
    'generator': 'tests/oracle/pyload_gen.py',
    'python': sys.version.split()[0],
    'pyyaml': yaml.__version__,
    'yaml': YAML,
    'json': JSON,
    'refs': REFS,
}
with open(a.out, 'w', encoding='ascii') as f:
    json.dump(doc, f, ensure_ascii=True, separators=(',', ':'))
    f.write('\n')
print(f'{len(YAML)} yaml cases, {len(JSON)} json cases, refs to {len(REFS)} cases ({same} as dumped, {differ} not); {counts}; '
      f'{os.path.getsize(a.out)} bytes')

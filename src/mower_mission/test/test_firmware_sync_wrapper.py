"""utils/firmware-sync's switch (E6-4): MOWER_FIRMWARE_SYNC picks the flasher.

  0 / false / no / off   skip
  python / rust          firmware/tools/mower_flash.py / its Rust port mower_flash
  anything else          the image's default: rust since E6-5 (python is the roll-back)

Runs the wrapper with PATH holding nothing but fakes: `python3` and
`mower_flash` log their argv and answer the way the real ones do (the
manifest's "version sha8"; FAKE_RC as the sync's exit code), `mkdir` is the
real one. The wrapper's firmware directory is pointed at a temporary one, the
only line of it changed; the transcripts name the temporary paths by the
production ones. Each case's calls, stdout, stderr and exit code are held to
deploy/test/golden/firmware_sync_wrapper.json. Its python and skip cases are
what the wrapper did before the switch (c90789b); since E6-5 the values that
name no flasher, the robot's MOWER_FIRMWARE_SYNC=1 among them, are the rust
cases'. Rewrite it only when the wrapper changes on purpose:

    python3 src/mower_mission/test/test_firmware_sync_wrapper.py --write-golden
"""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
WRAPPER = REPO / 'utils' / 'firmware-sync'
GOLDEN = REPO / 'deploy' / 'test' / 'golden' / 'firmware_sync_wrapper.json'
FW = '/opt/mower/firmware'
FW_LINE = f'dir={FW}\n'
DEFAULT_LINE = 'default=rust\n'
PORT = '/dev/stmcom'
STATE = '/home/mower/.mower'
USING_RUST = '[firmware-sync] using mower_flash (rust)'
MANIFEST = {'version': '0.6.0-e64', 'semver': '0.6.0', 'git_sha': 'e64e64e6aa11bb22cc33dd44ee55ff6677889900',
            'git_sha32': 0xe64e64e6, 'build_unix': 1793000000, 'dirty': 0}
SYNC_ARGS = ['-p', PORT, 'sync', f'{FW}/mower_robot_firmware.bin', '--report', f'{STATE}/firmware_sync.json']

# python3 and mower_flash in one script, told apart by the name it runs as
FAKE = '''#!{python}
import json, os, sys
tool = os.path.basename(sys.argv[0])
args = sys.argv[1:]
with open(os.environ['FAKE_LOG'], 'a') as f:
    f.write(json.dumps([tool] + args) + '\\n')
if tool == 'python3' and args[:1] == ['-c']:
    os.execv(sys.executable, [sys.executable] + args)  # the wrapper's manifest one-liner, for real
if tool == 'mower_flash' and args[:1] == ['manifest'] and len(args) == 2:
    # what the real one prints (the image smoke test in build.yml holds it to the python3 -c)
    d = json.load(open(args[1]))
    print(d['version'], d['git_sha'][:8])
    sys.exit(0)
flasher = (tool == 'python3' and args[:1] and args[0].endswith('/mower_flash.py')) or tool == 'mower_flash'
if not flasher:
    print('unexpected call', file=sys.stderr)
    sys.exit(99)
print('(flasher stdout)', flush=True)
print('(flasher stderr)', file=sys.stderr, flush=True)
sys.exit(int(os.environ['FAKE_RC']))
'''

# name: (MOWER_FIRMWARE_SYNC or None for unset, the sync's exit code, what is missing)
CASES = {
    'skip-0': ('0', 0, ()),
    'skip-false': ('false', 0, ()),
    'skip-no': ('no', 0, ()),
    'skip-off': ('off', 0, ()),
    'default-unset': (None, 0, ()),
    'default-empty': ('', 0, ()),
    'default-1': ('1', 0, ()),
    'default-1-rc1': ('1', 1, ()),
    'default-1-rc2': ('1', 2, ()),
    'default-true': ('true', 0, ()),
    'default-yes': ('yes', 0, ()),
    'default-on': ('on', 0, ()),
    'default-Rust': ('Rust', 0, ()),
    'default-PYTHON': ('PYTHON', 0, ()),
    'default-2': ('2', 0, ()),
    'default-unset-no-port': (None, 0, ('port',)),
    'default-unset-no-bundle': (None, 0, ('bundle',)),
    'python-rc0': ('python', 0, ()),
    'python-rc1': ('python', 1, ()),
    'python-rc2': ('python', 2, ()),
    'python-no-port': ('python', 0, ('port',)),
    'python-no-python3': ('python', 0, ('python3',)),
    'rust-rc0': ('rust', 0, ()),
    'rust-rc1': ('rust', 1, ()),
    'rust-rc2': ('rust', 2, ()),
    'rust-no-port': ('rust', 0, ('port',)),
    'rust-no-bundle': ('rust', 0, ('bundle',)),
    'rust-no-python3': ('rust', 0, ('python3',)),
    'rust-no-mower_flash': ('rust', 0, ('mower_flash',)),
}


def run_case(tmp, value, rc, missing, wrapper=WRAPPER, default=None):
    """One wrapper run; returns its transcript."""
    tmp = Path(tmp)
    fw, bindir, state, port = tmp / 'fw', tmp / 'bin', tmp / 'state', tmp / 'stmcom'
    fw.mkdir()
    bindir.mkdir()
    (fw / 'mower_robot_firmware.json').write_text(json.dumps(MANIFEST, indent=2))
    (fw / 'mower_flash.py').write_text('raise SystemExit("never run: python3 is a fake")\n')
    if 'bundle' not in missing:
        (fw / 'mower_robot_firmware.bin').write_bytes(bytes(range(64)))
    if 'port' not in missing:
        port.write_bytes(b'')
    fake = bindir / 'fake'
    fake.write_text(FAKE.format(python=sys.executable))
    fake.chmod(0o755)
    for tool in ('python3', 'mower_flash'):
        if tool not in missing:
            (bindir / tool).symlink_to(fake)
    (bindir / 'mkdir').symlink_to(shutil.which('mkdir'))
    text = Path(wrapper).read_text()
    assert text.count(FW_LINE) == 1, 'the wrapper no longer sets its firmware dir in one line'
    text = text.replace(FW_LINE, f'dir={fw}\n')
    if default is not None:
        assert text.count(DEFAULT_LINE) == 1, 'the wrapper no longer sets its default in one line'
        text = text.replace(DEFAULT_LINE, f'default={default}\n')
    script = tmp / 'firmware-sync'
    script.write_text(text)
    log = tmp / 'calls.jsonl'
    env = {'PATH': str(bindir), 'HOME': str(tmp / 'home'), 'MOWER_STM_PORT': str(port),
           'MOWER_STATE_DIR': str(state), 'FAKE_LOG': str(log), 'FAKE_RC': str(rc)}
    if value is not None:
        env['MOWER_FIRMWARE_SYNC'] = value
    p = subprocess.run(['/bin/bash', str(script)], env=env, capture_output=True, text=True, timeout=60)

    def name(s):
        for path, label in ((script, '/usr/local/bin/firmware-sync'), (fw, FW), (state, STATE), (port, PORT)):
            s = s.replace(str(path), label)
        return re.sub(r'^(/usr/local/bin/firmware-sync: line )\d+: ', r'\1N: ', s)

    calls = [json.loads(ln) for ln in log.read_text().splitlines()] if log.exists() else []
    return {
        'MOWER_FIRMWARE_SYNC': value,
        'sync_rc': rc,
        'missing': list(missing),
        'calls': [[name(a) for a in c] for c in calls],
        'stdout': [name(ln) for ln in p.stdout.splitlines()],
        'stderr': [name(ln) for ln in p.stderr.splitlines()],
        'rc': p.returncode,
        'state_dir_created': state.is_dir(),
    }


def run_all(tmp, wrapper=WRAPPER):
    out = {}
    for case, (value, rc, missing) in CASES.items():
        d = Path(tmp) / case
        d.mkdir()
        out[case] = run_case(d, value, rc, missing, wrapper)
    return out


def golden_text(about, cases):
    """JSON, one call per line."""
    def value(k, v):
        if k in ('calls', 'stdout', 'stderr') and v:
            return '[\n' + ',\n'.join('    ' + json.dumps(x) for x in v) + '\n   ]'
        return json.dumps(v)
    body = ',\n'.join(
        f'  {json.dumps(case)}: {{\n' + ',\n'.join(f'   {json.dumps(k)}: {value(k, v)}' for k, v in t.items())
        + '\n  }' for case, t in cases.items())
    return f'{{\n "about": {json.dumps(about)},\n "cases": {{\n{body}\n }}\n}}\n'


@pytest.fixture(scope='module')
def transcripts(tmp_path_factory):
    return run_all(tmp_path_factory.mktemp('wrapper'))


def test_every_case_matches_the_golden(transcripts):
    golden = json.loads(GOLDEN.read_text())['cases']
    assert sorted(transcripts) == sorted(golden)
    for case in CASES:
        assert transcripts[case] == golden[case], case


def test_the_container_always_starts(transcripts):
    for case, t in transcripts.items():
        assert t['rc'] == 0, case


def test_the_skip_values_run_nothing(transcripts):
    for case in ('skip-0', 'skip-false', 'skip-no', 'skip-off'):
        t = transcripts[case]
        assert t['calls'] == [] and not t['state_dir_created'], case
        assert t['stdout'] == ['[firmware-sync] disabled by MOWER_FIRMWARE_SYNC'], case


def _flashers(t):
    return [c[0] if c[0] == 'mower_flash' else Path(c[1]).name for c in t['calls']
            if c[1:2] != ['-c'] and c[1:2] != ['manifest']]


def test_each_value_runs_one_flasher_and_only_that(transcripts):
    for case, (value, _, missing) in CASES.items():
        t = transcripts[case]
        if case.startswith('skip-') or {'port', 'bundle'} & set(missing):
            assert t['calls'] == [], case
        elif value == 'python':
            assert all(c[0] == 'python3' for c in t['calls']), case
            assert _flashers(t) == ([] if 'python3' in missing else ['mower_flash.py']), case
        else:
            # rust, named or the default: nothing of python, not even the log line's manifest
            assert all(c[0] == 'mower_flash' for c in t['calls']), case
            assert _flashers(t) == ([] if 'mower_flash' in missing else ['mower_flash']), case


def test_rust_differs_from_python_by_its_one_line(transcripts):
    for rc in (0, 1, 2):
        py, rs = transcripts[f'python-rc{rc}'], transcripts[f'rust-rc{rc}']
        assert rs['stdout'][0] == USING_RUST
        assert rs['stdout'][1:] == py['stdout']
        assert rs['stderr'] == py['stderr']
        # the same sync command line after the program
        assert py['calls'][-1] == ['python3', f'{FW}/mower_flash.py'] + SYNC_ARGS
        assert rs['calls'][-1] == ['mower_flash'] + SYNC_ARGS
        # and the same "version sha8" in the bundled line
        assert rs['stdout'][1] == f'[firmware-sync] bundled 0.6.0-e64 e64e64e6 on {PORT}'
    assert USING_RUST not in transcripts['python-rc0']['stdout']


def test_a_value_naming_no_flasher_is_rust(transcripts):
    """1 (the robot's), unset and anything else run exactly what rust runs."""
    for case, (value, rc, missing) in CASES.items():
        if case.startswith(('skip-', 'python-', 'rust-')):
            continue
        rust = next(t for c, t in transcripts.items()
                    if CASES[c][0] == 'rust' and CASES[c][1:] == (rc, missing))
        assert {**transcripts[case], 'MOWER_FIRMWARE_SYNC': 'rust'} == rust, case


def test_a_missing_rust_flasher_fails_the_sync_without_python(transcripts):
    t = transcripts['rust-no-mower_flash']
    assert t['calls'] == []
    assert t['stdout'][0] == USING_RUST
    assert t['stdout'][-1] == ('[firmware-sync] FAILED (rc=127); continuing, '
                               'the driver will report the mismatch')
    assert t['stderr'] == ['/usr/local/bin/firmware-sync: line N: mower_flash: command not found']


def test_the_robot_value_leaves_the_choice_to_the_image():
    """The robot's .env has MOWER_FIRMWARE_SYNC=1, as .env.example does, and
    compose passes it with 1 as its default: the image's default decides."""
    env = (REPO / 'deploy' / '.env.example').read_text(encoding='utf-8')
    assert re.findall(r'^MOWER_FIRMWARE_SYNC=(.*)$', env, re.M) == ['1']
    compose = (REPO / 'deploy' / 'docker-compose.yaml').read_text(encoding='utf-8')
    assert '- MOWER_FIRMWARE_SYNC=${MOWER_FIRMWARE_SYNC:-1}' in compose
    assert CASES['default-1'][0] == '1'
    assert WRAPPER.read_text(encoding='utf-8').count(DEFAULT_LINE) == 1


def test_the_default_is_one_line(tmp_path):
    """With `default=python` (E6-4's image) the values that name no flasher
    run python again; python and rust stay what they name."""
    for case, (value, rc, missing) in CASES.items():
        if case.startswith('skip-') or missing:
            continue
        d = tmp_path / case
        d.mkdir()
        t = run_case(d, value, rc, missing, default='python')
        want = 'mower_flash' if value == 'rust' else 'python3'
        assert t['calls'] and all(c[0] == want for c in t['calls']), case
        assert (USING_RUST in t['stdout']) == (want == 'mower_flash'), case


if __name__ == '__main__':
    import tempfile
    if sys.argv[1:] != ['--write-golden']:
        sys.exit('usage: test_firmware_sync_wrapper.py --write-golden')
    with tempfile.TemporaryDirectory() as tmp:
        cases = run_all(tmp)
    GOLDEN.write_text(golden_text(
        'utils/firmware-sync per MOWER_FIRMWARE_SYNC value (E6-4), written by '
        'src/mower_mission/test/test_firmware_sync_wrapper.py: the fakes python3 and mower_flash '
        'on an otherwise empty PATH, the sync exiting sync_rc, `missing` naming what is absent '
        '(the port, the bundled .bin, a tool), temporary paths shown as the production ones. '
        'The python and skip cases are what the wrapper did before E6-4 (c90789b); since E6-5 '
        '(default=rust) the values that name no flasher, 1 among them, run what rust runs.', cases))
    print(f'wrote {GOLDEN}')

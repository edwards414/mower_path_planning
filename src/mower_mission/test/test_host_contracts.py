"""The file contracts between the robot's host and its container, frozen as
golden files (E6 design 4.3, G2-G6) so the Python and shell host tools can be
replaced without anyone noticing:

  G2  host.request: its writers (utils/mower-host-request,
      mower_mission/host_request.py, and in Rust write_host_request, whose test
      reads the same golden) and its consumer deploy/host/mower-host-request.sh
  G3  update_status / update_check / image.json: deploy/host/mower-update.sh
  G4  link_status.json: deploy/host/mower-link-status.py
  G5  identity.json and the pairing URL: deploy/host/mower-pair
  G6  robot_status.json -> mower-update.sh --auto, and its json_field

Every case runs the real tool. The shell tools get a PATH holding only fakes
(docker, systemctl, logger, date, ...) and the few real tools they need, so
nothing here can reach a real docker or systemctl. The Python tools run under a
small driver that replaces what they read from the machine: machine-id,
hostname, randomness, the clock, the LAN address, /proc/net/wireless; the modem
is a fake on a real pty. Temporary paths are shown as the production ones. Each
case's calls, files, stdout, stderr and exit code are held to the JSON files in
deploy/test/golden/host/. The Rust ports read the same files (E6-7, G4 and G5).

These pin today's behaviour, bugs included (the cases say so where it matters).
What the tools do as root is in test_host_state_dir.py; here the state dir has
one owner. Rewrite them only when a tool changes on purpose:

    python3 src/mower_mission/test/test_host_contracts.py --write-golden
"""
import base64
import concurrent.futures
import fcntl
import hashlib
import importlib.util
import json
import os
import re
import select
import shlex
import shutil
import stat
import subprocess
import sys
import tempfile
import threading
import time
import tty
from pathlib import Path
from unittest import mock

import pytest

REPO = Path(__file__).resolve().parents[3]
GOLDEN = REPO / 'deploy' / 'test' / 'golden' / 'host'
HOST = REPO / 'deploy' / 'host'
REQUEST_SH = HOST / 'mower-host-request.sh'
UPDATE_SH = HOST / 'mower-update.sh'
LINK_PY = HOST / 'mower-link-status.py'
PAIR_PY = HOST / 'mower-pair'
SHELL_WRITER = REPO / 'utils' / 'mower-host-request'
PYTHON_WRITER = REPO / 'src' / 'mower_mission' / 'mower_mission' / 'host_request.py'

NOW = 1790000000
STATE = '/home/cat/.mower'
MOWER = '/opt/mower'
AT_PORT = '/dev/lte_at'
# what the real PATH was: only used to find the few real tools a fake PATH holds
REAL_PATH = os.environ.get('PATH', '') + os.pathsep + '/usr/bin:/bin'

# docker, systemctl, logger, sleep, qrencode, the update script, in one script told
# apart by the name it runs as. Every call goes to the log with what
# update_status.json said at that moment and where it was called from.
FAKE = r'''#!@PYTHON@ -IS
import json, os, sys
tool = os.path.basename(sys.argv[0])
args = sys.argv[1:]
spec = json.load(open(os.environ['FAKE_SPEC']))
try:
    state = json.load(open(os.path.join(spec['state'], 'update_status.json'))).get('state')
except (OSError, ValueError):
    state = None
with open(spec['log'], 'a') as f:
    f.write(json.dumps([state, os.getcwd(), tool] + args) + '\n')
if tool == 'systemctl':
    sys.exit(spec.get('systemctl_rc', 0))
if tool == 'qrencode':
    print('<QR>')
    sys.exit(0)
if tool != 'docker':
    sys.exit(0)
d = spec.get('docker', {})
repo = spec.get('repo', '')
pulled = os.path.exists(spec['pulled'])
local = d.get('remote', '') if pulled else d.get('local', '')
tag_id = d.get('pulled_tag_id', d.get('tag_id', '')) if pulled else d.get('tag_id', '')
two = args[:2]
if two == ['image', 'inspect']:
    if '{{.Id}}' in ' '.join(args):
        print(tag_id)
    elif local:
        print(args[-1].rsplit(':', 1)[0] + '@' + local)
elif two == ['manifest', 'inspect']:
    if d.get('manifest_fail'):
        sys.exit(1)
    print(d['manifest_text'] if 'manifest_text' in d else json.dumps({'Descriptor': {'digest': d.get('remote', '')}}))
elif two == ['compose', 'ps']:
    if d.get('running', True):
        print('cid-lawan')
elif two == ['compose', 'up']:
    print('Container lawan_node Started')
    sys.exit(d.get('compose_up_rc', 0))
elif two == ['compose', 'restart']:
    sys.exit(d.get('compose_restart_rc', 0))
elif two == ['image', 'prune']:
    pass
elif args[:1] == ['inspect']:
    print(d.get('running_id', ''))
elif args[:1] == ['pull']:
    fails = 0
    if os.path.exists(spec['pulled'] + '.fails'):
        fails = int(open(spec['pulled'] + '.fails').read())
    if 'pull_fail' in d and fails < d.get('pull_fail_times', 1000):
        open(spec['pulled'] + '.fails', 'w').write(str(fails + 1))
        if d.get('busy_on_pull_fail'):
            with open(os.path.join(spec['state'], 'robot_status.json'), 'w') as f:
                f.write(json.dumps({'busy': True, 'time': d['busy_on_pull_fail']}))
        print(d['pull_fail'])
        sys.exit(1)
    open(spec['pulled'], 'w').close()
    print('Status: Downloaded newer image')
else:
    print('unexpected docker ' + ' '.join(args), file=sys.stderr)
    sys.exit(99)
'''

# flock(1) for a machine without util-linux (macOS): the lock itself is real
FLOCK_SHIM = r'''#!@PYTHON@
import fcntl, sys
assert sys.argv[1] == '-n'
try:
    fcntl.flock(int(sys.argv[2]), fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    sys.exit(1)
'''

# sed -i wants its suffix argument on BSD; only --tag uses it
SED_SHIM = '''#!/bin/sh
if [ "$1" = "-i" ]; then shift; exec /usr/bin/sed -i '' "$@"; fi
exec /usr/bin/sed "$@"
'''


# What the shell tools run to read and write the state dir (as_state_owner, state_write)
STATE_DIR_TOOLS = ('python3', 'sh', 'rm', 'mv', 'cat', 'stat', 'setpriv', 'env')


def fake_bin(bindir, fakes=(), real=()):
    """A directory for PATH: the named fakes, the named real tools, python3."""
    bindir = Path(bindir)
    bindir.mkdir(parents=True, exist_ok=True)
    fake = bindir / '.fake'
    fake.write_text(FAKE.replace('@PYTHON@', sys.executable))
    fake.chmod(0o755)
    (bindir / 'date').write_text(f'#!/bin/sh\n[ "$*" = "+%s" ] || exit 99\necho {NOW}\n')
    (bindir / 'date').chmod(0o755)
    for name in fakes:
        (bindir / name).symlink_to(fake)
    for name in real:
        if name == 'flock' and not shutil.which('flock', path=REAL_PATH):
            (bindir / 'flock').write_text(FLOCK_SHIM.replace('@PYTHON@', sys.executable))
            (bindir / 'flock').chmod(0o755)
        elif name == 'sed' and sys.platform == 'darwin':
            (bindir / 'sed').write_text(SED_SHIM)
            (bindir / 'sed').chmod(0o755)
        elif name == 'python3':
            (bindir / 'python3').write_text(f'#!/bin/sh\nexec {shlex.quote(sys.executable)} "$@"\n')
            (bindir / 'python3').chmod(0o755)
        elif name == 'setpriv' and not shutil.which(name, path=REAL_PATH):
            continue  # util-linux; only a root run of the shell tools reaches it
        else:
            (bindir / name).symlink_to(shutil.which(name, path=REAL_PATH))
    return bindir


def namer(*pairs):
    """Temporary paths -> production names; bash's line numbers -> N, and the
    part of its arithmetic errors that differs between versions (macOS has 3.2)."""
    def name(text):
        for path, label in pairs:
            text = text.replace(str(path), label)
        text = re.sub(r'\(error token is .*\)$', '(error token is ...)', text, flags=re.M)
        return re.sub(r'^(.*: line )\d+: ', r'\1N: ', text, flags=re.M)
    return name


def lines(text):
    """The lines of what a tool printed; only a newline ends one."""
    rows = text.split('\n')
    return rows[:-1] if rows[-1] == '' else rows


def read_or_none(path):
    try:
        return Path(path).read_text(encoding='utf-8')
    except FileNotFoundError:
        return None


def parses(text):
    try:
        json.loads(text)
        return True
    except (TypeError, ValueError):
        return False


def _pretty(v, ind=0):
    """JSON a person can diff: dicts broken up, lists of scalars on one line."""
    pad = ' ' * ind
    if isinstance(v, dict):
        if not v:
            return '{}'
        items = [f'{pad} {json.dumps(k, ensure_ascii=False)}: {_pretty(x, ind + 1)}' for k, x in v.items()]
        return '{\n' + ',\n'.join(items) + f'\n{pad}}}'
    if isinstance(v, list):
        flat = json.dumps(v, ensure_ascii=False)
        if not any(isinstance(x, (dict, list)) for x in v) and (len(flat) <= 110 or len(v) <= 1):
            return flat
        items = [f'{pad} {_pretty(x, ind + 1)}' for x in v]
        return '[\n' + ',\n'.join(items) + f'\n{pad}]'
    return json.dumps(v, ensure_ascii=False)


def golden_text(about, cases, **extra):
    return _pretty({'about': about, **extra, 'cases': cases}) + '\n'


def load_golden(name):
    return json.loads((GOLDEN / name).read_text(encoding='utf-8'))


def run_table(table, one, tmp):
    """`one(dir, case)` for each case of a table, a few at a time."""
    def work(item):
        i, name = item
        d = Path(tmp) / f'{i:03}'  # not the name: macOS folds case
        d.mkdir()
        return name, one(d, table[name])
    old = os.umask(0o022)  # what a service runs under: the modes are part of the contract
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
            return dict(pool.map(work, enumerate(table)))
    finally:
        os.umask(old)


def assert_same(got, want):
    assert sorted(got) == sorted(want), 'the case tables differ'
    for case in want:
        assert got[case] == want[case], case


# ---- G2: host.request ------------------------------------------------------

def _load_python_writer():
    spec = importlib.util.spec_from_file_location('mower_host_request_under_test', PYTHON_WRITER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# what Rust's write_host_request writes for ('update', 'app') at NOW: its keys in
# serde_json's order. mower_rs_common/tests/host_request_golden.rs holds it to this.
RUST_REQUEST_TEXT = '{"action":"update","requested_by":"app","time":%d}\n' % NOW

SHELL_WRITER_CASES = {
    'update': {'argv': ['update']},
    'restart': {'argv': ['restart']},
    'reboot': {'argv': ['reboot']},
    'poweroff': {'argv': ['poweroff']},
    'update-by-app': {'argv': ['update', 'app']},
    'poweroff-by-container': {'argv': ['poweroff', 'container']},
    # not in its list, though the other two writers and the consumer take it
    'check-refused': {'argv': ['check']},
    'unknown-refused': {'argv': ['format-disk']},
    'no-action': {'argv': []},
    # it does not escape: the file is not JSON, and the consumer treats that as nothing asked
    'quote-in-by': {'argv': ['reboot', 'a"b']},
    'unset-state-dir': {'argv': ['update'], 'state_dir': 'home'},
    'missing-state-dir': {'argv': ['update'], 'state_dir': 'missing'},
}

PYTHON_WRITER_CASES = {
    'update': {'action': 'update'},
    'check': {'action': 'check', 'by': 'app'},
    'restart': {'action': 'restart', 'by': 'app'},
    'reboot': {'action': 'reboot', 'by': 'container'},
    'poweroff': {'action': 'poweroff', 'by': 'container'},
    'quote-in-by': {'action': 'reboot', 'by': 'a"b'},
    'non-ascii-by': {'action': 'update', 'by': '前院'},
    'unknown-refused': {'action': 'format-disk'},
    'empty-refused': {'action': ''},
}

RUST_WRITER = {
    'accept': [['update', 'app'], ['check', 'app'], ['restart', 'app'], ['reboot', 'container'],
               ['poweroff', 'container'], ['reboot', 'a"b'], ['update', '前院']],
    'reject': ['format-disk', '', 'UPDATE', 'update '],
    'text_update_app': RUST_REQUEST_TEXT,
}


def run_shell_writer(tmp, case):
    bindir = fake_bin(tmp / 'bin', (), ('mkdir', 'mv'))
    home, state = tmp / 'home', tmp / 'state'
    home.mkdir()
    env = {'PATH': str(bindir), 'HOME': str(home)}
    target = state
    where = case.get('state_dir')
    if where == 'home':
        target = home / '.mower'
    else:
        env['MOWER_STATE_DIR'] = str(state)
        if where != 'missing':
            state.mkdir()
    p = subprocess.run(['/bin/sh', str(SHELL_WRITER)] + case['argv'], env=env, capture_output=True, text=True,
                       timeout=30, cwd=tmp)
    return {'argv': case['argv'], 'rc': p.returncode, 'stdout': lines(p.stdout), 'stderr': lines(p.stderr),
            'file': read_or_none(target / 'host.request'),
            'tmp_left': (target / 'host.request.tmp').exists()}


def run_python_writers(tmp):
    mod = _load_python_writer()
    out = {}
    for name, case in PYTHON_WRITER_CASES.items():
        d = Path(tmp) / name
        kwargs = {'requested_by': case['by']} if 'by' in case else {}
        refused = False
        with mock.patch.object(mod.time, 'time', return_value=NOW + 0.5):
            try:
                path = mod.write_request(case['action'], directory=str(d), **kwargs)
                assert Path(path) == d / 'host.request'
            except ValueError:
                refused = True
        out[name] = {'action': case['action'], 'requested_by': case.get('by'), 'refused': refused,
                     'file': read_or_none(d / 'host.request'), 'tmp_left': (d / 'host.request.tmp').exists()}
    return out


def _req(action, by='app', **more):
    return json.dumps({'action': action, 'time': NOW - 5, 'requested_by': by, **more})


CONSUMER_CASES = {
    'update': {'request': _req('update')},
    'check': {'request': _req('check')},
    'restart': {'request': _req('restart')},
    'reboot': {'request': _req('reboot', 'container')},
    'poweroff': {'request': _req('poweroff', 'container')},
    # the three writers' exact texts
    'text-of-the-shell-writer': {'request': '{"action":"poweroff","time":%d,"requested_by":"container"}\n' % NOW},
    'text-of-the-python-writer': {'request': '{"action": "restart", "time": %d, "requested_by": "ros"}\n' % NOW},
    'text-of-the-rust-writer': {'request': RUST_REQUEST_TEXT},
    # nothing to do
    'no-file': {'request': None},
    'bad-json': {'request': 'not json'},
    'empty-file': {'request': ''},
    'json-array': {'request': '[1, 2]'},
    'no-action-key': {'request': '{"requested_by": "app"}'},
    'empty-action': {'request': _req('')},
    'unknown-action': {'request': _req('format-disk')},
    # json.load's value goes through print: null is the word None, 5 the digit
    'null-action': {'request': '{"action": null}'},
    'number-action': {'request': '{"action": 5}'},
    # the shell writer's text with a quote in it does not parse: nothing asked
    'unparsable-from-the-shell-writer': {'request': '{"action":"reboot","time":%d,"requested_by":"a"b"}\n' % NOW},
    # status() does not escape: update_status.json is not JSON after this one
    'quote-in-by-reboot': {'request': _req('reboot', 'a"b')},
    'quote-in-by-poweroff': {'request': _req('poweroff', 'a"b')},
    'no-by': {'request': '{"action": "reboot"}'},
    'null-by': {'request': '{"action": "reboot", "requested_by": null}'},
    # a failed restart still ends in "restarted"
    'restart-compose-fails': {'request': _req('restart'), 'docker': {'compose_restart_rc': 1}},
    'restart-no-mower-dir': {'request': _req('restart'), 'no_mower_dir': True},
    'reboot-systemctl-fails': {'request': _req('reboot', 'container'), 'systemctl_rc': 1},
    # the earlier status is replaced whatever it said
    'restart-replaces-the-status': {'request': _req('restart'),
                                    'status_before': '{"state":"failed","message":"x","tag":"main","time":1}\n'},
    'check-keeps-the-status': {'request': _req('check'),
                               'status_before': '{"state":"failed","message":"x","tag":"main","time":1}\n'},
}


def run_consumer(tmp, case):
    state, mower, cwd = tmp / 'state', tmp / 'opt_mower', tmp / 'cwd'
    state.mkdir()
    cwd.mkdir()
    bindir = fake_bin(tmp / 'bin', ('logger', 'docker', 'systemctl'), STATE_DIR_TOOLS)
    if not case.get('no_mower_dir'):
        (mower / 'host').mkdir(parents=True)
        (mower / 'host' / 'mower-update.sh').symlink_to(bindir / '.fake')
    log = tmp / 'log'
    spec = {'state': str(state), 'log': str(log), 'pulled': str(tmp / 'pulled'),
            'docker': case.get('docker', {}), 'systemctl_rc': case.get('systemctl_rc', 0)}
    (tmp / 'spec.json').write_text(json.dumps(spec))
    if case['request'] is not None:
        (state / 'host.request').write_text(case['request'], encoding='utf-8')
    if 'status_before' in case:
        (state / 'update_status.json').write_text(case['status_before'])
    p = subprocess.run(['/bin/bash', str(REQUEST_SH), str(state)], capture_output=True, text=True, timeout=30,
                       env={'PATH': str(bindir), 'MOWER_DIR': str(mower), 'FAKE_SPEC': str(tmp / 'spec.json')},
                       cwd=cwd)
    name = namer((mower, MOWER), (state, STATE), (cwd, '<cwd>'), (REQUEST_SH, f'{MOWER}/host/mower-host-request.sh'))
    calls = [json.loads(ln) for ln in log.read_text().splitlines()] if log.exists() else []
    status = read_or_none(state / 'update_status.json')
    return {
        'given': case,
        'calls': [[c[0]] + [name(a) for a in c[1:]] for c in calls],
        'stdout': lines(name(p.stdout)),
        'stderr': lines(name(p.stderr)),
        'rc': p.returncode,
        'update_status': status,
        'update_status_is_json': parses(status) if status is not None else None,
        'request_left': (state / 'host.request').exists(),
        'tmp_left': (state / 'update_status.json.tmp').exists(),
    }


def g2_now(tmp):
    tmp = Path(tmp)
    (tmp / 'shell').mkdir()
    (tmp / 'python').mkdir()
    (tmp / 'consumer').mkdir()
    return {
        'writers': {
            'shell': run_table(SHELL_WRITER_CASES, run_shell_writer, tmp / 'shell'),
            'python': run_python_writers(tmp / 'python'),
            'rust': RUST_WRITER,
        },
        'consumer': run_table(CONSUMER_CASES, run_consumer, tmp / 'consumer'),
    }


@pytest.fixture(scope='module')
def g2(tmp_path_factory):
    return g2_now(tmp_path_factory.mktemp('g2'))


def test_g2_shell_writer_matches_the_golden(g2):
    assert_same(g2['writers']['shell'], load_golden('g2_host_request.json')['writers']['shell'])


def test_g2_python_writer_matches_the_golden(g2):
    assert_same(g2['writers']['python'], load_golden('g2_host_request.json')['writers']['python'])


def test_g2_rust_writers_table_is_the_goldens():
    assert load_golden('g2_host_request.json')['writers']['rust'] == RUST_WRITER


def test_g2_consumer_matches_the_golden(g2):
    assert_same(g2['consumer'], load_golden('g2_host_request.json')['cases'])


def test_g2_the_writers_say_the_same_thing(g2):
    shell, python = g2['writers']['shell'], g2['writers']['python']
    want = {'action': 'poweroff', 'time': NOW, 'requested_by': 'container'}
    assert json.loads(shell['poweroff-by-container']['file']) == want
    assert json.loads(python['poweroff']['file']) == want
    assert json.loads(RUST_REQUEST_TEXT) == {'action': 'update', 'time': NOW, 'requested_by': 'app'}
    assert json.loads(shell['update-by-app']['file']) == {'action': 'update', 'time': NOW, 'requested_by': 'app'}
    # the integer, whatever the writer
    assert type(json.loads(python['poweroff']['file'])['time']) is int
    for writer in (shell, python):
        for case, t in writer.items():
            assert not t['tmp_left'], case
    for case in ('check-refused', 'unknown-refused', 'no-action'):
        assert shell[case]['rc'] == 2 and shell[case]['file'] is None, case
    assert python['unknown-refused']['refused'] and python['unknown-refused']['file'] is None
    assert python['empty-refused']['refused']


def test_g2_the_python_writer_escapes_and_the_shell_writer_does_not(g2):
    assert json.loads(g2['writers']['python']['quote-in-by']['file'])['requested_by'] == 'a"b'
    assert not parses(g2['writers']['shell']['quote-in-by']['file'])


def test_g2_every_case_deletes_the_request(g2):
    for case, t in g2['consumer'].items():
        assert not t['request_left'], case
        assert not t['tmp_left'], case


def test_g2_the_status_is_written_before_what_it_announces(g2):
    def tools(case):
        return [(c[0], c[2:]) for c in g2['consumer'][case]['calls'] if c[2] != 'logger']
    assert tools('restart') == [('restarting', ['docker', 'compose', 'restart', 'lawan_node'])]
    assert json.loads(g2['consumer']['restart']['update_status'])['state'] == 'idle'
    assert tools('reboot') == [('rebooting', ['systemctl', 'reboot'])]
    assert tools('poweroff') == [('powering_off', ['systemctl', 'poweroff'])]
    assert tools('update') == [(None, ['mower-update.sh'])]
    assert tools('check') == [(None, ['mower-update.sh', '--check'])]
    # compose runs in the compose file's directory
    assert [c[1] for c in g2['consumer']['restart']['calls'] if c[2] == 'docker'] == [MOWER]


def test_g2_the_status_of_this_script_has_no_tag(g2):
    assert json.loads(g2['consumer']['restart']['update_status']) == {'state': 'idle', 'message': 'restarted', 'time': NOW}
    assert json.loads(g2['consumer']['reboot']['update_status']) == {
        'state': 'rebooting', 'message': 'host reboot requested by container', 'time': NOW}
    assert json.loads(g2['consumer']['poweroff']['update_status'])['message'] == 'host power-off requested by container'


def test_g2_what_nothing_asks_for_does_nothing(g2):
    for case in ('bad-json', 'empty-file', 'json-array', 'no-action-key', 'empty-action',
                 'unparsable-from-the-shell-writer'):
        t = g2['consumer'][case]
        assert [c[2] for c in t['calls']] == ['logger'], case
        by = 'app' if case in ('empty-action', 'no-action-key') else ''
        assert t['calls'][0][3:] == ['-t', 'mower-host-request', f'action= by={by}'], case
        assert t['update_status'] is None and t['rc'] == 0, case
    assert g2['consumer']['no-file']['calls'] == []
    for case, word in (('unknown-action', 'format-disk'), ('null-action', 'None'), ('number-action', '5')):
        t = g2['consumer'][case]
        assert t['calls'][-1][3:] == ['-t', 'mower-host-request', f"ignoring unknown action '{word}'"], case
        assert t['update_status'] is None, case


def test_g2_known_bugs_are_pinned(g2):
    """Left in place (E6 design D7): the Rust consumer of Phase F fixes them."""
    # status() does not escape a quote in requested_by
    bad = g2['consumer']['quote-in-by-reboot']
    assert bad['update_status_is_json'] is False
    assert bad['calls'][-1][2:] == ['systemctl', 'reboot']
    # a restart that failed reports success, in both ways it can fail
    for case in ('restart-compose-fails', 'restart-no-mower-dir'):
        assert json.loads(g2['consumer'][case]['update_status'])['message'] == 'restarted', case


# ---- G3: update_status / update_check / image.json ---------------------------

DIGEST_A = 'sha256:aaaa1111'
DIGEST_B = 'sha256:bbbb2222'
ID_A = 'sha256:id-a'
ID_B = 'sha256:id-b'
REPO_IMAGE = 'docker.io/fxrbindi/mower_path_planning'
# the robot at rest: the tag matches the registry and the service runs the tag
AT_REST = {'local': DIGEST_A, 'remote': DIGEST_A, 'tag_id': ID_A, 'running_id': ID_A, 'running': True}
NEW_ON_THE_REGISTRY = {**AT_REST, 'remote': DIGEST_B, 'pulled_tag_id': ID_B}


def _status(state, age=3600):
    return json.dumps({'state': state, 'message': 'x', 'tag': 'main', 'time': NOW - age}) + '\n'


G3_CASES = {
    'up-to-date': {},
    'new-image': {'docker': NEW_ON_THE_REGISTRY},
    # the tag matches, the container is on the previous image: one restart
    'pulled-but-not-running-it': {'docker': {**AT_REST, 'running_id': 'sha256:id-old'}},
    'service-not-running': {'docker': {**AT_REST, 'running': False}},
    'force': {'args': ['--force']},
    'registry-unreachable-pull-works': {'docker': {**NEW_ON_THE_REGISTRY, 'manifest_fail': True}},
    'never-pulled': {'docker': {**NEW_ON_THE_REGISTRY, 'local': ''}},
    'pull-denied': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'Error response from daemon: denied'}},
    'pull-unauthorized': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'unauthorized: authentication required'}},
    'pull-no-network': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'dial tcp: lookup r: no such host'}},
    'pull-timeout': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'net/http: TLS handshake timeout'}},
    # the last line of the output, with a quote: this script escapes it
    'pull-other-with-a-quote': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'layer 1\nmanifest "x" unknown'}},
    # a pull the network cut off is tried again, 3 times, 30 s apart; the others are not
    'pull-cut-off-once': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'unexpected EOF', 'pull_fail_times': 1}},
    'pull-cut-off-every-time': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'unexpected EOF'}},
    'pull-no-route': {'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'dial tcp: network is unreachable'}},
    'compose-up-fails': {'docker': {**NEW_ON_THE_REGISTRY, 'compose_up_rc': 1}},
    'interrupted-pulling': {'status_before': _status('pulling')},
    'interrupted-restarting': {'status_before': _status('restarting')},
    'interrupted-rebooting': {'status_before': '{"state":"rebooting","message":"x","time":1}\n'},
    'earlier-failure-is-not-an-interruption': {'status_before': _status('failed')},
    'tag-replaces-the-line': {'args': ['--tag', 'v0.5.0'], 'env_file': 'OTHER=1\nIMAGE_TAG=main\n'},
    'tag-appends-a-line': {'args': ['--tag', 'v0.5.0'], 'env_file': 'OTHER=1\n'},
    'tag-defaults-to-stable': {'env_file': ''},
    'no-env-file-is-created': {'env_file': None},
    'image-repo-from-env': {'env_file': 'IMAGE_REPO=ghcr.io/edwards414/mower_path_planning\nIMAGE_TAG=main\n',
                            'docker': {**NEW_ON_THE_REGISTRY, 'pull_fail': 'denied'}},
    'check-up-to-date': {'args': ['--check']},
    'check-moved': {'args': ['--check'], 'docker': NEW_ON_THE_REGISTRY},
    'check-registry-down': {'args': ['--check'], 'docker': {**AT_REST, 'manifest_fail': True}},
    'check-garbage-answer': {'args': ['--check'], 'docker': {**AT_REST, 'manifest_text': 'not json'}},
    'check-list-answer': {'args': ['--check'], 'docker': {
        **AT_REST, 'manifest_text': json.dumps([{'Descriptor': {'digest': DIGEST_B}}, {'Descriptor': {'digest': 'other'}}])}},
    'check-empty-list-answer': {'args': ['--check'], 'docker': {**AT_REST, 'manifest_text': '[]'}},
    'check-keeps-update-status': {'args': ['--check'], 'status_before': _status('idle')},
    'lock-held': {'lock': True, 'status_before': _status('idle')},
    'lock-held-check': {'lock': True, 'args': ['--check']},
    'unknown-argument': {'args': ['--nope']},
    'no-mower-dir': {'no_mower_dir': True},
}


def run_update(tmp, case):
    state, mower, cwd = tmp / 'state', tmp / 'opt_mower', tmp / 'cwd'
    state.mkdir()
    cwd.mkdir()
    bindir = fake_bin(tmp / 'bin', ('docker', 'sleep'), STATE_DIR_TOOLS + ('mkdir', 'touch', 'grep', 'sed', 'head', 'flock'))
    if not case.get('no_mower_dir'):
        mower.mkdir()
        env_file = case.get('env_file', 'IMAGE_TAG=main\n')
        if env_file is not None:
            (mower / '.env').write_text(env_file)
    if 'status_before' in case:
        (state / 'update_status.json').write_text(case['status_before'])
    if 'robot_status' in case:
        (state / 'robot_status.json').write_text(case['robot_status'])
    log = tmp / 'log'
    spec = {'state': str(state), 'log': str(log), 'pulled': str(tmp / 'pulled'),
            'docker': case.get('docker', AT_REST)}
    (tmp / 'spec.json').write_text(json.dumps(spec))
    env = {'PATH': str(bindir), 'MOWER_DIR': str(mower), 'MOWER_STATE_DIR': str(state), 'FAKE_SPEC': str(tmp / 'spec.json'),
           'MOWER_CPUFREQ_DIR': str(tmp / 'no-cpufreq'), **case.get('env', {})}
    lock = None
    if case.get('lock'):
        lock = open(state / '.update.lock', 'w')
        fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        p = subprocess.run(['/bin/bash', str(UPDATE_SH)] + case.get('args', []), capture_output=True, text=True,
                           timeout=60, env=env, cwd=cwd)
    finally:
        if lock:
            lock.close()
    name = namer((mower, MOWER), (state, STATE), (cwd, '<cwd>'), (UPDATE_SH, f'{MOWER}/host/mower-update.sh'))
    calls = [json.loads(ln) for ln in log.read_text().splitlines()] if log.exists() else []
    files = {}
    for f in ('update_status.json', 'update_check.json', 'image.json'):
        text = read_or_none(state / f)
        files[f] = text
        assert not (state / f'{f}.tmp').exists(), f
    return {
        'given': case,
        'calls': [[c[0]] + [name(a) for a in c[1:]] for c in calls],
        'stdout': lines(name(p.stdout)),
        'stderr': lines(name(p.stderr)),
        'rc': p.returncode,
        **files,
        'env_file_after': read_or_none(mower / '.env'),
    }


def g3_now(tmp):
    return run_table(G3_CASES, run_update, tmp)


@pytest.fixture(scope='module')
def g3(tmp_path_factory):
    return g3_now(tmp_path_factory.mktemp('g3'))


def test_g3_matches_the_golden(g3):
    assert_same(g3, load_golden('g3_update_files.json')['cases'])


def test_g3_the_states_come_in_their_order(g3):
    def states(case):
        return [ln.split(' ')[1].rstrip(':') for ln in g3[case]['stdout'] if ln.startswith('[mower-update] ')
                and ln.split(' ')[1].rstrip(':') in ('pulling', 'restarting', 'idle', 'up_to_date', 'failed')]
    assert states('new-image') == ['pulling', 'restarting', 'idle']
    assert states('up-to-date') == ['up_to_date']
    assert states('pull-denied') == ['pulling', 'failed']
    assert states('compose-up-fails') == ['pulling', 'restarting', 'failed']
    assert states('interrupted-pulling') == ['failed', 'up_to_date']
    # the file says so before docker is asked
    assert [c[0] for c in g3['new-image']['calls'] if c[2:4] == ['docker', 'pull']] == ['pulling']
    up = [c for c in g3['new-image']['calls'] if c[2:5] == ['docker', 'compose', 'up']]
    assert [c[0] for c in up] == ['restarting'] and up[0][1] == MOWER


def test_g3_a_pull_the_network_cut_off_is_tried_again(g3):
    def pulls(case):
        return [c for c in g3[case]['calls'] if c[2:4] == ['docker', 'pull']]

    def naps(case):
        return [c[3:] for c in g3[case]['calls'] if c[2] == 'sleep']
    assert len(pulls('pull-cut-off-once')) == 2 and naps('pull-cut-off-once') == [['30']]
    assert json.loads(g3['pull-cut-off-once']['update_status.json'])['state'] == 'idle'
    assert len(pulls('pull-cut-off-every-time')) == 3 and naps('pull-cut-off-every-time') == [['30'], ['30']]
    assert json.loads(g3['pull-cut-off-every-time']['update_status.json'])['state'] == 'failed'
    assert g3['pull-cut-off-every-time']['rc'] == 1
    for case in ('pull-denied', 'pull-unauthorized', 'pull-other-with-a-quote'):
        assert len(pulls(case)) == 1 and naps(case) == [], case


def test_g3_the_documents_robot_status_reads(g3):
    """Their shapes: robot_status/src/info.rs reads these keys, types included."""
    t = g3['new-image']
    assert json.loads(t['update_status.json']) == {
        'state': 'idle', 'message': f'updated to {REPO_IMAGE}:main ({DIGEST_B})', 'tag': 'main', 'time': NOW}
    assert json.loads(t['image.json']) == {
        'image': f'{REPO_IMAGE}:main', 'tag': 'main', 'digest': DIGEST_B, 'id': ID_B, 'pulled_at': NOW}
    assert json.loads(t['update_check.json']) == {
        'time': NOW, 'tag': 'main', 'image': f'{REPO_IMAGE}:main', 'remote_digest': DIGEST_B,
        'local_digest': DIGEST_B, 'error': ''}
    down = g3['check-registry-down']
    assert down['rc'] == 1 and down['update_status.json'] is None and down['image.json'] is None
    assert json.loads(down['update_check.json']) == {
        'time': NOW, 'tag': 'main', 'image': f'{REPO_IMAGE}:main', 'remote_digest': '',
        'local_digest': DIGEST_A, 'error': 'registry lookup failed (network or docker.io login)'}


def test_g3_this_script_escapes_the_quote_mower_host_request_does_not(g3):
    t = g3['pull-other-with-a-quote']
    assert json.loads(t['update_status.json'])['message'] == f'pull of {REPO_IMAGE}:main failed on attempt 1: manifest "x" unknown'


def test_g3_a_check_never_touches_the_service(g3):
    for case in ('check-up-to-date', 'check-moved', 'check-registry-down', 'check-garbage-answer', 'check-list-answer'):
        for c in g3[case]['calls']:
            assert c[2:4] == ['docker', 'image'] or c[2:4] == ['docker', 'manifest'], (case, c)
    assert g3['check-keeps-update-status']['update_status.json'] == _status('idle')


def test_g3_a_held_lock_changes_nothing(g3):
    for case in ('lock-held', 'lock-held-check'):
        assert g3[case]['calls'] == [] and g3[case]['rc'] == 0, case
        assert g3[case]['update_check.json'] is None and g3[case]['image.json'] is None, case
    assert g3['lock-held']['update_status.json'] == _status('idle')


# ---- G4: link_status.json ----------------------------------------------------

# The modem's replies per command, bytes as latin-1 characters; several strings
# are sent 200 ms apart. A command with no entry gets no answer. `delay_ms` holds
# the answer back, `close_after` unplugs the modem when the next command arrives.
EC25 = {
    'AT+CSQ': ['AT+CSQ\r\r\n+CSQ: 15,99\r\n\r\nOK\r\n'],
    'AT+CREG?': ['AT+CREG?\r\r\n+CREG: 0,1\r\n\r\nOK\r\n'],
    'AT+COPS?': ['AT+COPS?\r\r\n+COPS: 0,0,"Chunghwa",7\r\n\r\nOK\r\n'],
    'AT+QNWINFO': ['AT+QNWINFO\r\r\n+QNWINFO: "FDD LTE","46692","LTE BAND 7",3050\r\n\r\nOK\r\n'],
    'AT+QCSQ': ['AT+QCSQ\r\r\n+QCSQ: "LTE",-60,-90,150,-9\r\n\r\nOK\r\n'],
}
# lubancat's SIM7600G-H (the reply captured on the robot 2026-10-07, test_lte_uplink.py)
SIM7600 = {
    'AT+CSQ': ['AT+CSQ\r\r\n+CSQ: 20,99\r\n\r\nOK\r\n'],
    'AT+CREG?': ['AT+CREG?\r\r\n+CREG: 0,5\r\n\r\nOK\r\n'],
    'AT+COPS?': ['AT+COPS?\r\r\n+COPS: 0,2,"46692",7\r\n\r\nOK\r\n'],
    'AT+QNWINFO': ['AT+QNWINFO\r\r\nERROR\r\n'],
    'AT+QCSQ': ['AT+QCSQ\r\r\nERROR\r\n'],
    'AT+CPSI?': ['AT+CPSI?\r\r\n+CPSI: LTE,Online,466-92,0x639D,134534710,430,EUTRAN-BAND7,3050,5,5,-103,-998,-722,13\r\n\r\nOK\r\n'],
}
ALL_ERROR = {c: ['ERROR\r\n'] for c in ('AT+CSQ', 'AT+CREG?', 'AT+COPS?', 'AT+QNWINFO', 'AT+QCSQ', 'AT+CPSI?')}
WIRELESS_HEADER = ('Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n'
                   ' face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n')
WIRELESS = WIRELESS_HEADER + ' wlan0: 0000   49.  -61.  -256        0      0      0      0     12        0\n'


def _addr(ifname, operstate, local=None, family='inet'):
    """One interface of `ip -j -4 addr` (iproute2 6.1, the shape recorded in a container)."""
    info = [{'family': family, 'local': local, 'prefixlen': 24, 'scope': 'global', 'label': ifname,
             'valid_life_time': 4294967295, 'preferred_life_time': 4294967295}] if local else []
    return {'ifindex': 2, 'ifname': ifname, 'flags': ['BROADCAST', 'UP'], 'mtu': 1500, 'qdisc': 'mq',
            'operstate': operstate, 'group': 'default', 'addr_info': info}


IP_ROBOT = json.dumps([_addr('lo', 'UNKNOWN', '127.0.0.1'), _addr('eth0', 'UP', '192.168.0.113'),
                       _addr('wwan0', 'UNKNOWN', '10.64.1.2'), _addr('tailscale0', 'UNKNOWN', '100.64.0.7'),
                       _addr('docker0', 'DOWN', '172.17.0.1'), _addr('br-1234', 'UP', '172.18.0.1'),
                       _addr('veth99', 'UP', '169.254.0.1'), _addr('dummy0', 'DOWN', '10.0.0.1')])

G4_CASES = {
    # the modems
    'ec25': {'modem': EC25, 'ip': {'stdout': IP_ROBOT}, 'wireless': WIRELESS},
    'sim7600': {'modem': SIM7600, 'ip': {'stdout': IP_ROBOT}, 'wireless': WIRELESS_HEADER},
    'no-modem': {'ip': {'stdout': IP_ROBOT}, 'wireless': WIRELESS_HEADER},
    # a directory opens with EISDIR whoever runs the test; the message names the port
    'cannot-open': {'at_port': 'directory'},
    # termios.error is not an OSError: nothing is written, the old file stays
    'not-a-tty': {'at_port': 'regular-file'},
    'every-command-errors': {'modem': ALL_ERROR},
    'silent-modem': {'modem': {}},
    'csq-unknown': {'modem': {**EC25, 'AT+CSQ': ['+CSQ: 99,99\r\n\r\nOK\r\n'], 'AT+CREG?': ['+CREG: 0,2\r\n\r\nOK\r\n']}},
    'csq-bounds-0': {'modem': {**EC25, 'AT+CSQ': ['+CSQ: 0,0\r\n\r\nOK\r\n']}},
    'csq-bounds-31': {'modem': {**EC25, 'AT+CSQ': ['+CSQ: 31,7\r\n\r\nOK\r\n']}},
    'csq-leading-zeros': {'modem': {**EC25, 'AT+CSQ': ['+CSQ:007,099\r\n\r\nOK\r\n']}},
    'creg-roaming-long-form': {'modem': {**EC25, 'AT+CREG?': ['+CREG: 2,5,"1A2B","00C3",7\r\n\r\nOK\r\n']}},
    'creg-searching': {'modem': {**EC25, 'AT+CREG?': ['+CREG: 0,2\r\n\r\nOK\r\n']}},
    'creg-one-number': {'modem': {**EC25, 'AT+CREG?': ['+CREG: 1\r\n\r\nOK\r\n']}},
    'cops-no-tech': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"Far EasTone"\r\n\r\nOK\r\n']}},
    'cops-gsm': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"TWM",0\r\n\r\nOK\r\n']}},
    'cops-utran': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"TWM",2\r\n\r\nOK\r\n']}},
    'cops-unknown-tech': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"TWM",13\r\n\r\nOK\r\n']}},
    'cops-not-registered': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0\r\n\r\nOK\r\n']}},
    'cops-empty-name': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"",7\r\n\r\nOK\r\n']}},
    # int from +QCSQ, float from +CPSI: part of the contract
    'qcsq-sinr-floor': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "LTE",-60,-90,0,-9\r\n\r\nOK\r\n']}},
    'qcsq-sinr-one-step': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "LTE",-60,-90,1,-9\r\n\r\nOK\r\n']}},
    'qcsq-sinr-top': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "LTE",-60,-140,250,-20\r\n\r\nOK\r\n']}},
    'qcsq-sinr-negative': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "LTE",-60,0,-7,0\r\n\r\nOK\r\n']}},
    # 61/5.0-20.0 is -7.800000000000001 before round()
    'qcsq-sinr-needs-the-rounding': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "LTE",-60,-90,61,-9\r\n\r\nOK\r\n']}},
    'qcsq-no-service-falls-back-to-cpsi': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "NOSERVICE"\r\n\r\nOK\r\n'],
                                                     'AT+CPSI?': SIM7600['AT+CPSI?']}},
    'qcsq-not-lte-falls-back-to-cpsi': {'modem': {**EC25, 'AT+QCSQ': ['+QCSQ: "WCDMA",-70,-100,100\r\n\r\nOK\r\n'],
                                                  'AT+CPSI?': ['ERROR\r\n']}},
    # +CPSI? overwrites what +QNWINFO said
    'cpsi-overrides-qnwinfo': {'modem': {**SIM7600, 'AT+QNWINFO': EC25['AT+QNWINFO']}},
    'cpsi-no-service': {'modem': {**SIM7600, 'AT+CPSI?': ['+CPSI: NO SERVICE,Online\r\n\r\nOK\r\n']}},
    'cpsi-wcdma': {'modem': {**SIM7600, 'AT+CPSI?': ['+CPSI: WCDMA,Online,466-92,0x639D,1,WCDMA 2100,1,2,3,4,5\r\n\r\nOK\r\n']}},
    'cpsi-rounding-and-negative-zero': {'modem': {**SIM7600, 'AT+CPSI?': [
        '+CPSI: LTE,Online,466-92,0x1,1,1,EUTRAN-BAND3,1,5,5,-1,-1,-1,-0\r\n\r\nOK\r\n']}},
    # the bytes the modem sends: decoded with errors='replace', then \uXXXX in the file
    'operator-not-utf8': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"Chung\xffhwa \xe4\xb8\xad\xe8",7\r\n\r\nOK\r\n']}},
    'operator-utf8': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"\xe4\xb8\xad\xe8\x8f\xaf\xf0\x9f\x8c\xb1",7\r\n\r\nOK\r\n']}},
    # the answer in two reads
    'answer-in-two-pieces': {'modem': {**EC25, 'AT+CSQ': ['+CSQ: 15,99\r\n', '\r\nOK\r\n']}},
    # at() stops at the bytes OK anywhere: the rest of the answer never gets read
    'operator-name-holds-OK': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"OK', 'TEL",7\r\n\r\nOK\r\n']}},
    'operator-name-holds-OK-in-one-read': {'modem': {**EC25, 'AT+COPS?': ['+COPS: 0,0,"OKTEL",7\r\n\r\nOK\r\n']}},
    # no resync: a late answer lands in the next command's window and ends it
    'late-answer': {'modem': {k: v for k, v in EC25.items() if k != 'AT+CREG?'}, 'delay_ms': {'AT+CSQ': 1700}},
    # an unsolicited line in front of an answer
    'urc-in-front': {'modem': {**EC25, 'AT+CSQ': ['\r\n+QIND: "csq",1,2\r\n+CSQ: 15,99\r\n\r\nOK\r\n']}},
    # unplugged: the next write fails with EIO, what was read so far stays
    'unplugged-after-csq': {'modem': EC25, 'close_after': 'AT+CSQ'},
    # /proc/net/wireless
    'wifi-no-file': {'wireless': None},
    'wifi-header-only': {'wireless': WIRELESS_HEADER},
    'wifi-two': {'wireless': WIRELESS + ' wlan1: 0000   0.  -256.  -256        0      0      0      0      0        0\n'},
    'wifi-whole-numbers': {'wireless': WIRELESS_HEADER + ' wlan0: 0000   49  -61  -256 0 0 0 0 12 0\n'},
    'wifi-short-row-skipped': {'wireless': WIRELESS_HEADER + ' wlan0: 0000 49.\n wlan1: 0000 7. -9.\n'},
    # float() raises and only OSError is caught: nothing is written
    'wifi-bad-number': {'wireless': WIRELESS_HEADER + ' wlan0: 0000 x. -9.\n'},
    # `ip -j -4 addr`
    'ip-missing': {'ip': None},
    'ip-fails': {'ip': {'stdout': '', 'rc': 1}},
    'ip-bad-json': {'ip': {'stdout': 'not json'}},
    'ip-not-utf8': {'ip': {'stdout': '[{"ifname":"\xff"}]', 'latin1': True}},
    'ip-blank-output': {'ip': {'stdout': '  \n'}},
    'ip-hangs': {'ip': {'hang': True}},
    # the interfaces: no IPv4, and the {} older iproute2 prints for those
    'ip-no-ipv4': {'ip': {'stdout': json.dumps([_addr('wlan0', 'DOWN'), _addr('eth0', 'UP', '192.168.0.113')])}},
    'ip-empty-object': {'ip': {'stdout': json.dumps([{}, _addr('eth0', 'UP', '192.168.0.113'), {}])}},
    'ip-states': {'ip': {'stdout': json.dumps([_addr('a0', 'UP', '1.1.1.1'), _addr('a1', 'UNKNOWN'), _addr('a2', 'DOWN'),
                                               _addr('a3', 'LOWERLAYERDOWN'), {'ifname': 'a4'}])}},
    'ip-prefix-skips': {'ip': {'stdout': json.dumps([_addr(n, 'UP') for n in (
        'lo', 'loop1', 'docker0', 'dockerfoo', 'veth1', 'vethernet', 'br-1', 'br0', 'dummy0', 'dummy', 'eth0')])}},
    'ip-two-addresses': {'ip': {'stdout': json.dumps([{**_addr('eth0', 'UP'), 'addr_info': [
        {'family': 'inet6', 'local': 'fe80::1'}, {'family': 'inet', 'local': '10.0.0.1'}, {'family': 'inet', 'local': '10.0.0.2'}]}])}},
    'ip-address-without-local': {'ip': {'stdout': json.dumps([{**_addr('eth0', 'UP'), 'addr_info': [{'family': 'inet'}]}])}},
    'ip-empty-list': {'ip': {'stdout': '[]'}},
    'ip-empty-dict': {'ip': {'stdout': '{}'}},
    # not what ip prints: these raise past ifaces()'s except, nothing is written
    'ip-items-not-objects': {'ip': {'stdout': '[1]'}},
    'ip-dict-with-keys': {'ip': {'stdout': '{"a": 1}'}},
    'ip-null': {'ip': {'stdout': 'null'}},
    'ip-ifname-not-a-string': {'ip': {'stdout': '[{"ifname": 5}]'}},
    'ip-addr-info-null': {'ip': {'stdout': '[{"ifname": "eth0", "addr_info": null}]'}},
    # the state dir
    'state-dir-missing': {'no_state_dir': True},
    'state-dir-with-a-slash': {'state_arg_suffix': '/'},
    # names that are links: followed by nothing, in the state dir or at its own name
    'tmp-is-a-link': {'link_at': 'link_status.json.tmp'},
    'file-is-a-link': {'link_at': 'link_status.json'},
    'state-dir-is-a-link': {'state_is_link': True},
    'state-dir-with-a-slash-is-a-link': {'state_is_link': True, 'state_arg_suffix': '/'},
    'left-over-tmp': {'tmp_before': 'left over'},
    'argument-order': {'argv': ['--once', '<state>', 'ignored-extra', '--ignored']},
    'bad-period-fails-at-the-start': {'env': {'MOWER_LINK_PERIOD_S': 'soon'}},
    'bad-period-even-if-empty': {'env': {'MOWER_LINK_PERIOD_S': ''}},
    'period-is-only-for-the-loop': {'env': {'MOWER_LINK_PERIOD_S': '0.5'}},
    'at-port-env-empty-is-no-modem': {'env': {'MOWER_LTE_AT': ''}},
}

STALE_LINK = '{"stale": true}'
# what a link in the state dir leads to, in G4 and G5
OTHER_FILE = '{"robot_id": "MW-OTHER", "secret": "OTHER", "stale": false}\n'


class FakeModem(threading.Thread):
    """The modem side of a pty, for the sampler's AT port."""

    def __init__(self, script, delay_ms=None, close_after=None):
        super().__init__(daemon=True)
        self.script = script
        self.delay_ms = delay_ms or {}
        self.close_after = close_after
        self.commands = []
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.done = threading.Event()

    def run(self):
        buf = b''
        due = []
        while not self.done.is_set():
            now = time.monotonic()
            for item in [d for d in due if d[0] <= now]:
                due.remove(item)
                os.write(self.master, item[1])
            r, _, _ = select.select([self.master], [], [], 0.005)
            if not r:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if not data:
                return
            buf += data
            while b'\r\n' in buf:
                cmd, buf = buf.split(b'\r\n', 1)
                cmd = cmd.decode('latin-1')
                unplug = self.close_after in self.commands
                self.commands.append(cmd)
                if unplug:
                    os.close(self.master)
                    self.master = None
                    return
                at = time.monotonic() + self.delay_ms.get(cmd, 0) / 1000.0
                for i, piece in enumerate(self.script.get(cmd, [])):
                    due.append((at + 0.2 * i, piece.encode('latin-1')))

    def stop(self):
        self.done.set()
        self.join(timeout=5)
        for fd in (self.master, self.slave):
            if fd is not None:
                os.close(fd)


# Runs the sampler's own --once, with /proc/net/wireless and the clock replaced
# and its chown recorded. Python's line in `time` is .5: int(), not round().
LINK_DRIVER = r'''
import builtins, json, os, runpy, sys, time
cfg = json.loads(os.environ['DRIVER_CFG'])
real_open = builtins.open
def fake_open(file, *a, **k):
    if file == '/proc/net/wireless':
        file = cfg['wireless']
    return real_open(file, *a, **k)
builtins.open = fake_open
time.time = lambda: cfg['now'] + 0.5
real_chown, real_fchown = os.chown, os.fchown
def chown(path, uid, gid):
    with real_open(cfg['chown_log'], 'a') as f:
        f.write(json.dumps([path, uid, gid]) + '\n')
    real_chown(path, uid, gid)
def fchown(fd, uid, gid):
    with real_open(cfg['chown_log'], 'a') as f:
        f.write(json.dumps(['fd', uid, gid]) + '\n')
    real_fchown(fd, uid, gid)
os.chown, os.fchown = chown, fchown
sys.argv = ['mower-link-status.py'] + cfg['argv']
runpy.run_path(cfg['script'], run_name='__main__')
'''


def exception_of(stderr):
    """The exception type out of a traceback's last line."""
    last = stderr.strip().splitlines()[-1] if stderr.strip() else ''
    return last.split(':', 1)[0] if 'Traceback' in stderr else None


def run_link(tmp, case):
    """One case. The unplugged modem has at() spin until its deadline, where its
    check and its select race: now and then the tool dies of select's ValueError
    on a negative timeout and loses the round (test_g4_at_can_lose_a_round_at_its_
    deadline holds that). The golden holds what it does otherwise."""
    if not case.get('close_after'):
        return run_link_once(tmp, case)
    for attempt in range(8):
        d = tmp / f'try{attempt}'
        d.mkdir()
        result = run_link_once(d, case)
        if result['raises'] != 'ValueError':
            break
    return result


def run_link_once(tmp, case):
    state, cwd = tmp / 'state', tmp / 'cwd'
    cwd.mkdir()
    other = tmp / 'other-file'
    other.write_text(OTHER_FILE)
    elsewhere = tmp / 'elsewhere'
    if case.get('state_is_link'):
        elsewhere.mkdir()
        state.symlink_to(elsewhere)
    elif not case.get('no_state_dir'):
        state.mkdir()
        (state / 'link_status.json').write_text(STALE_LINK)
    if case.get('link_at'):
        (state / case['link_at']).unlink(missing_ok=True)
        (state / case['link_at']).symlink_to(other)
    if case.get('tmp_before'):
        (state / 'link_status.json.tmp').write_text(case['tmp_before'])
    bindir = tmp / 'bin'
    bindir.mkdir()
    ip = case.get('ip', {'stdout': '[]'})
    if ip is not None:
        tool = bindir / 'ip'
        if ip.get('hang'):
            tool.write_text('#!/bin/sh\nexec /bin/sleep 30\n')
        else:
            data = ip['stdout'].encode('latin-1' if ip.get('latin1') else 'utf-8')
            (tmp / 'ip.out').write_bytes(data)
            tool.write_text(f'#!/bin/sh\n[ "$*" = "-j -4 addr" ] || exit 99\n/bin/cat {shlex.quote(str(tmp / "ip.out"))}\n'
                            f'exit {ip.get("rc", 0)}\n')
        tool.chmod(0o755)
    wireless = tmp / 'wireless'
    if case.get('wireless') is not None:
        wireless.write_text(case['wireless'])
    modem = None
    at_port = str(tmp / 'no-modem')
    if case.get('at_port') == 'directory':
        at_port = str(tmp / 'portdir')
        Path(at_port).mkdir()
    elif case.get('at_port') == 'regular-file':
        at_port = str(tmp / 'portfile')
        Path(at_port).write_text('')
    elif 'modem' in case:
        modem = FakeModem(case['modem'], case.get('delay_ms'), case.get('close_after'))
        at_port = modem.path
        modem.start()
    state_arg = str(state) + case.get('state_arg_suffix', '')
    argv = [state_arg if a == '<state>' else a for a in case.get('argv', ['<state>', '--once'])]
    chown_log = tmp / 'chowns'
    cfg = {'now': NOW, 'wireless': str(wireless), 'argv': argv, 'script': str(LINK_PY), 'chown_log': str(chown_log)}
    (tmp / 'driver.py').write_text(LINK_DRIVER)
    env = {'PATH': str(bindir), 'DRIVER_CFG': json.dumps(cfg), 'MOWER_LTE_AT': at_port, 'PYTHONUTF8': '1',
           'PYTHONDONTWRITEBYTECODE': '1', **case.get('env', {})}
    try:
        p = subprocess.run([sys.executable, str(tmp / 'driver.py')], capture_output=True, text=True, timeout=60,
                           env=env, cwd=cwd)
    finally:
        if modem:
            modem.stop()
    name = namer((at_port, AT_PORT), (state, STATE))
    target = state / 'link_status.json'
    text = read_or_none(target)
    chowns = []
    if chown_log.exists():
        st = state.stat()
        for ln in chown_log.read_text().splitlines():
            path, uid, gid = json.loads(ln)
            chowns.append([name(path), [uid, gid] == [st.st_uid, st.st_gid]])
    return {
        'given': case,
        'commands': modem.commands if modem else [],
        'rc': p.returncode,
        'stdout': name(p.stdout),
        # the errno of a link at the state dir's name differs between systems, the raising does not
        'raises': 'OSError' if case.get('state_is_link') and p.returncode else exception_of(p.stderr),
        'file': name(text) if text is not None else None,
        'other_untouched': other.read_text() == OTHER_FILE,
        'elsewhere_empty': not any(elsewhere.iterdir()) if elsewhere.is_dir() else None,
        'mode': stat.S_IMODE(target.stat().st_mode) if text is not None and text != STALE_LINK else None,
        'chown_to_the_dir_owner': chowns,
        'tmp_left': (state / 'link_status.json.tmp').exists(),
    }


def g4_now(tmp):
    return run_table(G4_CASES, run_link, tmp)


@pytest.fixture(scope='module')
def g4(tmp_path_factory):
    return g4_now(tmp_path_factory.mktemp('g4'))


def test_g4_matches_the_golden(g4):
    assert_same(g4, load_golden('g4_link_status.json')['cases'])


def test_g4_defaults_are_the_goldens():
    assert load_golden('g4_link_status.json')['defaults'] == link_defaults()


def _link_tool():
    spec = importlib.util.spec_from_file_location('mower_link_status_under_test', LINK_PY)
    mod = importlib.util.module_from_spec(spec)
    with mock.patch.dict(os.environ, clear=True):
        spec.loader.exec_module(mod)
    return mod


def test_g4_at_can_lose_a_round_at_its_deadline():
    """The bug in place: the deadline passes between at()'s check and its select. ValueError is
    not an OSError, so the whole sample is lost, and the old file stays until the next round."""
    mod = _link_tool()
    master, slave = os.openpty()
    try:
        with mock.patch.object(mod.time, 'monotonic', side_effect=[0.0, 1.0, 2.0]):
            with pytest.raises(ValueError):
                mod.at(slave, 'AT+CSQ')
    finally:
        os.close(master)
        os.close(slave)


def link_defaults():
    """What the tool assumes when nothing is set: read from its own module."""
    mod = _link_tool()
    return {'at_port': mod.AT_PORT, 'period_s': mod.PERIOD_S, 'skip_ifaces': list(mod.SKIP_IFACES),
            'at_timeout_s': mod.at.__defaults__[0], 'state_dir': STATE}


def _lte(g4, case):
    return json.loads(g4[case]['file'])['lte']


def test_g4_the_numbers_keep_their_python_types(g4):
    """rsrp and rsrq are ints from +QCSQ and floats from +CPSI: telemetry's reader sees that."""
    q = _lte(g4, 'ec25')
    assert (q['rsrp_dbm'], q['rsrq_db'], q['sinr_db']) == (-90, -9, 10.0)
    assert type(q['rsrp_dbm']) is int and type(q['rsrq_db']) is int and type(q['sinr_db']) is float
    c = _lte(g4, 'sim7600')
    assert (c['rsrp_dbm'], c['rsrq_db'], c['sinr_db']) == (-99.8, -10.3, 13.0)
    assert type(c['rsrp_dbm']) is float and type(c['rsrq_db']) is float and type(c['sinr_db']) is float
    assert type(q['rssi_dbm']) is int and q['rssi_dbm'] == -83 and q['csq'] == 15 and q['ber'] == 99
    w = json.loads(g4['ec25']['file'])['wifi']
    assert w == [{'iface': 'wlan0', 'link': 49.0, 'rssi_dbm': -61.0}] and type(w[0]['link']) is float
    assert type(json.loads(g4['ec25']['file'])['time']) is int and json.loads(g4['ec25']['file'])['time'] == NOW


def test_g4_quectel_never_gets_asked_the_simcom_question(g4):
    assert g4['ec25']['commands'] == ['AT+CSQ', 'AT+CREG?', 'AT+COPS?', 'AT+QNWINFO', 'AT+QCSQ']
    assert g4['sim7600']['commands'] == ['AT+CSQ', 'AT+CREG?', 'AT+COPS?', 'AT+QNWINFO', 'AT+QCSQ', 'AT+CPSI?']


def test_g4_the_shapes_without_a_modem(g4):
    assert _lte(g4, 'no-modem') == {'present': False}
    bad = _lte(g4, 'cannot-open')
    assert bad['present'] is True and bad['error'] == f"[Errno 21] Is a directory: '{AT_PORT}'"
    assert bad['csq'] is None and list(bad)[:2] == ['present', 'rssi_dbm']
    assert _lte(g4, 'at-port-env-empty-is-no-modem') == {'present': False}


def test_g4_what_raises_leaves_the_old_file(g4):
    for case, exc in (('not-a-tty', 'termios.error'), ('wifi-bad-number', 'ValueError'),
                      ('ip-items-not-objects', 'AttributeError'), ('ip-dict-with-keys', 'AttributeError'),
                      ('ip-null', 'TypeError'), ('ip-ifname-not-a-string', 'AttributeError'),
                      ('ip-addr-info-null', 'TypeError'), ('bad-period-fails-at-the-start', 'ValueError'),
                      ('bad-period-even-if-empty', 'ValueError')):
        t = g4[case]
        assert t['rc'] == 1 and t['raises'] == exc, case
        assert t['file'] == STALE_LINK and t['stdout'] == '' and not t['tmp_left'], case
    t = g4['state-dir-missing']
    assert t['rc'] == 1 and t['raises'] == 'FileNotFoundError' and t['file'] is None


def test_g4_a_written_file_is_replaced_and_given_to_the_dirs_owner(g4):
    t = g4['ec25']
    assert t['mode'] == 0o644 and not t['tmp_left'] and t['file'] != STALE_LINK
    assert t['chown_to_the_dir_owner'] == [['fd', True]], 'on the open file'
    assert json.loads(t['stdout']) == json.loads(t['file'])
    # compact in the file, indent=1 from --once
    assert t['file'].startswith('{"time": ') and t['stdout'].startswith('{\n "time": ')
    assert '\n' not in t['file'] and t['stdout'].endswith('}\n')


def test_g4_no_link_is_followed(g4):
    for case in ('tmp-is-a-link', 'file-is-a-link'):
        t = g4[case]
        assert t['rc'] == 0 and t['other_untouched'], case
        assert json.loads(t['file'])['lte'] == {'present': False} and not t['tmp_left'], case
    for case in ('state-dir-is-a-link', 'state-dir-with-a-slash-is-a-link'):
        t = g4[case]
        assert t['rc'] == 1 and t['elsewhere_empty'], case
    assert g4['left-over-tmp']['rc'] == 0 and not g4['left-over-tmp']['tmp_left']


def test_g4_interfaces(g4):
    ifs = json.loads(g4['ec25']['file'])['ifaces']
    assert ifs == [{'name': 'eth0', 'up': True, 'addr': '192.168.0.113'},
                   {'name': 'wwan0', 'up': True, 'addr': '10.64.1.2'},
                   {'name': 'tailscale0', 'up': True, 'addr': '100.64.0.7'}]
    assert json.loads(g4['ip-no-ipv4']['file'])['ifaces'][0] == {'name': 'wlan0', 'up': False, 'addr': None}
    # older iproute2's {} comes out as a nameless interface
    assert json.loads(g4['ip-empty-object']['file'])['ifaces'][0] == {'name': '', 'up': False, 'addr': None}
    for case in ('ip-missing', 'ip-fails', 'ip-bad-json', 'ip-not-utf8', 'ip-blank-output', 'ip-hangs',
                 'ip-empty-list', 'ip-empty-dict'):
        assert json.loads(g4[case]['file'])['ifaces'] == [], case


def test_g4_known_quirks_are_pinned(g4):
    # the bytes OK inside an operator name split across two reads
    t = _lte(g4, 'operator-name-holds-OK')
    assert t['operator'] is None and _lte(g4, 'operator-name-holds-OK-in-one-read')['operator'] == 'OKTEL'
    # a late answer eats the next command's window; csq stays unknown
    t = _lte(g4, 'late-answer')
    assert t['csq'] is None and t['registered'] is None and t['operator'] == 'Chunghwa'
    # unplugged: what was read stays, the error is the write's
    t = _lte(g4, 'unplugged-after-csq')
    assert t['csq'] == 15 and t['registered'] is None and t['error'] == '[Errno 5] Input/output error'
    # a negative zero stays one
    assert _lte(g4, 'cpsi-rounding-and-negative-zero')['sinr_db'] == 0.0
    assert '"sinr_db": -0.0' in g4['cpsi-rounding-and-negative-zero']['file']
    # errors='replace', then ensure_ascii
    assert '"operator": "Chung\\ufffdhwa \\u4e2d\\ufffd"' in g4['operator-not-utf8']['file']


def test_g4_sinr_and_rounding_sweeps_are_the_goldens():
    assert load_golden('g4_link_status.json')['sweeps'] == link_sweeps()


def link_sweeps():
    """Over more than a modem can say, by digest, Rust repeating them through its parsers: parse_cpsi, and
    the sum in lte() that is not a function of its own."""
    mod = _link_tool()

    def digest(values):
        return hashlib.sha256('\n'.join(repr(v) for v in values).encode()).hexdigest()
    qcsq = [round(i / 5.0 - 20.0, 1) for i in range(-2000, 2001)]
    cpsi = []
    for i in range(-2000, 2001):
        c = mod.parse_cpsi(f'+CPSI: LTE,Online,466-92,0x1,1,1,B,1,5,5,{i},{i},-1,{i}\r\n')
        cpsi += [c['rsrq_db'], c['rsrp_dbm'], c['sinr_db']]
    return {'qcsq_sinr_-2000..=2000': digest(qcsq), 'cpsi_rsrq_rsrp_sinr_-2000..=2000': digest(cpsi)}


# ---- G5: identity.json and the pairing URL ------------------------------------

MACHINE_ID = '9041cf96a09a4c2e964969d4befc76b3'
# 20 "random" bytes per secret, 8 for the seed without a machine-id
R1, R2 = '00' * 19 + '01', '11' * 19 + '22'
R8 = 'a5' * 8
HAVE = {
    'robot_id': 'MW-7K3Q9P', 'name': 'bench', 'secret': 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA', 'created': 1789000000,
    'device_key': 'dk-1234',
}
HAVE_TEXT = json.dumps(HAVE, indent=2) + '\n'
RELAY = 'wss://relay.mower.fxrbindi.com/r'
ENV = f'# the robot\nIMAGE_TAG=main\nPAIR_RELAY_URL={RELAY}\n'


def _have(**more):
    return json.dumps({**HAVE, **more}, indent=2) + '\n'


# argv: what follows `mower-pair --state-dir S --env E`, unless `raw_argv`.
# identity: the file's bytes as latin-1 characters (None: no file). defaults:
# hostname lubancat, machine-id MACHINE_ID, lan 192.168.0.113, user cat 1000:1000.
G5_CASES = {
    # a new identity
    'new': {'argv': ['--no-qr']},
    'new-says-qrencode-is-missing': {'argv': []},
    'new-with-qrencode': {'argv': [], 'qrencode': True},
    'new-makes-the-state-dir': {'argv': ['--no-qr'], 'no_state_dir': True},
    'new-json': {'argv': ['--json']},
    'new-rotate-draws-twice': {'argv': ['--rotate', '--no-qr'], 'random': [R1, R2]},
    'new-with-name': {'argv': ['--name', '前院', '--no-qr']},
    'new-hostname-in-unicode': {'argv': ['--no-qr'], 'hostname': 'lubancat-前院'},
    'machine-id-second-path': {'argv': ['--no-qr'], 'machine_ids': {'/var/lib/dbus/machine-id': 'dbus-machine-id\n'}},
    'machine-id-empty-goes-on': {'argv': ['--no-qr'], 'machine_ids': {'/etc/machine-id': '\n', '/var/lib/dbus/machine-id': 'x'}},
    'machine-id-stripped': {'argv': ['--no-qr'], 'machine_ids': {'/etc/machine-id': f'  {MACHINE_ID} \r\n'}},
    'machine-id-none-is-hostname-and-random': {'argv': ['--no-qr'], 'machine_ids': {}, 'random': [R8, R1]},
    'machine-id-not-utf8-fails': {'argv': ['--no-qr'], 'machine_ids': {'/etc/machine-id': '\xff\n'}},
    # an identity that is there
    'keep': {'argv': ['--no-qr'], 'identity': HAVE_TEXT},
    'keep-leaves-a-wrong-mode-alone': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'identity_mode': 0o644},
    'keep-says-qrencode-is-missing': {'argv': [], 'identity': HAVE_TEXT},
    'keep-with-qrencode': {'argv': [], 'identity': HAVE_TEXT, 'qrencode': True},
    'keep-json': {'argv': ['--json'], 'identity': HAVE_TEXT},
    'keep-json-with-qrencode': {'argv': ['--json'], 'identity': HAVE_TEXT, 'qrencode': True},
    'keep-no-name': {'argv': ['--no-qr'], 'identity': json.dumps({'robot_id': 'MW-1', 'secret': 'S'})},
    'keep-null-name': {'argv': ['--no-qr'], 'identity': json.dumps({'robot_id': 'MW-1', 'secret': 'S', 'name': None})},
    'keep-empty-name': {'argv': ['--no-qr'], 'identity': json.dumps({'robot_id': 'MW-1', 'secret': 'S', 'name': ''})},
    'keep-odd-types': {'argv': ['--no-qr'], 'identity': json.dumps({'robot_id': 5, 'secret': True, 'name': [1, 'x', None, 2.5]})},
    'keep-odd-types-json': {'argv': ['--json'], 'identity': json.dumps({'robot_id': 5, 'secret': 1.5, 'name': {'a': 'b'}})},
    'keep-key-order-and-extras': {'argv': ['--json'], 'identity': json.dumps(
        {'zz': 1, 'secret': 'S', 'relay': 'old', 'robot_id': 'MW-1', 'url': 'old', 'lan': 'old', 'a': {'b': [1.0, None]}})},
    # rotate keeps what the agent added, in its place
    'rotate': {'argv': ['--rotate', '--no-qr'], 'identity': HAVE_TEXT},
    'rotate-again-replaces-rotated': {'argv': ['--rotate', '--no-qr'], 'identity': _have(rotated=1789500000, more=1)},
    'rotate-with-qrencode': {'argv': ['--rotate'], 'identity': HAVE_TEXT, 'qrencode': True},
    'rotate-json': {'argv': ['--rotate', '--json'], 'identity': HAVE_TEXT},
    'rotate-fixes-the-mode': {'argv': ['--rotate', '--no-qr'], 'identity': HAVE_TEXT, 'identity_mode': 0o644},
    # names, the URL's and the file's escaping
    'name-space': {'argv': ['--name', 'front yard', '--no-qr'], 'identity': HAVE_TEXT},
    'name-tilde': {'argv': ['--name', 'a~b', '--no-qr'], 'identity': HAVE_TEXT},
    'name-star': {'argv': ['--name', 'a*b', '--no-qr'], 'identity': HAVE_TEXT},
    'name-chinese': {'argv': ['--name', '前院', '--no-qr'], 'identity': HAVE_TEXT},
    'name-chinese-json': {'argv': ['--name', '前院', '--json'], 'identity': HAVE_TEXT},
    'name-ampersand': {'argv': ['--name', 'a&b=c', '--no-qr'], 'identity': HAVE_TEXT},
    'name-reserved': {'argv': ['--name', "a+b%c/d?e#f:g;h,i@j'k", '--no-qr'], 'identity': HAVE_TEXT},
    'name-safe-set': {'argv': ['--name', 'aZ09_.-~', '--no-qr'], 'identity': HAVE_TEXT},
    'name-quote-backslash': {'argv': ['--name', 'a"b\\c', '--no-qr'], 'identity': HAVE_TEXT},
    'name-quote-backslash-json': {'argv': ['--name', 'a"b\\c', '--json'], 'identity': HAVE_TEXT},
    'name-emoji-json': {'argv': ['--name', '🌱', '--json'], 'identity': HAVE_TEXT},
    'name-control-chars': {'argv': ['--name', 'a\tb\nc\x7f', '--json'], 'identity': HAVE_TEXT},
    'name-empty-changes-nothing': {'argv': ['--name', '', '--no-qr'], 'identity': HAVE_TEXT},
    'name-says-new-secret': {'argv': ['--name', 'x', '--no-qr'], 'identity': HAVE_TEXT},
    'name-and-rotate': {'argv': ['--rotate', '--name', 'x', '--no-qr'], 'identity': HAVE_TEXT},
    # argparse
    'arg-equals': {'argv': ['--name=a=b', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-equals-empty': {'argv': ['--name=', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-equals-dash': {'argv': ['--name=--rotate', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-abbreviated': {'argv': ['--rot', '--na', 'x', '--no-'], 'identity': HAVE_TEXT},
    'arg-abbreviated-equals': {'argv': ['--na=x', '--no'], 'identity': HAVE_TEXT},
    'arg-ambiguous': {'argv': ['--n', 'x'], 'identity': HAVE_TEXT},
    'arg-unknown': {'argv': ['--bogus'], 'identity': HAVE_TEXT},
    'arg-positional': {'argv': ['x'], 'identity': HAVE_TEXT},
    'arg-name-without-value': {'argv': ['--name'], 'identity': HAVE_TEXT},
    'arg-name-then-option': {'argv': ['--name', '--rotate'], 'identity': HAVE_TEXT},
    'arg-name-dash': {'argv': ['--name', '-', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-name-negative-number': {'argv': ['--name', '-5', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-name-negative-decimal': {'argv': ['--name', '-.5', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-name-dash-word': {'argv': ['--name', '-x'], 'identity': HAVE_TEXT},
    'arg-name-dash-with-space': {'argv': ['--name', '-x y', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-flag-with-value': {'argv': ['--rotate=1'], 'identity': HAVE_TEXT},
    'arg-last-wins': {'argv': ['--name', 'a', '--name', 'b', '--no-qr'], 'identity': HAVE_TEXT},
    'arg-help': {'argv': ['--help'], 'identity': HAVE_TEXT},
    'arg-help-short': {'argv': ['-h'], 'identity': HAVE_TEXT},
    'arg-help-abbreviated': {'argv': ['--he'], 'identity': HAVE_TEXT},
    'arg-help-before-a-bad-flag': {'argv': ['-h', '--bogus'], 'identity': HAVE_TEXT},
    'arg-bad-flag-before-help': {'argv': ['--bogus', '-h'], 'identity': HAVE_TEXT},
    'arg-short-unknown': {'argv': ['-x'], 'identity': HAVE_TEXT},
    # a bad identity.json is silently replaced: the agent's device_key goes with
    # it and the backend then answers 409 (design 1.7, D6; left as it is)
    'bad-not-json': {'argv': ['--no-qr'], 'identity': '{"robot_id": "MW-1", "secret": "S", "device_key": "dk"'},
    'bad-empty-file': {'argv': ['--no-qr'], 'identity': ''},
    'bad-list': {'argv': ['--no-qr'], 'identity': '[1]'},
    'bad-no-secret': {'argv': ['--no-qr'], 'identity': '{"robot_id": "MW-1", "device_key": "dk"}'},
    'bad-empty-secret': {'argv': ['--no-qr'], 'identity': '{"robot_id": "MW-1", "secret": "", "device_key": "dk"}'},
    'bad-no-robot-id': {'argv': ['--no-qr'], 'identity': '{"secret": "S"}'},
    'bad-robot-id-zero': {'argv': ['--no-qr'], 'identity': '{"robot_id": 0, "secret": "S"}'},
    'bad-not-utf8': {'argv': ['--no-qr'], 'identity': '{"robot_id": "MW-\xff", "secret": "S"}'},
    'bad-bom': {'argv': ['--no-qr'], 'identity': '\xef\xbb\xbf{"robot_id": "MW-1", "secret": "S"}'},
    'bad-with-rotate': {'argv': ['--rotate', '--no-qr'], 'identity': 'not json', 'random': [R1, R2]},
    # a directory where the file goes: load says none, the replace fails
    'identity-is-a-directory': {'argv': ['--no-qr'], 'identity_is_dir': True},
    # the .env's relay
    'env-relay': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': ENV},
    'env-relay-json': {'argv': ['--json'], 'identity': HAVE_TEXT, 'env_file': ENV},
    'env-double-quotes': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'PAIR_RELAY_URL="{RELAY}"\n'},
    'env-single-quotes': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f"PAIR_RELAY_URL='{RELAY}'\n"},
    'env-both-quotes': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f"""PAIR_RELAY_URL="'{RELAY}'"\n"""},
    'env-quotes-the-other-way': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f"""PAIR_RELAY_URL='"{RELAY}"'\n"""},
    'env-spaces': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'  PAIR_RELAY_URL = {RELAY}  \n'},
    'env-value-with-equals': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': 'PAIR_RELAY_URL=wss://r/x?a=1&b=2\n'},
    'env-commented-out': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'#PAIR_RELAY_URL={RELAY}\n  # PAIR_RELAY_URL=x\n'},
    'env-export-is-another-key': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'export PAIR_RELAY_URL={RELAY}\n'},
    'env-last-wins': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': 'PAIR_RELAY_URL=ws://a\nPAIR_RELAY_URL=ws://b\n'},
    'env-empty-value': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': 'PAIR_RELAY_URL=\n'},
    'env-crlf': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'A=1\r\nPAIR_RELAY_URL={RELAY}\r\nB=2\r\n'},
    'env-bare-cr': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'A=1\rPAIR_RELAY_URL={RELAY}\rB=2'},
    'env-no-newline-at-the-end': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': f'PAIR_RELAY_URL={RELAY}'},
    'env-missing-file': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_file': None},
    'env-is-a-directory': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'env_is_dir': True},
    # read after the identity is saved: fails, with the file already there
    'env-not-utf8-fails': {'argv': ['--no-qr'], 'env_file': 'A=\xff\n'},
    # no LAN address
    'lan-none': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'lan': ''},
    'lan-none-json': {'argv': ['--json'], 'identity': HAVE_TEXT, 'lan': ''},
    # the file's owner
    'owner-other-user': {'argv': ['--owner', 'mower', '--rotate', '--no-qr'], 'identity': HAVE_TEXT,
                         'passwd': {'cat': [1000, 1000], 'mower': [1001, 1002]}},
    'owner-unknown': {'argv': ['--owner', 'nobody9', '--rotate', '--no-qr'], 'identity': HAVE_TEXT},
    'owner-chown-denied': {'argv': ['--rotate', '--no-qr'], 'identity': HAVE_TEXT, 'chown_error': 'EPERM'},
    'owner-chown-eacces': {'argv': ['--rotate', '--no-qr'], 'identity': HAVE_TEXT, 'chown_error': 'EACCES'},
    # another error of chown is not swallowed; the .tmp stays
    'owner-chown-fails': {'argv': ['--rotate', '--no-qr'], 'identity': HAVE_TEXT, 'chown_error': 'EINVAL'},
    # the state dir
    'state-dir-with-a-slash': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'state_arg_suffix': '/'},
    'state-dir-from-the-environment': {'argv': ['--no-qr'], 'identity': HAVE_TEXT, 'state_from_env': True},
    'state-dir-empty-in-the-environment': {'argv': ['--no-qr'], 'state_env_empty': True},
    'state-dir-is-a-file': {'argv': ['--no-qr'], 'state_is_file': True},
    # names that are links: followed by nothing, in the state dir or at its own name
    'tmp-is-a-link': {'argv': ['--no-qr'], 'link_at': 'identity.json.tmp'},
    'identity-is-a-link': {'argv': ['--json'], 'link_at': 'identity.json'},
    'state-dir-is-a-link': {'argv': ['--no-qr'], 'state_is_link': True},
    'state-dir-with-a-slash-is-a-link': {'argv': ['--no-qr'], 'state_is_link': True, 'state_arg_suffix': '/'},
    'left-over-tmp': {'argv': ['--no-qr'], 'tmp_before': 'left over'},
}

CHOWN_ERRORS = {'EPERM': 1, 'EACCES': 13, 'EINVAL': 22}

PAIR_DRIVER = r'''
import argparse, builtins, json, os, pwd, runpy, secrets, socket, sys, time
cfg = json.loads(os.environ['DRIVER_CFG'])
real_open = builtins.open
def fake_open(file, *a, **k):
    if file in cfg['machine_files']:
        file = cfg['machine_files'][file]
    return real_open(file, *a, **k)
builtins.open = fake_open
time.time = lambda: cfg['now'] + 0.5
draws = list(cfg['random'])
def token_bytes(n):
    with real_open(cfg['log'], 'a') as f:
        f.write(json.dumps(['random', n]) + '\n')
    b = bytes.fromhex(draws.pop(0))
    assert len(b) == n
    return b
secrets.token_bytes = token_bytes
socket.gethostname = lambda: cfg['hostname']
class FakeSocket:
    def __init__(self, family, kind):
        assert (family, kind) == (socket.AF_INET, socket.SOCK_DGRAM)
    def connect(self, addr):
        assert addr == ('10.255.255.255', 1)
        if not cfg['lan']:
            raise OSError(101, 'Network is unreachable')
    def getsockname(self):
        return (cfg['lan'], 54321)
    def close(self):
        pass
socket.socket = FakeSocket
class Pw:
    def __init__(self, uid, gid):
        self.pw_uid, self.pw_gid = uid, gid
def getpwnam(name):
    if name not in cfg['passwd']:
        raise KeyError(name)
    return Pw(*cfg['passwd'][name])
pwd.getpwnam = getpwnam
def chown(who, uid, gid):
    with real_open(cfg['log'], 'a') as f:
        f.write(json.dumps(['chown', who if isinstance(who, str) else 'fd', uid, gid]) + '\n')
    if cfg['chown_errno']:
        raise OSError(cfg['chown_errno'], os.strerror(cfg['chown_errno']))
os.chown = os.fchown = chown
sys.argv = ['mower-pair'] + cfg['argv']
runpy.run_path(cfg['script'], run_name='__main__')
'''


def run_pair(tmp, case):
    state, cwd = tmp / 'state', tmp / 'cwd'
    cwd.mkdir()
    path = state / 'identity.json'
    other = tmp / 'other-file'
    other.write_text(OTHER_FILE)
    elsewhere = tmp / 'elsewhere'
    if case.get('state_is_file'):
        state.write_text('a file')
    elif case.get('state_is_link'):
        elsewhere.mkdir()
        state.symlink_to(elsewhere)
    elif not case.get('no_state_dir'):
        state.mkdir(mode=0o755)
    if case.get('link_at'):
        (state / case['link_at']).symlink_to(other)
    if case.get('tmp_before'):
        (state / 'identity.json.tmp').write_text(case['tmp_before'])
    old_mtime = 1000000000
    if 'identity' in case and case['identity'] is not None:
        path.write_bytes(case['identity'].encode('latin-1'))
        path.chmod(case.get('identity_mode', 0o600))
        os.utime(path, (old_mtime, old_mtime))
    if case.get('identity_is_dir'):
        path.mkdir()
    env_path = tmp / 'env'
    if case.get('env_is_dir'):
        env_path.mkdir()
    elif case.get('env_file') is not None:
        env_path.write_bytes(case['env_file'].encode('latin-1'))
    machine_files = {}
    ids = case.get('machine_ids', {'/etc/machine-id': MACHINE_ID + '\n'})
    for i, real in enumerate(('/etc/machine-id', '/var/lib/dbus/machine-id')):
        m = tmp / f'machine-{i}'
        machine_files[real] = str(m)
        if real in ids:
            m.write_bytes(ids[real].encode('latin-1'))
    bindir = fake_bin(tmp / 'bin', ('qrencode',) if case.get('qrencode') else ())
    log = tmp / 'log'
    state_arg = str(state) + case.get('state_arg_suffix', '')
    prefix = [] if case.get('state_from_env') or case.get('state_env_empty') else ['--state-dir', state_arg]
    argv = prefix + ['--env', str(env_path)] + case['argv']
    cfg = {'now': NOW, 'random': case.get('random', [R1]), 'hostname': case.get('hostname', 'lubancat'),
           'lan': case.get('lan', '192.168.0.113'), 'passwd': case.get('passwd', {'cat': [1000, 1000]}),
           'chown_errno': CHOWN_ERRORS.get(case.get('chown_error')), 'log': str(log), 'argv': argv,
           'script': str(PAIR_PY), 'machine_files': machine_files}
    (tmp / 'spec.json').write_text(json.dumps({'state': str(state), 'log': str(tmp / 'qr-log'), 'pulled': str(tmp / 'pulled')}))
    (tmp / 'driver.py').write_text(PAIR_DRIVER)
    env = {'PATH': str(bindir), 'DRIVER_CFG': json.dumps(cfg), 'PYTHONUTF8': '1', 'PYTHONDONTWRITEBYTECODE': '1',
           'FAKE_SPEC': str(tmp / 'spec.json')}
    if case.get('state_from_env'):
        env['MOWER_STATE_DIR'] = str(state)
    if case.get('state_env_empty'):
        env['MOWER_STATE_DIR'] = ''
    p = subprocess.run([sys.executable, str(tmp / 'driver.py')], capture_output=True, timeout=30, env=env,
                       cwd=cwd)
    stdout, stderr = p.stdout.decode('utf-8'), p.stderr.decode('utf-8')
    name = namer((state, STATE))
    text = None
    if path.is_file():
        text = path.read_bytes().decode('latin-1')
    log_rows = [json.loads(ln) for ln in log.read_text().splitlines()] if log.exists() else []
    qr_calls = []
    if (tmp / 'qr-log').exists():
        qr_calls = [json.loads(ln)[3:] for ln in (tmp / 'qr-log').read_text().splitlines()]
    tmp_file = state / 'identity.json.tmp'
    return {
        'given': case,
        'rc': p.returncode,
        # argparse's help and error texts are its own: the rc and what changed are the contract
        'stdout': [] if p.returncode == 2 or stdout.startswith('usage: ') else lines(name(stdout)),
        # the errno of a link at the state dir's name differs between systems, the raising does not
        'raises': 'OSError' if case.get('state_is_link') and p.returncode else exception_of(stderr),
        'other_untouched': other.read_text() == OTHER_FILE,
        'elsewhere_empty': not any(elsewhere.iterdir()) if elsewhere.is_dir() else None,
        'identity': text,
        'mode': stat.S_IMODE(path.stat().st_mode) if path.is_file() else None,
        'rewritten': path.is_file() and path.stat().st_mtime != old_mtime,
        'draws': [r[1] for r in log_rows if r[0] == 'random'],
        'chown': [[name(r[1]), r[2], r[3]] for r in log_rows if r[0] == 'chown'],
        'tmp_left': tmp_file.is_file(),
        'qrencode': [[name(a) for a in c] for c in qr_calls],
        'state_dir_mode': stat.S_IMODE(state.stat().st_mode) if case.get('no_state_dir') and state.is_dir() else None,
    }


def g5_now(tmp):
    return run_table(G5_CASES, run_pair, tmp)


@pytest.fixture(scope='module')
def g5(tmp_path_factory):
    return g5_now(tmp_path_factory.mktemp('g5'))


def pair_defaults(tmp):
    """The argument defaults, from the tool's own parser."""
    tmp = Path(tmp)
    driver = PAIR_DRIVER.replace(
        "sys.argv = ['mower-pair'] + cfg['argv']",
        "real_parse = argparse.ArgumentParser.parse_args\n"
        "def parse_args(self, *a, **k):\n"
        "    print(json.dumps(vars(real_parse(self, *a, **k))))\n"
        "    sys.exit(0)\n"
        "argparse.ArgumentParser.parse_args = parse_args\n"
        "sys.argv = ['mower-pair'] + cfg['argv']")
    (tmp / 'driver.py').write_text(driver)
    out = {}
    for name, extra in (('unset', {}), ('from-env', {'MOWER_STATE_DIR': '/some/where'})):
        cfg = {'now': NOW, 'random': [], 'hostname': 'x', 'lan': '', 'passwd': {}, 'chown_errno': None,
               'log': str(tmp / 'log'), 'argv': [], 'script': str(PAIR_PY), 'machine_files': {}}
        p = subprocess.run([sys.executable, str(tmp / 'driver.py')], capture_output=True, text=True, timeout=30,
                           env={'PATH': '/nonexistent', 'DRIVER_CFG': json.dumps(cfg), 'PYTHONDONTWRITEBYTECODE': '1', **extra})
        assert p.returncode == 0, p.stderr
        out[name] = json.loads(p.stdout)
    return out


def test_g5_matches_the_golden(g5):
    assert_same(g5, load_golden('g5_pair.json')['cases'])


def test_g5_defaults_are_the_goldens(tmp_path):
    assert load_golden('g5_pair.json')['defaults'] == pair_defaults(tmp_path)


def _identity(g5, case):
    return json.loads(g5[case]['identity'])


def _url(g5, case):
    return next(ln for ln in g5[case]['stdout'] if ln.startswith('https://'))


def test_g5_a_new_identity(g5):
    alphabet = '0123456789ABCDEFGHJKMNPQRSTVWXYZ'
    digest = hashlib.sha256(b'mower-robot-id:' + MACHINE_ID.encode()).digest()
    robot_id = 'MW-' + ''.join(alphabet[b % 32] for b in digest[:6])
    secret = base64.b32encode(bytes.fromhex(R1)).decode().rstrip('=')
    t = g5['new']
    assert t['identity'] == json.dumps({'robot_id': robot_id, 'name': 'lubancat', 'secret': secret, 'created': NOW},
                                       indent=2) + '\n'
    assert t['mode'] == 0o600 and t['rc'] == 0 and t['rewritten'] and not t['tmp_left']
    assert t['chown'] == [['fd', 1000, 1000]], 'on the open file'
    assert t['stdout'][:5] == [f'robot_id : {robot_id}', 'name     : lubancat', 'relay    : (none, LAN only)',
                               'lan      : 192.168.0.113:9090', f'identity : {STATE}/identity.json  (new secret)']
    assert _url(g5, 'new') == f'https://mower.fxrbindi.com/pair?v=1&id={robot_id}&s={secret}&n=lubancat&l=192.168.0.113'


def test_g5_nothing_is_written_without_a_reason(g5):
    for case in ('keep', 'keep-leaves-a-wrong-mode-alone', 'keep-json', 'keep-with-qrencode', 'name-empty-changes-nothing',
                 'arg-help', 'arg-unknown', 'arg-name-without-value'):
        t = g5[case]
        assert not t['rewritten'] and t['chown'] == [] and t['draws'] == [], case
        assert t['identity'] == HAVE_TEXT, case
    assert g5['keep']['mode'] == 0o600
    assert g5['keep-leaves-a-wrong-mode-alone']['mode'] == 0o644
    assert '(new secret)' not in '\n'.join(g5['keep']['stdout'])


def test_g5_rotate_keeps_the_agents_device_key(g5):
    d = _identity(g5, 'rotate')
    assert d['device_key'] == 'dk-1234' and d['rotated'] == NOW and d['secret'] != HAVE['secret']
    assert list(d) == ['robot_id', 'name', 'secret', 'created', 'device_key', 'rotated']
    assert g5['rotate']['mode'] == 0o600
    assert 'secret rotated: paired phones must scan again; restart the stack: sudo systemctl restart mower' in g5['rotate']['stdout']
    assert list(_identity(g5, 'rotate-again-replaces-rotated'))[-2:] == ['rotated', 'more']
    assert g5['rotate-fixes-the-mode']['mode'] == 0o600
    # a --name change says "new secret" too; it only says rotated for --rotate
    assert any('(new secret)' in ln for ln in g5['name-says-new-secret']['stdout'])
    assert not any(ln.startswith('secret rotated') for ln in g5['name-says-new-secret']['stdout'])


def test_g5_the_urls_escaping_is_quote_plus(g5):
    assert '&n=front+yard&' in _url(g5, 'name-space')
    assert '&n=a~b&' in _url(g5, 'name-tilde')
    assert '&n=a%2Ab&' in _url(g5, 'name-star')
    assert '&n=%E5%89%8D%E9%99%A2&' in _url(g5, 'name-chinese')
    assert '&n=a%26b%3Dc&' in _url(g5, 'name-ampersand')
    assert '&n=aZ09_.-~&' in _url(g5, 'name-safe-set')
    relay = 'h=wss%3A%2F%2Frelay.mower.fxrbindi.com%2Fr'
    url = _url(g5, 'env-relay')
    assert url.index('?v=1&id=') < url.index('&s=') < url.index('&n=') < url.index('&' + relay) < url.index('&l=')
    assert '&l=' not in _url(g5, 'lan-none')
    assert '&n=' not in _url(g5, 'keep-no-name') and '&n=' not in _url(g5, 'keep-null-name')


def test_g5_json_is_the_identity_plus_three(g5):
    d = json.loads('\n'.join(g5['env-relay-json']['stdout']))
    assert list(d) == list(HAVE) + ['relay', 'lan', 'url']
    assert d['relay'] == RELAY and d['lan'] == '192.168.0.113' and d['url'].startswith('https://mower.fxrbindi.com/pair?')
    # non-ASCII as \uXXXX, astral characters as a surrogate pair
    assert '  "name": "\\u524d\\u9662",' in g5['name-chinese-json']['stdout']
    assert '  "name": "\\ud83c\\udf31",' in g5['name-emoji-json']['stdout']
    # what the file had under those names is overwritten where it stood
    d = json.loads('\n'.join(g5['keep-key-order-and-extras']['stdout']))
    assert list(d) == ['zz', 'secret', 'relay', 'robot_id', 'url', 'lan', 'a']
    # the QR is for people
    assert g5['keep-json-with-qrencode']['qrencode'] == []


def test_g5_the_qr_comes_first_when_stdout_is_a_pipe(g5):
    """Python's stdout is block-buffered on a pipe, and qrencode writes to the same fd. On a terminal
    the text comes first (the e2e tests of the Rust port hold that half)."""
    t = g5['keep-with-qrencode']
    assert t['stdout'][0] == '<QR>' and t['stdout'][1].startswith('robot_id : ')
    url = t['stdout'][-2]
    assert url.startswith('https://') and t['qrencode'] == [['-t', 'ANSIUTF8', '-m', '2', url]]
    assert '(install qrencode to print the QR code here)' in g5['keep-says-qrencode-is-missing']['stdout']


def test_g5_a_bad_identity_is_replaced_and_its_device_key_lost(g5):
    """The known danger of design 1.7 / D6, pinned: the Rust port keeps it."""
    for case in ('bad-not-json', 'bad-empty-file', 'bad-list', 'bad-no-secret', 'bad-empty-secret', 'bad-no-robot-id',
                 'bad-robot-id-zero', 'bad-not-utf8', 'bad-bom'):
        t = g5[case]
        assert t['rc'] == 0 and t['rewritten'], case
        assert list(_identity(g5, case)) == ['robot_id', 'name', 'secret', 'created'], case
    assert g5['bad-with-rotate']['draws'] == [20, 20]


def test_g5_machine_id_fallbacks(g5):
    alphabet = '0123456789ABCDEFGHJKMNPQRSTVWXYZ'

    def rid(seed):
        return 'MW-' + ''.join(alphabet[b % 32] for b in hashlib.sha256(b'mower-robot-id:' + seed).digest()[:6])
    assert _identity(g5, 'machine-id-second-path')['robot_id'] == rid(b'dbus-machine-id')
    assert _identity(g5, 'machine-id-empty-goes-on')['robot_id'] == rid(b'x')
    assert _identity(g5, 'machine-id-stripped')['robot_id'] == rid(MACHINE_ID.encode())
    t = g5['machine-id-none-is-hostname-and-random']
    assert t['draws'] == [8, 20] and _identity(g5, 'machine-id-none-is-hostname-and-random')['robot_id'] == rid(
        b'lubancat' + bytes.fromhex(R8))
    assert g5['machine-id-not-utf8-fails']['raises'] == 'UnicodeDecodeError'


def test_g5_failures_and_owner(g5):
    assert g5['owner-other-user']['chown'] == [['fd', 1001, 1002]]
    assert g5['owner-unknown']['chown'] == [] and g5['owner-unknown']['rc'] == 0
    for case in ('owner-chown-denied', 'owner-chown-eacces'):
        assert g5[case]['rc'] == 0 and g5[case]['rewritten'], case
    t = g5['owner-chown-fails']
    assert t['rc'] == 1 and t['raises'] == 'OSError' and not t['rewritten'] and t['tmp_left']
    t = g5['identity-is-a-directory']
    assert t['rc'] == 1 and t['tmp_left'] and t['identity'] is None
    t = g5['env-not-utf8-fails']
    assert t['rc'] == 1 and t['raises'] == 'UnicodeDecodeError' and t['rewritten'], 'saved before the .env is read'
    assert g5['state-dir-empty-in-the-environment']['rc'] == 1
    assert g5['state-dir-is-a-file']['rc'] == 1
    for case, rc in (('arg-unknown', 2), ('arg-positional', 2), ('arg-name-without-value', 2), ('arg-ambiguous', 2),
                     ('arg-help', 0), ('arg-help-before-a-bad-flag', 0), ('arg-bad-flag-before-help', 0)):
        assert g5[case]['rc'] == rc, case


def test_g5_no_link_is_followed(g5):
    t = g5['tmp-is-a-link']
    assert t['rc'] == 0 and t['other_untouched'] and not t['tmp_left'] and t['mode'] == 0o600
    t = g5['identity-is-a-link']
    assert t['rc'] == 0 and t['other_untouched'] and t['mode'] == 0o600
    assert 'MW-OTHER' not in '\n'.join(t['stdout']), 'not read through the link either'
    for case in ('state-dir-is-a-link', 'state-dir-with-a-slash-is-a-link'):
        assert g5[case]['rc'] == 1 and g5[case]['elsewhere_empty'], case
    assert g5['left-over-tmp']['rc'] == 0 and not g5['left-over-tmp']['tmp_left']


def test_g5_the_defaults_a_person_gets():
    d = load_golden('g5_pair.json')['defaults']
    assert d['unset'] == {'state_dir': STATE, 'owner': 'cat', 'env': f'{MOWER}/.env', 'name': None, 'rotate': False,
                          'json': False, 'no_qr': False}
    assert d['from-env']['state_dir'] == '/some/where'


# ---- G6: robot_status.json -> mower-update.sh --auto ---------------------------

def _rs(**over):
    """What robot_status writes (info.rs write_robot_status): sorted keys, no newline."""
    doc = {'busy': False, 'moving': False, 'nav_running': False, 'software': '0.6.0', 'time': NOW}
    doc.update(over)
    return json.dumps(doc, separators=(',', ':'))


BUSY_RS = _rs(busy=True, moving=True)
G6_CASES = {
    'busy-fresh': {'robot_status': BUSY_RS},
    'idle-fresh': {'robot_status': _rs()},
    'busy-age-30': {'robot_status': _rs(busy=True, time=NOW - 30)},
    'busy-age-31-is-stale': {'robot_status': _rs(busy=True, time=NOW - 31)},
    'busy-from-the-future': {'robot_status': _rs(busy=True, time=NOW + 100)},
    'no-file': {},
    'busy-null': {'robot_status': _rs(busy=None)},
    'busy-absent': {'robot_status': json.dumps({'time': NOW})},
    'busy-the-string-true': {'robot_status': _rs(busy='true')},
    'busy-the-string-True': {'robot_status': _rs(busy='True')},
    'busy-one': {'robot_status': _rs(busy=1)},
    'bad-json-means-not-busy': {'robot_status': 'not json'},
    'empty-file-means-not-busy': {'robot_status': ''},
    # only digits are a time, read as decimal; anything else is no time, so not busy
    'busy-time-absent': {'robot_status': json.dumps({'busy': True})},
    'busy-time-null': {'robot_status': _rs(busy=True, time=None)},
    'busy-time-float': {'robot_status': _rs(busy=True, time=NOW + 0.5)},
    'busy-time-empty-string': {'robot_status': _rs(busy=True, time='')},
    'busy-time-negative': {'robot_status': _rs(busy=True, time=-5)},
    'busy-time-not-only-digits': {'robot_status': _rs(busy=True, time=f'{NOW}+0')},
    'busy-time-hex': {'robot_status': _rs(busy=True, time=hex(NOW))},
    'busy-time-digits-in-a-string': {'robot_status': _rs(busy=True, time=str(NOW))},
    'busy-time-leading-zeros': {'robot_status': _rs(busy=True, time=f'0{NOW}')},
    'off-in-the-env-file': {'robot_status': _rs(), 'env_file': 'IMAGE_TAG=main\nMOWER_AUTO_UPDATE=0\n'},
    'off-in-the-environment': {'robot_status': _rs(), 'env': {'MOWER_AUTO_UPDATE': '0'}},
    'env-file-beats-the-environment': {'robot_status': _rs(), 'env': {'MOWER_AUTO_UPDATE': '0'},
                                       'env_file': 'IMAGE_TAG=main\nMOWER_AUTO_UPDATE=1\n'},
    'lock-held': {'robot_status': _rs(), 'lock': True},
    'lock-held-and-busy': {'robot_status': BUSY_RS, 'lock': True},
    # not --auto: the script does not look at robot_status at all
    'manual-ignores-busy': {'robot_status': BUSY_RS, 'args': []},
    'check-ignores-busy': {'robot_status': BUSY_RS, 'args': ['--check']},
    'busy-holds-back-a-new-image': {'robot_status': BUSY_RS, 'docker': NEW_ON_THE_REGISTRY},
    'idle-takes-a-new-image': {'robot_status': _rs(), 'docker': NEW_ON_THE_REGISTRY},
    # the retries of a cut-off pull look again: a scheduled update stops, one someone asked for goes on
    'auto-stops-when-the-robot-turns-busy': {'robot_status': _rs(), 'docker': {
        **NEW_ON_THE_REGISTRY, 'pull_fail': 'unexpected EOF', 'busy_on_pull_fail': NOW}},
    'manual-goes-on-when-the-robot-turns-busy': {'robot_status': _rs(), 'args': [], 'docker': {
        **NEW_ON_THE_REGISTRY, 'pull_fail': 'unexpected EOF', 'busy_on_pull_fail': NOW}},
}

# value in the JSON -> what json_field prints
JSON_FIELD_TABLE = {
    'true': True, 'false': False, 'null': None, 'zero': 0, 'one': 1, 'float': 1.5, 'word': 'pulling',
    'empty': '', 'list': [1, 'a'], 'dict': {'a': 1},
}


def run_auto(tmp, case):
    return run_update(tmp, {**case, 'args': case.get('args', ['--auto'])})


def json_field_table(tmp):
    """deploy/host/mower-update.sh's json_field, cut out of it and run."""
    tmp = Path(tmp)
    text = UPDATE_SH.read_text()
    m = re.search(r'^json_field\(\) \{.*?^\}$', text, re.S | re.M)
    helper = re.search(r'^as_state_owner\(\) \{.*?^\}$', text, re.S | re.M)
    assert m and helper, 'mower-update.sh no longer has its json_field'
    bindir = fake_bin(tmp / 'bin', (), STATE_DIR_TOOLS)
    doc = tmp / 'doc.json'
    doc.write_text(json.dumps(JSON_FIELD_TABLE))
    (tmp / 'bad.json').write_text('not json')
    out = {}

    def field(path, key):
        p = subprocess.run(['/bin/bash', '-c', f'STATE_DIR={tmp}\n{helper.group(0)}\n{m.group(0)}\njson_field "$1" "$2"', 'x', str(path), key],
                           capture_output=True, text=True, timeout=30, env={'PATH': str(bindir)})
        return {'rc': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    for key in JSON_FIELD_TABLE:
        out[key] = field(doc, key)
    out['absent-key'] = field(doc, 'nope')
    out['no-such-file'] = field(tmp / 'missing.json', 'state')
    out['bad-file'] = field(tmp / 'bad.json', 'state')
    return out


def g6_now(tmp):
    tmp = Path(tmp)
    (tmp / 'auto').mkdir()
    (tmp / 'field').mkdir()
    return {'cases': run_table(G6_CASES, run_auto, tmp / 'auto'), 'json_field': json_field_table(tmp / 'field')}


@pytest.fixture(scope='module')
def g6(tmp_path_factory):
    return g6_now(tmp_path_factory.mktemp('g6'))


def test_g6_matches_the_golden(g6):
    golden = load_golden('g6_auto_update.json')
    assert_same(g6['cases'], golden['cases'])
    assert_same(g6['json_field'], golden['json_field'])


def _went_on(t):
    return any(c[2:4] == ['docker', 'manifest'] for c in t['calls'])


def test_g6_busy_holds_the_update_back(g6):
    held = 'robot is busy (moving or navigating), trying again next time'
    for case in ('busy-fresh', 'busy-age-30', 'busy-from-the-future', 'busy-the-string-true', 'busy-time-digits-in-a-string',
                 'busy-holds-back-a-new-image'):
        t = g6['cases'][case]
        assert t['stdout'] == [f'[mower-update] {held}'] and t['calls'] == [] and t['rc'] == 0, case
    for case in ('idle-fresh', 'busy-age-31-is-stale', 'no-file', 'busy-null', 'busy-absent', 'busy-the-string-True', 'busy-one',
                 'bad-json-means-not-busy', 'empty-file-means-not-busy', 'manual-ignores-busy', 'idle-takes-a-new-image'):
        assert _went_on(g6['cases'][case]), case
    assert any(c[2:5] == ['docker', 'compose', 'up'] for c in g6['cases']['idle-takes-a-new-image']['calls'])


def test_g6_only_digits_are_a_time(g6):
    for case in ('busy-time-absent', 'busy-time-null', 'busy-time-float', 'busy-time-empty-string', 'busy-time-negative',
                 'busy-time-not-only-digits', 'busy-time-hex'):
        t = g6['cases'][case]
        assert _went_on(t) and t['stderr'] == [] and t['rc'] == 0, case
    # a time may be in a string, and its leading zeros do not make it octal
    for case in ('busy-time-digits-in-a-string', 'busy-time-leading-zeros'):
        t = g6['cases'][case]
        assert not _went_on(t) and t['stdout'] == ['[mower-update] robot is busy (moving or navigating), trying again next time'], case


def test_g6_a_retry_looks_at_busy_only_for_a_scheduled_update(g6):
    def pulls(case):
        return [c for c in g6['cases'][case]['calls'] if c[2:4] == ['docker', 'pull']]
    auto = g6['cases']['auto-stops-when-the-robot-turns-busy']
    assert len(pulls('auto-stops-when-the-robot-turns-busy')) == 1 and auto['rc'] == 1
    assert 'the robot became busy' in json.loads(auto['update_status.json'])['message']
    assert len(pulls('manual-goes-on-when-the-robot-turns-busy')) == 3


def test_g6_auto_can_be_switched_off(g6):
    for case in ('off-in-the-env-file', 'off-in-the-environment'):
        t = g6['cases'][case]
        assert t['stdout'] == ['[mower-update] auto update disabled (MOWER_AUTO_UPDATE=0)'] and t['calls'] == [], case
    # .env is sourced after the environment is read
    assert _went_on(g6['cases']['env-file-beats-the-environment'])


def test_g6_a_held_lock_is_looked_at_first(g6):
    for case in ('lock-held', 'lock-held-and-busy'):
        assert g6['cases'][case]['stdout'] == [
            '[mower-update] an update is already in progress (another mower-update.sh holds the lock)'], case


def test_g6_json_field_says_what_the_design_says(g6):
    f = g6['json_field']
    assert f['true'] == {'rc': 0, 'stdout': 'true\n', 'stderr': ''}
    assert f['false']['stdout'] == 'false\n'
    assert f['null']['stdout'] == '\n' and f['absent-key']['stdout'] == '\n' and f['empty']['stdout'] == '\n'
    assert f['zero']['stdout'] == '0\n' and f['float']['stdout'] == '1.5\n' and f['word']['stdout'] == 'pulling\n'
    assert f['list']['stdout'] == "[1, 'a']\n" and f['dict']['stdout'] == "{'a': 1}\n"
    assert f['no-such-file']['rc'] == 1 and f['bad-file']['rc'] == 1
    assert f['no-such-file']['stdout'] == '' and f['bad-file']['stdout'] == ''


# ---- writing the goldens ------------------------------------------------------

def write_goldens():
    GOLDEN.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t).resolve()
        for d in ('g2', 'g3', 'g4', 'g5', 'g6', 'g5d'):
            (tmp / d).mkdir()
        g2_ = g2_now(tmp / 'g2')
        (GOLDEN / 'g2_host_request.json').write_text(golden_text(
            'G2 host.request (E6 design 4.3), written by src/mower_mission/test/test_host_contracts.py. writers: what '
            'utils/mower-host-request and mower_mission/host_request.py write and refuse, and rust: what '
            'write_host_request must accept, refuse and write (mower_rs_common/tests/host_request_golden.rs). cases: '
            'deploy/host/mower-host-request.sh on each request. A call is [update_status.json state at that moment, '
            'its cwd, the tool, its arguments]. time is fixed. Temporary paths are shown as the production ones.',
            g2_['consumer'], writers=g2_['writers']), encoding='utf-8')
        (GOLDEN / 'g3_update_files.json').write_text(golden_text(
            'G3 update_status.json, update_check.json, image.json (E6 design 4.3): deploy/host/mower-update.sh against '
            'a fake docker, given shown per case. A call is [update_status.json state at that moment, its cwd, the tool, '
            'its arguments].', g3_now(tmp / 'g3')), encoding='utf-8')
        (GOLDEN / 'g4_link_status.json').write_text(golden_text(
            'G4 link_status.json (E6 design 4.3): deploy/host/mower-link-status.py --once. given.modem: the replies per '
            'AT command, bytes as latin-1 characters, several strings 200 ms apart; a command with no entry gets no '
            'answer. stdout is --once, file what it wrote; a case that raises leaves the file it found, ' + STALE_LINK +
            '. mower_link (Rust) is held to these, modulo `raises`, which is only Python saying why it exited 1.',
            g4_now(tmp / 'g4'), defaults=link_defaults(), sweeps=link_sweeps()), encoding='utf-8')
        (GOLDEN / 'g5_pair.json').write_text(golden_text(
            'G5 identity.json and the pairing URL (E6 design 4.3): deploy/host/mower-pair --state-dir S --env E + argv, '
            'with machine-id, hostname, randomness, clock, LAN address and users replaced, and stdout a pipe. '
            'given.identity and identity are the file, bytes as latin-1 characters. An argparse exit (usage, help) keeps '
            'no stdout: its texts are not the contract. mowerctl pair (Rust) is held to these, modulo `raises`.',
            g5_now(tmp / 'g5'), defaults=pair_defaults(tmp / 'g5d')), encoding='utf-8')
        g6_ = g6_now(tmp / 'g6')
        (GOLDEN / 'g6_auto_update.json').write_text(golden_text(
            'G6 robot_status.json -> deploy/host/mower-update.sh --auto (E6 design 4.3), and json_field, its one reader, '
            'cut out of the script. robot_status.json as robot_status/src/info.rs writes it.', g6_['cases'],
            json_field=g6_['json_field']), encoding='utf-8')
    print(f'wrote {GOLDEN}')


if __name__ == '__main__':
    if sys.argv[1:] != ['--write-golden']:
        sys.exit('usage: test_host_contracts.py --write-golden')
    write_goldens()

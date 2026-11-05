"""The root host tools and the state dir (deploy/host/).

The state dir belongs to the robot user and the container writes it, so what is
in it is input, and a name in it can be anything. These tools run as root (their
systemd units have no User=, the others by sudo). The rules they are held to:

  - a value read from the state dir is validated before it is used as a number;
  - a write there does not follow a link, whether at the temporary name, at the
    final name or at the state dir's own name;
  - an owner or a mode is set on the open file, never by name;
  - the shell tools do their reads and writes there as the directory's owner.

Run as root (CI does, in its own step; the dev container is root) the tests also
check owners and the user each write runs as; those skip otherwise. Linux only:
the tools use stat -c, setpriv and flock.
"""
import json
import os
import pwd
import shutil
import stat
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='the host tools are Linux tools')

HOST = Path(__file__).resolve().parents[3] / 'deploy' / 'host'
UPDATE_SH = HOST / 'mower-update.sh'
REQUEST_SH = HOST / 'mower-host-request.sh'
COLLECTION_SH = HOST / 'mower-data-collection.sh'
LINK_PY = HOST / 'mower-link-status.py'
PAIR_PY = HOST / 'mower-pair'

AS_ROOT = os.geteuid() == 0
root_only = pytest.mark.skipif(not AS_ROOT, reason='needs root: owners, and the user a write runs as')
# the state dir's owner when running as root: nobody real, and not root
OWNER = 4321 if AS_ROOT else os.geteuid()
OWNER_GID = 4321 if AS_ROOT else os.getegid()
# for mower-pair's --owner, which takes a name
PAIR_USER = 'nobody' if AS_ROOT else pwd.getpwuid(os.geteuid()).pw_name
OTHER_TEXT = '{"robot_id": "MW-OTHER", "secret": "OTHER", "action": "reboot", "requested_by": "other-file", "busy": true}\n'

FAKE_DOCKER = '''#!/bin/sh
case "$*" in
  *"{{.Id}}"*) echo sha256:id ;;
  *RepoDigests*) echo repo@sha256:aa ;;
  "manifest inspect"*) echo '{"Descriptor": {"digest": "sha256:aa"}}' ;;
  *" ps "*) echo cid ;;
  "inspect --format"*) echo sha256:id ;;
  *" exec "*) echo /camera/front/image_raw/compressed ;;
esac
exit 0
'''


class World:
    """A robot's directories under a temporary one any user can walk."""

    def __init__(self, base):
        self.base = base
        self.state = base / 'state'
        self.state.mkdir()
        os.chown(self.state, OWNER, OWNER_GID)
        self.mower = base / 'opt_mower'
        (self.mower / 'host').mkdir(parents=True)
        (self.mower / '.env').write_text('IMAGE_TAG=main\n')
        self.bin = base / 'bin'
        self.bin.mkdir()
        self.tool('docker', FAKE_DOCKER)
        self.tool('systemctl', '#!/bin/sh\nexit 0\n')
        self.tool('logger', '#!/bin/sh\nexit 0\n')
        self.tool('sleep', '#!/bin/sh\nexit 0\n')
        # what must stay as it is, wherever a name in the state dir points
        self.other = base / 'other-file'
        self.other.write_text(OTHER_TEXT)
        self.other.chmod(0o640)
        self.other_before = self._other_now()

    def tool(self, name, text):
        path = self.bin / name
        path.write_text(text)
        path.chmod(0o755)

    def _other_now(self):
        st = self.other.lstat()
        return self.other.read_text(), stat.S_IMODE(st.st_mode), st.st_uid, st.st_gid

    def assert_other_untouched(self):
        assert self._other_now() == self.other_before, 'a file outside the state dir changed'

    def own(self, path):
        """As if the robot user had made it."""
        os.lchown(path, OWNER, OWNER_GID)

    def link(self, name, target=None):
        path = self.state / name
        path.symlink_to(target or self.other)
        self.own(path)
        return path

    def put(self, name, text):
        path = self.state / name
        path.write_text(text)
        self.own(path)
        return path

    def env(self, **more):
        env = {'PATH': f'{self.bin}:{os.environ["PATH"]}', 'MOWER_DIR': str(self.mower), 'MOWER_STATE_DIR': str(self.state),
               'MOWER_CPUFREQ_DIR': str(self.base / 'no-cpufreq'), 'PYTHONDONTWRITEBYTECODE': '1', 'HOME': str(self.base)}
        env.update(more)
        return env

    def _run(self, argv, **more):
        return subprocess.run(argv, env=self.env(**more), text=True, capture_output=True, timeout=30, cwd=self.base)

    def update(self, *args, **more):
        return self._run(['/bin/bash', str(UPDATE_SH), *args], **more)

    def request(self, action='restart', by='app', text=None, **more):
        if text is None:
            text = json.dumps({'action': action, 'time': int(time.time()), 'requested_by': by})
        self.put('host.request', text)
        return self._run(['/bin/bash', str(REQUEST_SH), str(self.state)], **more)

    def serve_request(self, **more):
        return self._run(['/bin/bash', str(REQUEST_SH), str(self.state)], **more)

    def collection(self, command):
        return self._run(['/bin/bash', str(COLLECTION_SH), command], ENV_FILE=str(self.mower / '.env'),
                         MAIN_COMPOSE_FILE=str(self.mower / 'docker-compose.yaml'))

    def link_status(self, state=None):
        return self._run([sys.executable, str(LINK_PY), str(state or self.state), '--once'], MOWER_LTE_AT=str(self.base / 'none'))

    def pair(self, *args, state=None):
        return self._run([sys.executable, str(PAIR_PY), '--state-dir', str(state or self.state), '--owner', PAIR_USER,
                          '--env', str(self.base / 'no-env'), '--no-qr', *args])

    def with_hook(self, script, argv, hook, **more):
        """Run a Python tool with `hook` (Python text, `state` and `other` given) in front of it."""
        driver = self.base / 'driver.py'
        driver.write_text(f'import json, os, runpy, sys\nstate = {str(self.state)!r}\nother = {str(self.other)!r}\n'
                          f'{hook}\nsys.argv = [{str(script)!r}] + {argv!r}\nrunpy.run_path({str(script)!r}, run_name="__main__")\n')
        return self._run([sys.executable, str(driver)], **more)

    def owner_of(self, name):
        st = (self.state / name).lstat()
        return st.st_uid, st.st_gid

    def mode_of(self, name):
        return stat.S_IMODE((self.state / name).lstat().st_mode)


@pytest.fixture
def world():
    base = Path(tempfile.mkdtemp(prefix='mower-state-dir-')).resolve()
    base.chmod(0o755)
    try:
        yield World(base)
    finally:
        shutil.rmtree(base, ignore_errors=True)


def _regular_json(path):
    assert not path.is_symlink(), path
    return json.loads(path.read_text())


# ---- values read from the state dir --------------------------------------------

def _robot_status(world, value):
    world.put('robot_status.json', json.dumps({'busy': True, 'moving': True, 'time': value}))


def test_a_fresh_busy_status_holds_a_scheduled_update(world):
    _robot_status(world, int(time.time()))
    res = world.update('--auto')
    assert res.returncode == 0, res.stderr
    assert 'robot is busy' in res.stdout and 'up_to_date' not in res.stdout


def test_digits_in_a_string_are_still_a_time(world):
    _robot_status(world, str(int(time.time())))
    assert 'robot is busy' in world.update('--auto').stdout


@pytest.mark.parametrize('form', ['{now}+0', '{now}-0', '{now}*1', 'hex', '+{now}'])
def test_only_digits_count_as_a_time(world, form):
    """Each of these was a number to the shell's arithmetic and is not one here."""
    now = int(time.time())
    value = hex(now) if form == 'hex' else form.format(now=now)
    _robot_status(world, value)
    res = world.update('--auto')
    assert res.returncode == 0, res.stderr
    assert 'robot is busy' not in res.stdout
    assert 'up_to_date' in res.stdout
    assert res.stderr == ''


@pytest.mark.parametrize('value', [None, '', 1790000000.5, True, [1], ' 12', '12 ', '1\n2', '-5', 'ROBOT_STATUS_MAX_AGE_S'])
def test_a_time_that_is_no_number_means_not_busy(world, value):
    _robot_status(world, value)
    res = world.update('--auto')
    assert res.returncode == 0, res.stderr
    assert 'robot is busy' not in res.stdout
    assert 'up_to_date' in res.stdout
    assert res.stderr == ''


def test_a_status_without_a_time_means_not_busy(world):
    world.put('robot_status.json', json.dumps({'busy': True}))
    res = world.update('--auto')
    assert 'up_to_date' in res.stdout and res.stderr == ''


def test_a_time_is_read_as_decimal_whatever_its_leading_zeros(world):
    _robot_status(world, '0' + str(int(time.time())))
    res = world.update('--auto')
    assert 'robot is busy' in res.stdout and res.stderr == ''


def test_digits_too_long_for_the_shell_are_harmless(world):
    _robot_status(world, '9' * 40)
    res = world.update('--auto')
    assert res.returncode == 0 and res.stderr == ''


# ---- writes do not follow links --------------------------------------------------

@pytest.mark.parametrize('name', ['update_status.json.tmp', 'image.json.tmp', 'update_check.json.tmp'])
def test_the_update_script_does_not_write_through_a_temporary_name(world, name):
    world.link(name)
    res = world.update()
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / name.removesuffix('.tmp'))
    assert not (world.state / name).exists(), 'a temporary name stays behind'


@pytest.mark.parametrize('name', ['update_status.json', 'image.json', 'update_check.json'])
def test_the_update_script_replaces_a_link_at_a_final_name(world, name):
    world.link(name)
    res = world.update('--force')
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / name)


def test_a_check_does_not_write_through_a_temporary_name(world):
    world.link('update_check.json.tmp')
    res = world.update('--check')
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / 'update_check.json')


def test_the_lock_leaves_the_target_of_a_link_alone(world):
    world.link('.update.lock')
    res = world.update()
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert not (world.state / '.update.lock').is_symlink()


def test_the_lock_is_a_regular_file_afterwards_whatever_was_there(world):
    os.mkfifo(world.state / '.update.lock')
    world.own(world.state / '.update.lock')
    res = world.update('--check')
    assert res.returncode == 0, res.stderr
    assert (world.state / '.update.lock').is_file()


def _rm_that_leaves_a_link_behind(world, name):
    """The name is taken again right after a tool removed it."""
    real = shutil.which('rm')
    world.tool('rm', f'#!/bin/sh\n{real} "$@"\nln -s {world.other} {world.state / name} 2>/dev/null\nexit 0\n')


def test_a_name_taken_again_after_the_removal_is_not_written_through(world):
    _rm_that_leaves_a_link_behind(world, 'update_check.json.tmp')
    world.update('--check')
    world.assert_other_untouched()


def test_the_lock_is_only_ever_opened_for_reading(world):
    world.link('.update.lock')
    _rm_that_leaves_a_link_behind(world, '.update.lock')
    world.update('--check')
    world.assert_other_untouched()


def test_a_status_that_is_not_a_file_does_not_stop_an_update(world):
    os.mkfifo(world.state / 'update_status.json')
    world.own(world.state / 'update_status.json')
    res = world.update()
    assert res.returncode == 0, res.stderr
    assert _regular_json(world.state / 'update_status.json')['state'] == 'up_to_date'


def test_the_request_handler_does_not_write_through_a_temporary_name(world):
    world.link('update_status.json.tmp')
    res = world.request('restart')
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / 'update_status.json')['message'] == 'restarted'


def test_the_request_handler_replaces_a_link_at_the_final_name(world):
    world.link('update_status.json')
    res = world.request('reboot', 'container')
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / 'update_status.json')['state'] == 'rebooting'


@root_only
def test_data_collection_does_not_write_through_a_temporary_name(world):
    world.link('recorder_profile.json.tmp')
    res = world.collection('start')
    try:
        assert res.returncode == 0, res.stderr
        world.assert_other_untouched()
        assert _regular_json(world.state / 'recorder_profile.json')['profile'] == 'data_collection'
        assert world.mode_of('recorder_profile.json') == 0o644
    finally:
        world.collection('stop')
    assert not (world.state / 'recorder_profile.json').exists()


@root_only
def test_data_collection_writes_and_removes_as_the_owner(world):
    rm = _log_uid_of(world, 'rm')
    res = world.collection('start')
    try:
        assert res.returncode == 0, res.stderr
        assert world.owner_of('recorder_profile.json') == (OWNER, OWNER_GID)
    finally:
        rm.write_text('')
        assert world.collection('stop').returncode == 0
    # the helper's own /run file is root's; the request is the owner's
    assert str(OWNER) in rm.read_text().split()
    assert not (world.state / 'recorder_profile.json').exists()


@pytest.mark.parametrize('name', ['link_status.json.tmp', 'link_status.json'])
def test_link_status_does_not_write_through_a_link(world, name):
    world.link(name)
    res = world.link_status()
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / 'link_status.json')['lte'] == {'present': False}
    assert not (world.state / 'link_status.json.tmp').exists()


def test_link_status_refuses_a_state_dir_that_is_a_link(world):
    real = world.base / 'elsewhere'
    real.mkdir()
    link = world.base / 'state-link'
    link.symlink_to(real)
    res = world.link_status(link)
    assert res.returncode == 1
    assert list(real.iterdir()) == []


def test_link_status_refuses_a_state_dir_with_a_slash_that_is_a_link(world):
    real = world.base / 'elsewhere'
    real.mkdir()
    link = world.base / 'state-link'
    link.symlink_to(real)
    res = world.link_status(f'{link}/')
    assert res.returncode == 1
    assert list(real.iterdir()) == []


def test_link_status_replaces_what_an_earlier_run_left_at_the_temporary_name(world):
    world.put('link_status.json.tmp', 'left over')
    res = world.link_status()
    assert res.returncode == 0, res.stderr
    assert _regular_json(world.state / 'link_status.json')['lte'] == {'present': False}


def _name_taken_again_after_its_removal(name):
    return ('real_unlink = os.unlink\n'
            'def unlink(path, **kw):\n'
            '    try:\n'
            '        real_unlink(path, **kw)\n'
            '    finally:\n'
            f'        os.symlink(other, os.path.join(state, {name!r}))\n'
            'os.unlink = unlink\n')


def test_link_status_does_not_open_a_name_taken_again(world):
    res = world.with_hook(LINK_PY, [str(world.state), '--once'], _name_taken_again_after_its_removal('link_status.json.tmp'),
                          MOWER_LTE_AT=str(world.base / 'none'))
    assert res.returncode == 1, 'the next round tries again'
    world.assert_other_untouched()


def test_pair_does_not_open_a_name_taken_again(world):
    res = world.with_hook(PAIR_PY, ['--state-dir', str(world.state), '--owner', PAIR_USER, '--env', str(world.base / 'no-env'),
                                    '--no-qr'], _name_taken_again_after_its_removal('identity.json.tmp'))
    assert res.returncode == 1
    world.assert_other_untouched()


def test_link_status_sets_the_owner_on_the_open_file(world):
    """The temporary name changes while the file is being finished."""
    hook = ('tmp = os.path.join(state, "link_status.json.tmp")\n'
            'real_dump = json.dump\n'
            'def dump(*a, **k):\n'
            '    real_dump(*a, **k)\n'
            '    os.unlink(tmp)\n'
            '    os.symlink(other, tmp)\n'
            'json.dump = dump\n')
    res = world.with_hook(LINK_PY, [str(world.state), '--once'], hook, MOWER_LTE_AT=str(world.base / 'none'))
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()


def test_link_status_takes_its_owner_from_the_open_file(world):
    """The name changes between the rename and whatever comes after it."""
    hook = ('tmp = os.path.join(state, "link_status.json.tmp")\n'
            'real_replace = os.replace\n'
            'def replace(src, dst, **kw):\n'
            '    os.unlink(tmp)\n'
            '    os.symlink(other, tmp)\n'
            '    return real_replace(src, dst, **kw)\n'
            'os.replace = replace\n')
    other = world.other
    other.chmod(0o640)
    res = world.with_hook(LINK_PY, [str(world.state), '--once'], hook, MOWER_LTE_AT=str(world.base / 'none'))
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()


def test_pair_does_not_write_through_a_temporary_name(world):
    world.link('identity.json.tmp')
    res = world.pair()
    assert res.returncode == 0, res.stderr
    world.assert_other_untouched()
    assert _regular_json(world.state / 'identity.json')['robot_id'].startswith('MW-')
    assert world.mode_of('identity.json') == 0o600


def test_pair_replaces_a_link_at_its_file_and_does_not_read_through_it(world):
    world.link('identity.json')
    res = world.pair('--json')
    assert res.returncode == 0, res.stderr
    assert 'MW-OTHER' not in res.stdout and 'OTHER' not in res.stdout
    world.assert_other_untouched()
    assert _regular_json(world.state / 'identity.json')['robot_id'] != 'MW-OTHER'


def test_pair_refuses_a_state_dir_that_is_a_link(world):
    real = world.base / 'elsewhere'
    real.mkdir()
    link = world.base / 'state-link'
    link.symlink_to(real)
    res = world.pair(state=link)
    assert res.returncode == 1
    assert list(real.iterdir()) == []


def test_pair_does_not_wait_on_something_that_is_not_a_file(world):
    os.mkfifo(world.state / 'identity.json')
    world.own(world.state / 'identity.json')
    res = world.pair()
    assert res.returncode == 0, res.stderr
    assert (world.state / 'identity.json').is_file()


def test_pair_sets_the_mode_and_owner_on_the_open_file(world):
    """The temporary name changes while the file is being finished."""
    hook = ('import pwd\n'
            'tmp = os.path.join(state, "identity.json.tmp")\n'
            'def swap():\n'
            '    os.unlink(tmp)\n'
            '    os.symlink(other, tmp)\n'
            'real_dump = json.dump\n'
            'def dump(*a, **k):\n'
            '    real_dump(*a, **k)\n'
            '    swap()\n'
            'json.dump = dump\n')
    res = world.with_hook(PAIR_PY, ['--state-dir', str(world.state), '--owner', PAIR_USER, '--env', str(world.base / 'no-env'),
                                    '--no-qr'], hook)
    world.assert_other_untouched()
    assert 'other-file' not in res.stdout


def test_the_secret_is_private_from_the_moment_the_file_exists(world):
    hook = ('seen = os.path.join(os.path.dirname(state), "mode-seen")\n'
            'real_dump = json.dump\n'
            'def dump(data, f, **k):\n'
            '    with open(seen, "w") as out:\n'
            '        out.write(oct(os.fstat(f.fileno()).st_mode & 0o777))\n'
            '    real_dump(data, f, **k)\n'
            'json.dump = dump\n')
    res = world.with_hook(PAIR_PY, ['--state-dir', str(world.state), '--owner', PAIR_USER, '--env', str(world.base / 'no-env'),
                                    '--no-qr'], hook)
    assert res.returncode == 0, res.stderr
    assert (world.base / 'mode-seen').read_text() == '0o600'


# ---- as root: owners, and who does the writing ------------------------------------

@root_only
@pytest.mark.parametrize('name', ['update_status.json', 'image.json', 'update_check.json'])
def test_the_update_scripts_files_belong_to_the_state_dirs_owner(world, name):
    assert world.update().returncode == 0
    assert world.owner_of(name) == (OWNER, OWNER_GID)
    assert world.mode_of(name) == 0o644


def test_the_files_stay_readable_whatever_umask_a_person_has(world):
    p = subprocess.run(['/bin/bash', '-c', 'umask 077; exec "$@"', 'x', '/bin/bash', str(UPDATE_SH)], env=world.env(),
                       text=True, capture_output=True, timeout=30, cwd=world.base)
    assert p.returncode == 0, p.stderr
    for name in ('update_status.json', 'image.json', 'update_check.json'):
        assert world.mode_of(name) == 0o644, name


@root_only
def test_the_request_handlers_status_belongs_to_the_state_dirs_owner(world):
    assert world.request('restart').returncode == 0
    assert world.owner_of('update_status.json') == (OWNER, OWNER_GID)
    assert world.mode_of('update_status.json') == 0o644


@root_only
def test_link_status_belongs_to_the_state_dirs_owner(world):
    assert world.link_status().returncode == 0
    assert world.owner_of('link_status.json') == (OWNER, OWNER_GID)
    assert world.mode_of('link_status.json') == 0o644


@root_only
def test_pair_gives_the_identity_to_its_owner(world):
    assert world.pair().returncode == 0
    uid = pwd.getpwnam(PAIR_USER)
    assert world.owner_of('identity.json') == (uid.pw_uid, uid.pw_gid)
    assert world.mode_of('identity.json') == 0o600


def _log_uid_of(world, tool):
    """`tool` on the PATH noting who runs it; the file is writable by anyone."""
    log = world.base / f'{tool}-uids'
    log.touch()
    log.chmod(0o666)
    real = shutil.which(tool)
    world.tool(tool, f'#!/bin/sh\nid -u >> {log}\nexec {real} "$@"\n')
    return log


@root_only
@pytest.mark.parametrize('tool', ['mv', 'cat'])
def test_the_update_script_writes_as_the_owner(world, tool):
    log = _log_uid_of(world, tool)
    assert world.update().returncode == 0
    uids = set(log.read_text().split())
    assert uids == {str(OWNER)}


@root_only
def test_the_request_handler_writes_and_removes_as_the_owner(world):
    mv, rm = _log_uid_of(world, 'mv'), _log_uid_of(world, 'rm')
    assert world.request('restart').returncode == 0
    assert set(mv.read_text().split()) == {str(OWNER)}
    assert set(rm.read_text().split()) == {str(OWNER)}
    assert not (world.state / 'host.request').exists()


@root_only
def test_a_writer_does_not_get_roots_environment(world):
    (world.mower / '.env').write_text('IMAGE_TAG=main\nMOWER_PROVISION_TOKEN=the-token\n')
    seen = world.base / 'env-seen'
    seen.touch()
    seen.chmod(0o666)
    world.tool('mv', f'#!/bin/sh\nenv >> {seen}\nexec {shutil.which("mv")} "$@"\n')
    assert world.update().returncode == 0
    assert 'PATH=' in seen.read_text()
    assert 'the-token' not in seen.read_text()


@root_only
def test_the_request_is_read_as_the_owner(world):
    """A request the owner cannot read is no request."""
    world.other.chmod(0o600)
    world.other_before = world._other_now()
    world.link('host.request')
    res = world.serve_request()
    assert res.returncode == 0, res.stderr
    assert not (world.state / 'update_status.json').exists()
    assert not (world.state / 'host.request').is_symlink()
    world.assert_other_untouched()


@root_only
def test_the_robot_status_is_read_as_the_owner(world):
    now = int(time.time())
    world.other.write_text(json.dumps({'busy': True, 'time': now}))
    world.other.chmod(0o600)
    world.other_before = world._other_now()
    world.link('robot_status.json')
    res = world.update('--auto')
    assert 'robot is busy' not in res.stdout and 'up_to_date' in res.stdout


@root_only
def test_what_a_previous_version_left_as_root_does_not_get_in_the_way(world):
    """Before this version the scripts wrote these as root: they must still be replaced."""
    for name in ('update_status.json', 'update_status.json.tmp', 'image.json.tmp', 'update_check.json.tmp', '.update.lock'):
        (world.state / name).write_text('{"state": "idle"}')
        os.chown(world.state / name, 0, 0)
        (world.state / name).chmod(0o644)
    res = world.update()
    assert res.returncode == 0, res.stderr
    assert _regular_json(world.state / 'update_status.json')['state'] == 'up_to_date'
    assert world.owner_of('update_status.json') == (OWNER, OWNER_GID)
    assert not (world.state / 'update_status.json.tmp').exists()


@root_only
def test_a_root_owned_state_dir_stays_writable(world):
    """A first run may make the directory itself: nobody else writes there."""
    root_state = world.base / 'root-state'
    res = world.update(MOWER_STATE_DIR=str(root_state))
    assert res.returncode == 0, res.stderr
    assert _regular_json(root_state / 'update_status.json')['state'] == 'up_to_date'


@root_only
def test_a_link_at_the_state_dirs_name_gets_only_its_owners_rights(world):
    """A link made by the robot user does not lend root to the write."""
    victim_dir = world.base / 'root-dir'
    victim_dir.mkdir()
    victim_dir.chmod(0o755)
    link = world.base / 'state-link'
    link.symlink_to(victim_dir)
    world.own(link)
    res = world.update('--check', MOWER_STATE_DIR=str(link))
    assert list(victim_dir.iterdir()) == [], res.stdout + res.stderr


# ---- what must keep working ---------------------------------------------------------

def _install_update_script(world):
    target = world.mower / 'host' / 'mower-update.sh'
    shutil.copy(UPDATE_SH, target)
    target.chmod(0o755)


def test_the_apps_update_path_end_to_end(world):
    """host.request as the container writes it -> the handler -> mower-update.sh -> update_status.json."""
    _install_update_script(world)
    res = world.request('update', 'app')
    assert res.returncode == 0, res.stderr
    assert not (world.state / 'host.request').exists()
    status = _regular_json(world.state / 'update_status.json')
    assert status['state'] == 'up_to_date' and type(status['time']) is int
    assert _regular_json(world.state / 'image.json')['digest'] == 'sha256:aa'


@root_only
def test_the_container_can_read_and_replace_what_the_tools_write(world):
    """uid 1000 in production: it reads these, and owns the directory they are in."""
    _install_update_script(world)
    assert world.request('update', 'app').returncode == 0
    assert world.link_status().returncode == 0
    for name in ('update_status.json', 'image.json', 'update_check.json', 'link_status.json'):
        p = subprocess.run(['setpriv', f'--reuid={OWNER}', f'--regid={OWNER_GID}', '--clear-groups', 'cat', str(world.state / name)],
                           capture_output=True, text=True, timeout=10)
        assert p.returncode == 0 and json.loads(p.stdout), (name, p.stderr)


def test_a_held_lock_is_still_a_running_update(world):
    """A script of the previous version holds it on the same file, opened for writing."""
    import fcntl
    held = open(world.state / '.update.lock', 'w')
    fcntl.flock(held, fcntl.LOCK_EX)
    try:
        res = world.update()
    finally:
        held.close()
    assert 'already in progress' in res.stdout
    assert not (world.state / 'update_status.json').exists()


def test_the_three_shell_tools_carry_the_same_helpers():
    def function(text, name):
        start = text.index(f'{name}() {{')
        return text[start:text.index('\n}\n', start)]
    update, request, collection = (p.read_text() for p in (UPDATE_SH, REQUEST_SH, COLLECTION_SH))
    for name in ('as_state_owner', 'state_write'):
        assert function(update, name) == function(request, name) == function(collection, name), name

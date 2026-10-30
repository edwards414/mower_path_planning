#!/usr/bin/env python3
"""Image A/B: run two robot images side by side on fake devices and diff what
the app can see (tools/ab/README.md). Host side; stdlib + websockets.

    ab.py run IMAGE_A IMAGE_B [--out DIR] [--cpu-seconds N] [--stop-grace S]
                              [--checks C,...] [--report-only]
    ab.py compare DIR [--checks C,...] [--report-only]
                              redo DIR/diff.txt and DIR/report.md

Exit 0 when the diff is empty (in the --checks given, default all), 1 when it
is not (0 with --report-only), 2 when the harness itself failed.
"""
import argparse
import concurrent.futures
import gzip
import json
import os
import re
import secrets
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
RS_TOOLS = os.path.join(REPO, 'src', 'mower_rs', 'tools')
COMPOSE = os.path.join(REPO, 'deploy', 'docker-compose.yaml')
PAIR = os.path.join(HERE, 'fixtures', 'pair.json')
BASELINE = os.path.join(RS_TOOLS, 'app_contract_baseline.json')

sys.path.insert(0, HERE)
import compare  # noqa: E402

# The robot's /opt/mower/.env as far as the stack sees it: every RUST_* switch
# on except RUST_NAV_CORE; GPS, NAV_COMPOSITION and ROSBRIDGE_ADDRESS at their
# compose defaults. Then what the harness has to differ in: no firmware flash
# (the fake STM32 has no bootloader) and no fleet backend (the agent idles).
ROBOT_ENV = {
    **{f'RUST_{s}': 'true' for s in (
        'STATUS', 'ADAPTER', 'RECORD', 'NAV', 'BATTERY', 'PID_AUTOTUNE', 'MAP', 'AGENT',
        'GUARDS', 'IMU', 'BRIDGE', 'BASE', 'LOCALIZE', 'DAEMON')},
    'RUST_NAV_CORE': 'false',
    'MOWER_FIRMWARE_SYNC': '0',
    'MOWER_BACKEND_URL': '',
}
DOMAIN = {'A': '41', 'B': '42'}   # one ROS domain per side, besides one netns each


def compose_service(path, name):
    """`command:` and `environment:` of one service of a compose file, read as
    the subset of YAML deploy/docker-compose.yaml writes them in: block
    sequences of plain scalars under `services: <name>:`."""
    out, section, service, key = {}, None, None, None
    for line in open(path):
        text = '' if line.lstrip().startswith('#') else line.split(' #')[0].rstrip()
        if not text.strip():
            continue
        indent = len(text) - len(text.lstrip())
        item = text.strip()
        if indent == 0:
            section = item.rstrip(':')
        elif section == 'services' and indent == 2:
            service, key = item.rstrip(':'), None
        elif section == 'services' and service == name and indent == 4:
            key = item[:-1] if item.endswith(':') else None
            if key:
                out[key] = []
        elif key and service == name and item.startswith('- '):
            out[key].append(item[2:].strip().strip('\'"'))
    return out


def interpolate(text, env):
    """compose's ${VAR}, ${VAR:-default} and ${VAR-default}."""
    def sub(m):
        name, op, default = m.group(1), m.group(2), m.group(3) or ''
        value = env.get(name)
        if op == ':-':
            return value if value else default
        if op == '-':
            return default if value is None else value
        return value or ''
    return re.sub(r'\$\{(\w+)(?:(:?-)([^}]*))?\}', sub, text)


def docker(*args, check=True, timeout=None):
    r = subprocess.run(['docker', *args], capture_output=True, text=True, timeout=timeout)
    if check and r.returncode != 0:
        raise RuntimeError(f'docker {" ".join(args[:3])}: {r.stderr.strip()}')
    return r.stdout.strip()


def image_info(ref):
    if subprocess.run(['docker', 'image', 'inspect', ref], capture_output=True).returncode != 0:
        print(f'[ab] pulling {ref}', flush=True)
        docker('pull', '--quiet', ref, timeout=1800)
    doc = json.loads(docker('image', 'inspect', ref))[0]
    env = dict(e.split('=', 1) for e in doc['Config'].get('Env') or [])
    return {'ref': ref, 'id': doc['Id'], 'repo_digests': doc.get('RepoDigests') or [],
            'arch': doc.get('Architecture'), 'created': doc.get('Created'), 'size': doc.get('Size'),
            'git_sha': env.get('MOWER_GIT_SHA', ''),
            'mower_env': {k: v for k, v in env.items() if k.startswith('MOWER_')},
            'entrypoint': doc['Config'].get('Entrypoint') or []}


class Side:
    def __init__(self, label, image, out, run_id, args):
        self.label, self.image, self.out = label, image, os.path.join(out, label)
        self.name = f'mower-ab-{run_id}-{label}'
        self.run_id, self.cpu_seconds, self.grace = run_id, args.cpu_seconds, args.stop_grace
        self.info, self.times = {}, {}

    def log(self, msg):
        print(f'[ab {self.label}] {time.strftime("%H:%M:%S")} {msg}', flush=True)

    def prepare(self, info):
        """The stack's argv and environment: the image's entrypoint and
        compose's lawan_node command, with the robot's switches."""
        os.makedirs(self.out, exist_ok=True)
        env = {**ROBOT_ENV, 'ROS_DOMAIN_ID': DOMAIN[self.label]}
        svc = compose_service(COMPOSE, 'lawan_node')
        command = [interpolate(a, env) for a in svc['command']]
        environment = [interpolate(e, env) for e in svc['environment']]
        with open(os.path.join(self.out, 'stack.argv'), 'w') as f:
            f.write('\n'.join(info['entrypoint'] + command) + '\n')
        with open(os.path.join(self.out, 'stack.env'), 'w') as f:
            f.write('\n'.join(environment) + '\n')
        self.info = {'image': info, 'command': command, 'environment': environment}

    def start(self):
        docker('run', '-d', '--name', self.name, '--label', f'mower-ab={self.run_id}',
               '-u', '0', '--cap-add', 'NET_ADMIN',
               '-p', '127.0.0.1::19090', '-p', '127.0.0.1::19091',
               '-v', f'{HERE}:/ab:ro', '-v', f'{RS_TOOLS}:/rs-tools:ro', '-v', f'{self.out}:/run-in:ro',
               '--entrypoint', 'bash', self.image, '/ab/side.sh')
        self.t0 = time.monotonic()

        def port(p):
            return docker('port', self.name, f'{p}/tcp').splitlines()[0].rsplit(':', 1)[1]
        self.gate, self.loopback = f'ws://127.0.0.1:{port(19090)}', f'ws://127.0.0.1:{port(19091)}'
        self.log(f'{self.image} as {self.name}: gate {self.gate}, loopback {self.loopback}')

    def stamp(self, what):
        self.times[what] = round(time.monotonic() - self.t0, 1)

    def python(self, script, args, out_name):
        """Start a host-side probe; its output goes to out_name."""
        f = open(os.path.join(self.out, out_name), 'w')
        p = subprocess.Popen([sys.executable, script, *args], stdout=f, stderr=subprocess.STDOUT,
                             env={**os.environ, 'MOWER_PAIR_FILE': PAIR})
        p.out_file = f
        return p

    @staticmethod
    def finish(p, timeout):
        try:
            return p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            p.kill()
            return p.wait()
        finally:
            p.out_file.close()

    def probe_app(self, seconds, name):
        """probe_app.py, against the robot's own baseline, saving what it saw."""
        return self.python(os.path.join(RS_TOOLS, 'probe_app.py'),
                           [self.gate, str(seconds), '--baseline', BASELINE,
                            '--save', os.path.join(self.out, name + '.json')], name + '.txt')

    def drive(self):
        """Everything one side goes through, start to stop."""
        probe = os.path.join(HERE, 'probe.py')
        try:
            if self.finish(self.python(probe, ['wait', self.loopback, '120'], 'bridge_wait.txt'), 180):
                self.log('the bridge never answered')
                return
            self.stamp('bridge_up_s')
            # until the app would show the robot healthy (online, fresh base
            # and odometry, a pose that updates): every side starts its checks
            # from that state, however long its start took
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                if self.finish(self.probe_app(5, 'ready'), 60) == 0:
                    self.stamp('contract_ok_s')
                    break
                time.sleep(1)
            self.log(f'ready: bridge {self.times["bridge_up_s"]} s, '
                     f'app contract {self.times.get("contract_ok_s", "never OK in 120")} s')
            # the app contract, one client, then two at once
            self.finish(self.probe_app(20, 'contract'), 90)
            for p in [self.probe_app(20, f'contract_2{c}') for c in 'ab']:
                self.finish(p, 90)
            # lists and read-only services on the loopback port, then every
            # listed topic through the gate
            self.finish(self.python(probe, ['services', self.loopback, os.path.join(self.out, 'services.json')],
                                    'services.txt'), 60)
            self.finish(self.python(probe, ['topics', self.gate, '10', os.path.join(self.out, 'topics.json')],
                                    'topics.txt'), 60)
            self.stamp('probes_done_s')
            if self.cpu_seconds:
                docker('exec', self.name, 'python3', '/ab/incontainer.py', 'cpu',
                       str(self.cpu_seconds), '/tmp/ab/cpu_idle.json', check=False, timeout=self.cpu_seconds + 60)
        finally:
            self.stop()

    def stop(self):
        """`docker stop` with compose's grace period, as an update or a
        reboot stops the robot's container; then the files out of it."""
        docker('exec', self.name, 'python3', '/ab/incontainer.py', 'snapshot', '/tmp/ab', check=False)
        # the container's clock, which also stamps its log: the lines before
        # this are what the stack printed while it ran, the rest its stop
        t_stop = float(docker('exec', self.name, 'date', '+%s.%N', check=False) or 'inf')
        t = time.monotonic()
        docker('stop', '--time', str(self.grace), self.name, check=False, timeout=self.grace + 60)
        state = json.loads(docker('inspect', '--format', '{{json .State}}', self.name))
        self.info['stop'] = {'seconds': round(time.monotonic() - t, 1), 'exit_code': state.get('ExitCode'),
                             'grace_s': self.grace}
        self.stamp('stopped_s')
        logs = subprocess.run(['docker', 'logs', '--timestamps', self.name], capture_output=True, text=True,
                              errors='replace')
        running, stopping = compare.split_log(logs.stdout + logs.stderr, t_stop)
        for name, lines in (('stack.log', running), ('stack_stop.log', stopping)):
            with open(os.path.join(self.out, name), 'w') as f:
                f.writelines(lines)
        docker('cp', f'{self.name}:/tmp/ab/.', self.out, check=False)
        docker('cp', f'{self.name}:/home/mower/.mower', os.path.join(self.out, 'state'), check=False)
        docker('rm', '-f', self.name, check=False)
        frames = os.path.join(self.out, 'fake_base.jsonl')
        if os.path.exists(frames):
            with open(frames, 'rb') as src, gzip.open(frames + '.gz', 'wb') as dst:
                shutil.copyfileobj(src, dst)
            os.remove(frames)
        self.log(f'stopped: docker stop took {self.info["stop"]["seconds"]} s, '
                 f'exit code {self.info["stop"]["exit_code"]}')


def run(args):
    out = os.path.abspath(args.out or os.path.join('ab-out', time.strftime('%Y%m%d-%H%M%S')))
    os.makedirs(out, exist_ok=True)
    run_id = secrets.token_hex(3)
    t0 = time.time()
    sides = [Side(label, ref, out, run_id, args) for label, ref in (('A', args.image_a), ('B', args.image_b))]
    for s in sides:
        s.prepare(image_info(s.image))
    try:
        for s in sides:
            s.start()
        with concurrent.futures.ThreadPoolExecutor(len(sides)) as pool:
            for f in [pool.submit(s.drive) for s in sides]:
                f.result()
    finally:
        for s in sides:
            docker('rm', '-f', s.name, check=False)
    meta = {'run_id': run_id, 'started': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(t0)),
            'seconds': round(time.time() - t0, 1),
            'sides': {s.label: {**s.info, 'times': s.times, 'ros_domain_id': DOMAIN[s.label]} for s in sides}}
    with open(os.path.join(out, 'run.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    return compare.render(out, args.checks)


def checks(text):
    names = tuple(c.strip() for c in text.split(',') if c.strip())
    unknown = set(names) - set(compare.CHECKS)
    if unknown or not names:
        raise argparse.ArgumentTypeError(f'checks are {",".join(compare.CHECKS)}; got {text!r}')
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='run both images and compare')
    r.add_argument('image_a')
    r.add_argument('image_b')
    r.add_argument('--out', help='output directory (default ab-out/<time>)')
    r.add_argument('--cpu-seconds', type=int, default=30, help='informational CPU window, 0 for none')
    r.add_argument('--stop-grace', type=int, default=60,
                   help="docker stop's grace period (compose's stop_grace_period is 60)")
    c = sub.add_parser('compare', help='compare an earlier run again')
    c.add_argument('dir')
    for p in (r, c):
        p.add_argument('--checks', type=checks, default=compare.CHECKS,
                       help=f'the checks whose difference fails the run (default all: {",".join(compare.CHECKS)})')
        p.add_argument('--report-only', action='store_true', help='exit 0 whatever the diff')
    args = ap.parse_args()
    try:
        empty = run(args) if args.cmd == 'run' else compare.render(os.path.abspath(args.dir), args.checks)
    except Exception as e:  # noqa: BLE001 -- any failure of the harness itself
        print(f'[ab] harness error: {type(e).__name__}: {e}', file=sys.stderr)
        return 2
    return 0 if empty or args.report_only else 1


if __name__ == '__main__':
    sys.exit(main())

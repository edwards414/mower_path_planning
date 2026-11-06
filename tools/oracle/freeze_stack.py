#!/usr/bin/env python3
"""Freezes what a robot image's running stack shows, the part of E9-06 that needs
the production stack: the DDS graph with its QoS, `ros2 param dump` of each
module node that serves parameters, what launch gives mower_rsd, the start-up
log and who writes /rosout. deploy/test/golden/ros_facts/README.md says what
they are and what is normalised.

    freeze_stack.py IMAGE [--work DIR] [--check]

IMAGE runs as tools/ab runs it, on fake devices with compose's lawan_node command
and the robot's switches, nothing of the image replaced. It has to be a ROS
image, and to be of the tree it runs from: launch's copies of the parameter
files must be the tree's. The tree's files are rewritten; `git diff` shows what
changed. --check then fails if anything but the log and PROVENANCE.txt did: the
log's lines depend on the commit and the host, so its differences are a warning.

Host side, stdlib only (it uses tools/ab's helpers). In the image it runs
facts/graph_dump.py and facts/rosout_listen.py, which need rclpy.
"""
import argparse
import json
import os
import re
import secrets
import shlex
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
FACTS = os.path.join(REPO, 'deploy', 'test', 'golden', 'ros_facts')
AB = os.path.join(REPO, 'tools', 'ab')
sys.path.insert(0, AB)
import ab  # noqa: E402
import compare  # noqa: E402

DOMAIN = '43'   # its own network namespace anyway
WAIT_NAV2_S = 300
ROS = 'source /opt/ros/jazzy/setup.bash; source /mower_ws/install/setup.bash; '

# What is specific to the run or the host, and what stands in for it. The state
# dir and the address are compose's defaults (deploy/docker-compose.yaml); the
# robot id is tools/ab's fixture. The repo is public: no address or home
# directory goes in, tests/frozen.rs checks that.
SCRUB = [
    (re.compile(r'/home/mower/\.mower'), '<MOWER_STATE_DIR>'),
    (re.compile(r'/home/mower'), '<HOME>'),
    (re.compile(r'10\.77\.0\.2'), '<ROSBRIDGE_ADDRESS>'),
    (re.compile(r'\bMW-E2TEST\b'), '<ROBOT_ID>'),
]
# Node names with a process id or an address in them.
VARIABLE_NODES = [
    (re.compile(r'transform_listener_impl_[0-9a-f]+'), 'transform_listener_impl_*'),
    (re.compile(r'launch_ros_\d+'), 'launch_ros_*'),
]
# Log lines the harness itself causes.
HARNESS_LOG = re.compile(r'/(ros_facts_|_ros2cli)')


def scrub(text):
    for rx, sub in SCRUB:
        text = rx.sub(sub, text)
    return text


def node_name(name):
    for rx, sub in VARIABLE_NODES:
        name = rx.sub(sub, name)
    return name


class Stack:
    def __init__(self, image, work):
        self.image, self.work = image, work
        self.name = f'mower-ros-facts-{secrets.token_hex(4)}'
        self.info = ab.image_info(image)

    def start(self):
        env = {**ab.ROBOT_ENV, 'ROS_DOMAIN_ID': DOMAIN}
        svc = ab.compose_service(ab.COMPOSE, 'lawan_node')
        command = [ab.interpolate(a, env) for a in svc['command']]
        environment = [ab.interpolate(e, env) for e in svc['environment']]
        with open(os.path.join(self.work, 'stack.argv'), 'w') as f:
            f.write('\n'.join(self.info['entrypoint'] + command) + '\n')
        with open(os.path.join(self.work, 'stack.env'), 'w') as f:
            f.write('\n'.join(environment) + '\n')
        ab.docker('run', '-d', '--name', self.name, '-u', '0', '--cap-add', 'NET_ADMIN',
                  '-v', f'{AB}:/ab:ro', '-v', f'{os.path.join(REPO, "src/mower_rs/tools")}:/rs-tools:ro',
                  '-v', f'{self.work}:/run-in:ro', '-v', f'{os.path.join(HERE, "facts")}:/facts:ro',
                  '--entrypoint', 'bash', self.image, '/ab/side.sh')

    def run(self, cmd, timeout=120):
        """`cmd` in the image, in ROS's environment, as the stack's user."""
        r = subprocess.run(['docker', 'exec', '-u', '1000', '-e', 'HOME=/home/mower', '-e', f'ROS_DOMAIN_ID={DOMAIN}',
                            self.name, 'bash', '-c', ROS + cmd], capture_output=True, text=True, timeout=timeout)
        if r.returncode != 0:
            raise RuntimeError(f'in the image: {cmd}: exit {r.returncode}\n{r.stdout}\n{r.stderr}')
        return r.stdout

    def start_rosout_listener(self):
        subprocess.run(['docker', 'exec', '-d', '-u', '1000', '-e', 'HOME=/home/mower', '-e', f'ROS_DOMAIN_ID={DOMAIN}',
                        self.name, 'bash', '-c',
                        ROS + 'mkdir -p /tmp/facts; exec python3 /facts/rosout_listen.py '
                              '--out /tmp/facts/rosout_names --stop-file /tmp/facts/stop_rosout'], check=True)

    def logs(self):
        r = subprocess.run(['docker', 'logs', self.name], capture_output=True, text=True, errors='replace')
        return r.stdout + r.stderr

    def wait_nav2(self):
        t0 = time.monotonic()
        while time.monotonic() - t0 < WAIT_NAV2_S:
            if 'Managed nodes are active' in self.logs():
                print(f'[facts] nav2 active after {time.monotonic() - t0:.0f} s', flush=True)
                return
            state = ab.docker('inspect', '-f', '{{.State.Running}}', self.name)
            if state != 'true':
                raise RuntimeError('the container stopped:\n' + self.logs()[-3000:])
            time.sleep(2)
        raise RuntimeError(f'nav2 not active within {WAIT_NAV2_S} s:\n' + self.logs()[-3000:])

    def stop(self):
        subprocess.run(['docker', 'rm', '-f', self.name], capture_output=True)


def graph_doc(raw):
    """The tool's graph with variable node names replaced and everything sorted
    again. Nothing else is left out: every QoS field stays."""
    def endpoint(e):
        return {'node': node_name(e['node']), 'type': e['type'], 'hash': e['hash'], 'qos': e['qos']}

    def sort_key(e):
        return json.dumps(e, sort_keys=True)

    topics = {}
    for name, t in raw['topics'].items():
        topics[node_name(name)] = {
            'types': t['types'],
            'publishers': sorted((endpoint(e) for e in t['publishers']), key=sort_key),
            'subscribers': sorted((endpoint(e) for e in t['subscribers']), key=sort_key)}
    services = {}
    for name, s in raw['services'].items():
        entry = services.setdefault(node_name(name), {'types': s['types'], 'servers': [], 'clients': []})
        entry['servers'] += [node_name(n) for n in s['servers']]
        entry['clients'] += [node_name(n) for n in s['clients']]
    for s in services.values():
        s['servers'].sort()
        s['clients'].sort()
    return {'nodes': sorted(node_name(n) for n in raw['nodes']), 'topics': topics, 'services': services}


def one_line(v):
    return json.dumps(v, ensure_ascii=False, separators=(',', ':'))


def graph_text(doc):
    """Sorted, one endpoint a line, so a diff says which endpoint changed."""
    out = ['{', ' "nodes": [']
    out.append(',\n'.join(f'  {one_line(n)}' for n in doc['nodes']))
    out += [' ],', ' "topics": {']
    topics = []
    for name in sorted(doc['topics']):
        t = doc['topics'][name]
        rows = [f'  {one_line(name)}: {{', f'   "types": {one_line(t["types"])},']
        for side in ('publishers', 'subscribers'):
            body = ',\n'.join(f'    {one_line(e)}' for e in t[side])
            rows.append(f'   "{side}": [' + (f'\n{body}\n   ]' if body else ']') + (',' if side == 'publishers' else ''))
        rows.append('  }')
        topics.append('\n'.join(rows))
    out.append(',\n'.join(topics))
    out += [' },', ' "services": {']
    out.append(',\n'.join(f'  {one_line(n)}: {one_line(doc["services"][n])}' for n in sorted(doc['services'])))
    out += [' }', '}']
    return '\n'.join(out) + '\n'


def module_nodes():
    """The nodes of mower_rsd's modules: what the resolver oracle made of the
    manifest."""
    with open(os.path.join(FACTS, 'resolver.json')) as f:
        return sorted({n['node'] for n in json.load(f)['names']} | {'mower_bus_bridge'})


def launch_facts(stack, out):
    """`--modules`, the rules and the parameter files of mower_rsd's argv, in order.
    A file of the image's install tree is named by its path in the repo, and
    must be the tree's; launch's temporary node-scoped files are kept."""
    argv = stack.run('pid=$(pgrep -x mower_rsd); tr "\\0" "\\n" </proc/$pid/cmdline').splitlines()
    doc = {'modules': argv[argv.index('--modules') + 1].split(','), 'remaps': [], 'params_files': []}
    for flag, value in zip(argv, argv[1:]):
        if flag == '-r':
            doc['remaps'].append(value)
        elif flag == '--params-file':
            text = stack.run(f'cat {shlex.quote(value)}')
            installed = re.fullmatch(r'/mower_ws/install/(\w+)/share/\1/(.*)', value)
            temporary = re.fullmatch(r'/tmp/mower_rsd_([a-z_]+)_[a-z0-9_]{8}\.yaml', value)
            if installed:
                path = os.path.join('src', installed.group(1), installed.group(2))
                with open(os.path.join(REPO, path)) as f:
                    if f.read() != text:
                        raise RuntimeError(f'the image\'s {value} is not this tree\'s {path}: the image is of another commit')
                doc['params_files'].append(path)
            elif temporary:
                rel = os.path.join('launch', temporary.group(1) + '.yaml')
                with open(os.path.join(out, rel), 'w') as f:
                    f.write(scrub(text))
                doc['params_files'].append(rel)
            else:
                raise RuntimeError(f'a parameter file of no known kind: {value}')
    with open(os.path.join(out, 'launch', 'args.json'), 'w') as f:
        json.dump(doc, f, indent=1)
        f.write('\n')


def startup_log(text, info):
    """The distinct lines mower_rsd printed, sorted, as tools/ab normalises them
    (stamps, decimal numbers, its own identity), and without what depends on the
    machine (compare.TIMING) or on this harness."""
    ident = compare.Identity(info['mower_env'])
    lines = set()
    for line in text.splitlines():
        m = re.match(r'\[mower_rsd-\d+\] (.*)', compare.LOG_SUBS[0][0].sub('', line))
        if not m:
            continue
        line = ident.text(m.group(1))
        for rx, sub in compare.LOG_SUBS:
            line = rx.sub(sub, line)
        line = scrub(line).rstrip()
        if line and not compare.TIMING.search(line) and not HARNESS_LOG.search(line):
            lines.add(line)
    return ''.join(line + '\n' for line in sorted(lines))


def freeze(stack, out):
    os.makedirs(os.path.join(out, 'launch'))
    os.makedirs(os.path.join(out, 'params'))
    os.makedirs(os.path.join(out, 'log'))
    stack.start()
    stack.start_rosout_listener()
    stack.wait_nav2()
    # the listener goes before the graph is read: only the stack on it
    time.sleep(5)
    stack.run('touch /tmp/facts/stop_rosout')
    t0 = time.monotonic()
    while True:
        try:
            names = stack.run('cat /tmp/facts/rosout_names')
            break
        except RuntimeError:
            if time.monotonic() - t0 > 30:
                raise
            time.sleep(0.5)
    with open(os.path.join(out, 'log', 'rosout_loggers.txt'), 'w') as f:
        f.write(''.join(node_name(n) + '\n' for n in sorted(set(names.split()))))

    print('[facts]', stack.run('python3 /facts/graph_dump.py --out /tmp/facts/graph.json --stable 25 --timeout 240',
                               timeout=300).strip(), flush=True)
    raw = json.loads(stack.run('cat /tmp/facts/graph.json'))
    with open(os.path.join(out, 'graph.json'), 'w') as f:
        f.write(graph_text(graph_doc(raw)))

    # Parameter services stay on r2r nodes until E9-23; most module nodes have
    # none, and for those param_overrides.json is the record.
    serving = []
    for node in module_nodes():
        if f'/{node}/list_parameters' in raw['services']:
            serving.append(node)
            text = stack.run(f'ros2 param dump /{node} --no-daemon', timeout=60)
            with open(os.path.join(out, 'params', node + '.yaml'), 'w') as f:
                f.write(scrub(text))
    print(f'[facts] parameter dumps: {serving}', flush=True)

    launch_facts(stack, out)
    with open(os.path.join(out, 'log', 'startup.txt'), 'w') as f:
        f.write(startup_log(stack.logs(), stack.info))


def install(out, info, image):
    """`out` over the tree's facts, then the manifest."""
    for rel in ('launch', 'params', 'log'):
        shutil.rmtree(os.path.join(FACTS, rel), ignore_errors=True)
        shutil.copytree(os.path.join(out, rel), os.path.join(FACTS, rel))
    shutil.copy(os.path.join(out, 'graph.json'), os.path.join(FACTS, 'graph.json'))
    digests = sorted({d.split('@', 1)[1] for d in info['repo_digests']}) or [info['id']]
    with open(os.path.join(FACTS, 'PROVENANCE.txt'), 'w') as f:
        f.write(f'# graph.json, params/, launch/, log/: tools/oracle/freeze_stack.py\n'
                f'image {image}\ndigest {" ".join(digests)}\ngit_sha {info["git_sha"]}\n'
                f'date {time.strftime("%Y-%m-%d", time.gmtime())}\n')
    subprocess.run([sys.executable, os.path.join(HERE, 'facts_manifest.py')], check=True)


def check():
    facts = os.path.relpath(FACTS, REPO)
    log = os.path.join(facts, 'log')
    def changed(*specs):
        r = subprocess.run(['git', '-C', REPO, 'status', '--porcelain', '--', *specs], capture_output=True, text=True, check=True)
        return r.stdout.strip()
    # SHA256SUMS moves with whichever file moved
    hard = changed(facts, f':(exclude){log}', f':(exclude){facts}/PROVENANCE.txt', f':(exclude){facts}/SHA256SUMS')
    soft = changed(log)
    if soft:
        note = 'the start-up log differs from the frozen one (report only: its lines depend on the commit and the host)'
        print(note)
        print(subprocess.run(['git', '-C', REPO, 'diff', '--', log], capture_output=True, text=True).stdout)
        if os.environ.get('GITHUB_ACTIONS'):
            print(f'::warning title=Frozen log::{note}')
    if hard:
        print(hard)
        print(subprocess.run(['git', '-C', REPO, 'diff', '--', facts, f':(exclude){log}'], capture_output=True, text=True).stdout[:20000])
        print('the stack no longer shows the frozen facts (diff above)', file=sys.stderr)
        return 1
    print('the stack shows the frozen graph, parameters and launch line')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image')
    ap.add_argument('--work', help='where the run\'s files go (a directory Docker can mount)')
    ap.add_argument('--check', action='store_true', help='fail if anything but the log changed')
    args = ap.parse_args()
    work = tempfile.mkdtemp(prefix='ros-facts-', dir=args.work)
    stack = Stack(args.image, work)
    try:
        out = os.path.join(work, 'out')
        os.makedirs(out)
        freeze(stack, out)
    finally:
        stack.stop()
    install(out, stack.info, args.image)
    shutil.rmtree(work, ignore_errors=True)
    return check() if args.check else 0


if __name__ == '__main__':
    sys.exit(main())

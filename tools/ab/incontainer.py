#!/usr/bin/env python3
"""Helpers side.sh runs inside the image under test, as root. Stdlib only:
the image need not have ROS, only python3.

    incontainer.py net ADDR             lo multicast on (deploy/host/mower.service
                                        does `ip link set lo multicast on`) and ADDR/32
                                        on lo, so ws_bridge binds its --address as on
                                        the robot (ADDR is the robot's VPN address)
    incontainer.py forward PORT=HOST:PORT ...
                                        TCP relay from 0.0.0.0:PORT to HOST:PORT, so a
                                        published port reaches a loopback listener
    incontainer.py cpu SECONDS OUT.json  CPU of every process over a window (/proc
                                        deltas, informational only)
    incontainer.py snapshot DIR         what runs: every process's user and argv,
                                        mower_rsd's argv, and the files it reads
                                        (--params-file, the bridge's --policy)
"""

import asyncio
import fcntl
import json
import os
import pwd
import shutil
import socket
import struct
import sys
import time

SIOCGIFFLAGS, SIOCSIFFLAGS, SIOCSIFADDR, SIOCSIFNETMASK = 0x8913, 0x8914, 0x8916, 0x891C
IFF_MULTICAST = 0x1000


def ifreq_addr(name, addr):
    return struct.pack('16sH2s4s8s', name.encode(), socket.AF_INET, b'\0\0',
                       socket.inet_aton(addr), b'\0' * 8).ljust(40, b'\0')


def net(addr):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    req = struct.pack('16sH', b'lo', 0).ljust(40, b'\0')
    flags = struct.unpack_from('16sH', fcntl.ioctl(s, SIOCGIFFLAGS, req))[1]
    fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack('16sH', b'lo', flags | IFF_MULTICAST).ljust(40, b'\0'))
    # an alias label is how the old ioctl API adds a second address
    fcntl.ioctl(s, SIOCSIFADDR, ifreq_addr('lo:ab', addr))
    fcntl.ioctl(s, SIOCSIFNETMASK, ifreq_addr('lo:ab', '255.255.255.255'))
    print(f'net: lo multicast on, {addr}/32 on lo', flush=True)


async def pipe(reader, writer):
    try:
        while True:
            data = await reader.read(65536)
            if not data:
                break
            writer.write(data)
            await writer.drain()
    except (ConnectionError, OSError):
        pass
    finally:
        writer.close()


async def relay(reader, writer, host, port):
    try:
        up_reader, up_writer = await asyncio.open_connection(host, port)
    except OSError:
        writer.close()
        return
    await asyncio.gather(pipe(reader, up_writer), pipe(up_reader, writer))


async def forward(specs):
    servers = []
    for spec in specs:
        listen, target = spec.split('=')
        host, port = target.rsplit(':', 1)

        def handler(r, w, host=host, port=int(port)):
            return relay(r, w, host, port)
        servers.append(await asyncio.start_server(handler, '0.0.0.0', int(listen)))
        print(f'forward: 0.0.0.0:{listen} -> {host}:{port}', flush=True)
    await asyncio.gather(*(s.serve_forever() for s in servers))


HZ = os.sysconf('SC_CLK_TCK')
LABELS = [('mower_rsd', 'mower_rsd'), ('component_container', 'nav2_container'),
          ('twist_mux', 'twist_mux'), ('robot_state_publisher', 'robot_state_publisher'),
          ('fake_base.py', 'fake_base'), ('fake_sensors.py', 'fake_sensors'),
          ('incontainer.py', 'harness'), ('ros2 launch', 'ros2_launch')]


def proc_ticks(path):
    try:
        with open(path) as f:
            head, rest = f.read().rsplit(') ', 1)
    except (OSError, ValueError):
        return None
    fields = rest.split()
    return head.split(' (', 1)[1], int(fields[11]) + int(fields[12])


def ticks():
    procs = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        s = proc_ticks(f'/proc/{pid}/stat')
        if s is None:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            threads = len(os.listdir(f'/proc/{pid}/task'))
        except OSError:
            continue
        name = next((label for key, label in LABELS if key in cmd), s[0])
        procs[pid] = {'comm': s[0], 'name': name, 'ticks': s[1], 'threads': threads}
    return procs


def cpu(seconds, out):
    a, t0 = ticks(), time.monotonic()
    time.sleep(seconds)
    b, window = ticks(), time.monotonic() - t0
    by_name = {}
    for pid, pb in b.items():
        pa = a.get(pid)
        if pa is None or pa['comm'] != pb['comm']:
            continue
        e = by_name.setdefault(pb['name'], {'cpu_pct': 0.0, 'processes': 0, 'threads': 0})
        e['cpu_pct'] += 100.0 * (pb['ticks'] - pa['ticks']) / HZ / window
        e['processes'] += 1
        e['threads'] += pb['threads']
    for e in by_name.values():
        e['cpu_pct'] = round(e['cpu_pct'], 2)
    doc = {'window_s': round(window, 1), 'loadavg': open('/proc/loadavg').read().split()[:3],
           'cpus': os.cpu_count(),
           'by_name': dict(sorted(by_name.items(), key=lambda kv: -kv[1]['cpu_pct']))}
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)


# where an image keeps the bridge's policy when mower_rsd's argv does not say
# (E7's compat-launch reads it itself)
POLICY_PATHS = ['/opt/mower/config/ws_bridge.yaml',
                '/mower_ws/install/mower_bringup/share/mower_bringup/config/ws_bridge.yaml']


def snapshot(out):
    rows, rsd = [], None
    for pid in sorted(filter(str.isdigit, os.listdir('/proc')), key=int):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0')[:-1]]
            with open(f'/proc/{pid}/comm') as f:
                comm = f.read().strip()
            user = pwd.getpwuid(os.stat(f'/proc/{pid}').st_uid).pw_name
        except (OSError, KeyError):
            continue
        if argv:
            rows.append(f'{user:<10} {" ".join(argv)}')
        if comm == 'mower_rsd' and rsd is None:
            rsd = argv
    with open(os.path.join(out, 'procs.txt'), 'w') as f:
        f.write('USER       COMMAND\n' + '\n'.join(sorted(rows)) + '\n')
    if rsd is None:
        return
    with open(os.path.join(out, 'rsd_argv.txt'), 'w') as f:
        f.write('\n'.join(rsd) + '\n')
    files = [rsd[i + 1] for i, a in enumerate(rsd[:-1]) if a == '--params-file']
    policy = [a.split('--policy ', 1)[1].split()[0] for a in rsd if '--policy ' in a]
    files += policy or [p for p in POLICY_PATHS if os.path.exists(p)][:1]
    os.makedirs(os.path.join(out, 'rsd_files'), exist_ok=True)
    for path in files:
        try:
            shutil.copy(path, os.path.join(out, 'rsd_files', os.path.basename(path)))
        except OSError as e:
            print(f'snapshot: {path}: {e}', file=sys.stderr)


def main():
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == 'net':
        net(args[0])
    elif cmd == 'forward':
        asyncio.run(forward(args))
    elif cmd == 'cpu':
        cpu(float(args[0]), args[1])
    elif cmd == 'snapshot':
        snapshot(args[0])
    else:
        sys.exit(f'unknown command {cmd}')


if __name__ == '__main__':
    main()

"""The ``host`` block of ``/robot/telemetry``: what ``top``, ``free`` and ``df``
show, sampled from /proc and /sys.

Python twin of ``mower_rs/crates/robot_status/src/host.rs`` (the Rust
robot_status is what runs on the robot; keep the two in step). The container
runs with ``pid: host``, so /proc lists every process on the robot; disk
capacity is statvfs on the state dir, its ``bags/`` directory and ``/``.

Rates (CPU %, per-process %, disk throughput) are differences between two
samples, so they are ``None`` in the first one. Per-process CPU is % of one
core like ``top``, measured against the jiffies /proc/stat says elapsed.
"""

import os

PERIOD_S = 2.0
TOP_PROCESSES = 10

_CPU_FIELDS = ('user', 'nice', 'system', 'idle', 'iowait', 'irq', 'softirq', 'steal')


def _read(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return None


def _read_trim(path):
    text = _read(path)
    text = text.strip() if text is not None else ''
    return text or None


def _r(value, digits=1):
    return None if value is None else round(value, digits)


# ---- /proc/stat ----------------------------------------------------------------

def _cpu_times(fields):
    out = {}
    for i, name in enumerate(_CPU_FIELDS):
        try:
            out[name] = int(fields[i])
        except (IndexError, ValueError):
            out[name] = 0
    return out


def cpu_total(t):
    """guest and guest_nice are already inside user and nice."""
    return sum(t[k] for k in _CPU_FIELDS)


def parse_stat(text):
    """-> {'all': times, 'cores': [(index, times)], 'procs_running': int|None}"""
    st = {'all': _cpu_times([]), 'cores': [], 'procs_running': None}
    for line in (text or '').splitlines():
        parts = line.split()
        if not parts:
            continue
        key, rest = parts[0], parts[1:]
        if key == 'cpu':
            st['all'] = _cpu_times(rest)
        elif key.startswith('cpu') and key[3:].isdigit():
            st['cores'].append((int(key[3:]), _cpu_times(rest)))
        elif key == 'procs_running' and rest:
            try:
                st['procs_running'] = int(rest[0])
            except ValueError:
                pass
    return st


def cpu_pct(prev, cur):
    """top's %Cpu(s) between two samples; busy leaves out idle and iowait."""
    dt = cpu_total(cur) - cpu_total(prev)
    if dt <= 0:
        return None

    def pct(*keys):
        return 100.0 * max(sum(cur[k] for k in keys) - sum(prev[k] for k in keys), 0) / dt

    idle, iowait = pct('idle'), pct('iowait')
    return {
        'busy': min(max(100.0 - idle - iowait, 0.0), 100.0),
        'user': pct('user', 'nice'),
        'system': pct('system'),
        'iowait': iowait,
        'irq': pct('irq', 'softirq'),
        'steal': pct('steal'),
    }


# ---- /proc/meminfo, /proc/loadavg ----------------------------------------------

def parse_meminfo(text):
    out = {}
    for line in (text or '').splitlines():
        key, sep, value = line.partition(':')
        if not sep:
            continue
        try:
            out[key.strip()] = int(value.split()[0])
        except (IndexError, ValueError):
            pass
    return out


def mem_block(m):
    """free's arithmetic: used = total - free - buffers - cache (+ SReclaimable)."""
    total = m.get('MemTotal')
    if total is None:
        return None
    free = m.get('MemFree', 0)
    buffers = m.get('Buffers', 0)
    cache = m.get('Cached', 0) + m.get('SReclaimable', 0)
    used = max(total - free - buffers - cache, 0)
    swap_total = m.get('SwapTotal', 0)

    def mb(kb):
        return round(kb / 1024.0, 1)

    return {
        'total_mb': mb(total),
        'used_mb': mb(used),
        'cache_mb': mb(buffers + cache),
        'free_mb': mb(free),
        'avail_mb': mb(m.get('MemAvailable', free)),
        'swap_total_mb': mb(swap_total),
        'swap_used_mb': mb(max(swap_total - m.get('SwapFree', 0), 0)),
    }


def parse_loadavg(text):
    """-> (load1, load5, load15, threads)."""
    f = (text or '').split()

    def num(i):
        try:
            return float(f[i])
        except (IndexError, ValueError):
            return None

    try:
        threads = int(f[3].split('/')[1])
    except (IndexError, ValueError):
        threads = None
    return num(0), num(1), num(2), threads


# ---- processes -----------------------------------------------------------------

def parse_proc_stat(text):
    """-> dict(comm, state, ticks, threads, start) or None."""
    if not text:
        return None
    open_, close = text.find('('), text.rfind(')')
    if open_ < 0 or close < open_:
        return None
    rest = text[close + 1:].split()

    def field(n):
        return int(rest[n - 3])

    try:
        return {
            'comm': text[open_ + 1:close],
            'state': rest[0][0],
            'ticks': field(14) + field(15),
            'threads': field(20),
            'start': field(22),
        }
    except (IndexError, ValueError):
        return None


def process_label(comm, cmdline):
    """(name, ROS node or None): __node:= for the node, the script for
    Python, the untruncated binary instead of the 15-char comm."""
    args = [a.decode('utf-8', 'replace') for a in (cmdline or b'').split(b'\0') if a]
    node = next((a[len('__node:='):] for a in args if a.startswith('__node:=')), None) or None
    if not args:
        return comm, node
    exe = args[0].rsplit('/', 1)[-1]
    if exe.startswith('python'):
        script = None
        it = iter(args[1:])
        for a in it:
            if a == '-m':
                script = next(it, None)
                break
            if a.startswith('-'):
                continue
            script = a.rsplit('/', 1)[-1]
            if script.endswith('.py'):
                script = script[:-3]
            break
        return (script or comm), node
    if comm and exe.startswith(comm):
        return exe, node
    return comm, node


def parse_status_rss_kb(text):
    for line in (text or '').splitlines():
        if line.startswith('VmRSS:'):
            try:
                return int(line.split()[1])
            except (IndexError, ValueError):
                return 0
    return 0


# ---- disks ---------------------------------------------------------------------

def parse_diskstats(text):
    """-> [(name, (read_sectors, write_sectors, io_ms))]"""
    out = []
    for line in (text or '').splitlines():
        f = line.split()
        if len(f) < 14:
            continue
        try:
            out.append((f[2], (int(f[5]), int(f[9]), int(f[12]))))
        except ValueError:
            pass
    return out


def is_shown_disk(name):
    return not (name.startswith(('loop', 'ram', 'zram'))
                or (name.startswith('mmcblk') and ('boot' in name or 'rpmb' in name)))


def _unescape(s):
    return (s.replace('\\040', ' ').replace('\\011', '\t')
            .replace('\\012', '\n').replace('\\134', '\\'))


def parse_mountinfo(text):
    """-> [(mount point, fstype, source)]"""
    out = []
    for line in (text or '').splitlines():
        left, sep, right = line.partition(' - ')
        if not sep:
            continue
        l, r = left.split(), right.split()
        if len(l) < 5 or len(r) < 2:
            continue
        out.append((_unescape(l[4]), r[0], _unescape(r[1])))
    return out


def mount_for(mounts, path):
    best = None
    for m in mounts:
        mp = m[0]
        if mp == '/' or path == mp or path.startswith(mp.rstrip('/') + '/'):
            if best is None or len(mp) > len(best[0]):
                best = m
    return best


def disk_entry(role, path, frsize, blocks, bfree, bavail, fstype, source):
    """df for one filesystem: used % = used / (used + available)."""
    used = max(blocks - bfree, 0)
    denom = used + bavail
    return {
        'role': role,
        'path': path,
        'dev': source.rsplit('/', 1)[-1],
        'fstype': fstype,
        'total_gb': round(blocks * frsize / 1e9, 2),
        'used_gb': round(used * frsize / 1e9, 2),
        'avail_gb': round(bavail * frsize / 1e9, 2),
        'used_pct': round(100.0 * used / denom, 1) if denom > 0 else None,
    }


def parse_cpu_list(text):
    out = []
    for part in (text or '').replace(',', ' ').split():
        a, sep, b = part.partition('-')
        try:
            out.extend(range(int(a), int(b) + 1) if sep else [int(a)])
        except ValueError:
            pass
    return out


def _parse_hex(s):
    try:
        return int(s.strip(), 16)
    except ValueError:
        return None


def _freq_policies(root='/sys/devices/system/cpu/cpufreq'):
    try:
        names = sorted(n for n in os.listdir(root) if n.startswith('policy'))
    except OSError:
        return []

    def mhz(p, f):
        v = _read_trim(os.path.join(p, f))
        try:
            return round(float(v) / 1000.0)
        except (TypeError, ValueError):
            return None

    out = []
    for n in names:
        p = os.path.join(root, n)
        out.append({
            'cpus': parse_cpu_list(_read_trim(os.path.join(p, 'related_cpus'))),
            'cur_mhz': mhz(p, 'scaling_cur_freq'),
            'max_mhz': mhz(p, 'cpuinfo_max_freq'),
            'limit_mhz': mhz(p, 'scaling_max_freq'),
        })
    return out


def _thermal_zones(root='/sys/class/thermal'):
    out = {}
    try:
        names = sorted(n for n in os.listdir(root) if n.startswith('thermal_zone'))
    except OSError:
        return out
    for n in names:
        kind = _read_trim(os.path.join(root, n, 'type'))
        temp = _read_trim(os.path.join(root, n, 'temp'))
        try:
            if kind:
                out[kind] = round(int(temp) / 1000.0, 1)
        except (TypeError, ValueError):
            pass
    return out


# ---- sampler -------------------------------------------------------------------

class HostSampler:
    """Keeps the previous sample for the rates; ``poll(now)`` returns a fresh
    block every PERIOD_S seconds and None in between."""

    def __init__(self, state_dir):
        self._state_dir = state_dir
        self._last = None
        self._prev = None

    def poll(self, now):
        if self._last is not None and now - self._last < PERIOD_S:
            return None
        self._last = now
        return self.sample(now)

    def sample(self, now):
        stat = parse_stat(_read('/proc/stat'))
        meminfo = parse_meminfo(_read('/proc/meminfo'))
        load1, load5, load15, threads = parse_loadavg(_read('/proc/loadavg'))
        total_kb = meminfo.get('MemTotal')
        avail_kb = meminfo.get('MemAvailable')
        mem_used_pct = (round(100.0 * (total_kb - avail_kb) / total_kb, 1)
                        if total_kb and avail_kb is not None else None)
        temp = _read_trim('/sys/class/thermal/thermal_zone0/temp')
        try:
            cpu_temp_c = round(int(temp) / 1000.0, 1)
        except (TypeError, ValueError):
            cpu_temp_c = None
        try:
            boot_uptime_s = round(float(_read('/proc/uptime').split()[0]))
        except (AttributeError, IndexError, ValueError):
            boot_uptime_s = None

        prev = self._prev
        dt_s = None if prev is None else now - prev['t']

        total = cpu_pct(prev['stat']['all'], stat['all']) if prev else None
        prev_cores = dict(prev['stat']['cores']) if prev else {}
        cores = []
        for i, cur in stat['cores']:
            c = cpu_pct(prev_cores[i], cur) if i in prev_cores else None
            cores.append(None if c is None else round(c['busy'], 1))
        cpu = {k: (None if total is None else round(total[v], 1))
               for k, v in (('pct', 'busy'), ('user', 'user'), ('system', 'system'),
                            ('iowait', 'iowait'), ('irq', 'irq'), ('steal', 'steal'))}
        cpu.update({'cores': cores, 'freq': _freq_policies(), 'temps_c': _thermal_zones()})

        ncpu = max(len(stat['cores']), 1)
        elapsed = None
        if prev:
            elapsed = (cpu_total(stat['all']) - cpu_total(prev['stat']['all'])) / ncpu
            if elapsed <= 0:
                elapsed = None
        procs_now = {}
        rows = []
        try:
            pids = [int(n) for n in os.listdir('/proc') if n.isdigit()]
        except OSError:
            pids = []
        for pid in pids:
            ps = parse_proc_stat(_read(f'/proc/{pid}/stat'))
            if ps is None:
                continue
            key = (pid, ps['start'])
            procs_now[key] = ps['ticks']
            old = prev['procs'].get(key) if prev else None
            pct = (100.0 * max(ps['ticks'] - old, 0) / elapsed
                   if old is not None and elapsed else 0.0)
            rows.append((pct, pid, ps))
        rows.sort(key=lambda r: (-r[0], -r[2]['ticks']))
        procs = []
        for pct, pid, ps in rows[:TOP_PROCESSES]:
            rss_kb = parse_status_rss_kb(_read(f'/proc/{pid}/status'))
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmdline = f.read()
            except OSError:
                cmdline = b''
            name, node = process_label(ps['comm'], cmdline)
            procs.append({
                'pid': pid,
                'name': name,
                'node': node,
                'state': ps['state'],
                'cpu': round(pct, 1) if elapsed else None,
                'mem': round(100.0 * rss_kb / total_kb, 1) if total_kb else None,
                'rss_mb': round(rss_kb / 1024.0, 1),
                'threads': ps['threads'],
            })

        mounts = parse_mountinfo(_read('/proc/self/mountinfo'))
        disks, seen = [], set()
        bags = self._state_dir.rstrip('/') + '/bags'
        for role, path in (('state', self._state_dir), ('bags', bags), ('root', '/')):
            try:
                sv = os.statvfs(path)
            except OSError:
                continue
            if sv.f_blocks == 0 or (sv.f_frsize, sv.f_blocks) in seen:
                continue
            seen.add((sv.f_frsize, sv.f_blocks))
            m = mount_for(mounts, path)
            disks.append(disk_entry(role, path, sv.f_frsize, sv.f_blocks, sv.f_bfree,
                                    sv.f_bavail, m[1] if m else '', m[2] if m else ''))

        counters = {name: c for name, c in parse_diskstats(_read('/proc/diskstats'))
                    if is_shown_disk(name) and os.path.exists(f'/sys/block/{name}')}
        io = []
        for name in sorted(counters):
            cur = counters[name]
            old = prev['disks'].get(name) if prev else None
            rate = old is not None and dt_s is not None and dt_s > 0
            sys_dir = f'/sys/block/{name}'
            size = _read_trim(f'{sys_dir}/size')
            life = [v for v in (_parse_hex(s) for s in (_read(f'{sys_dir}/device/life_time') or '').split())
                    if v is not None]
            pre_eol = _read_trim(f'{sys_dir}/device/pre_eol_info')
            io.append({
                'dev': name,
                'model': _read_trim(f'{sys_dir}/device/model') or _read_trim(f'{sys_dir}/device/name'),
                'size_gb': round(int(size) * 512 / 1e9, 1) if size and size.isdigit() else None,
                'read_kbs': round(max(cur[0] - old[0], 0) * 512 / 1024 / dt_s, 1) if rate else None,
                'write_kbs': round(max(cur[1] - old[1], 0) * 512 / 1024 / dt_s, 1) if rate else None,
                'util_pct': round(min(100.0 * max(cur[2] - old[2], 0) / (dt_s * 1000.0), 100.0), 1) if rate else None,
                'life_time': life or None,
                'pre_eol': _parse_hex(pre_eol) if pre_eol else None,
            })

        doc = {
            'load1': load1,
            'load5': load5,
            'load15': load15,
            'mem_used_pct': mem_used_pct,
            'cpu_temp_c': cpu_temp_c,
            'boot_uptime_s': boot_uptime_s,
            'sample_s': _r(dt_s, 2),
            'cpu': cpu,
            'mem': mem_block(meminfo),
            'disks': disks,
            'io': io,
            'tasks': {'procs': len(rows), 'threads': threads, 'running': stat['procs_running']},
            'procs': procs,
        }
        self._prev = {'t': now, 'stat': stat, 'procs': procs_now, 'disks': counters}
        return doc

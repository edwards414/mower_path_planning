#!/usr/bin/env python3
"""The Docker VM's CPU use every 2 s (/proc/stat is not namespaced), and a
stall canary: an unthrottled 20 ms tick whose lateness over 100 ms is logged
as {"stall_s": ...}. A gate closure or a late command that lines up with a
canary stall is the VM (Docker Desktop, another container starting), not
the throttled stack. Observer sidecar; development only.

    vmload.py --out /tmp/obs/vmload.jsonl
"""
import argparse
import json
import os
import threading
import time


def sample():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:9]]
    return sum(v), v[3] + v[4]


def canary(out, lock):
    last = time.monotonic()
    while True:
        time.sleep(0.02)
        now = time.monotonic()
        late = now - last - 0.02
        if late > 0.1:
            with lock:
                out.write(json.dumps({'t': round(time.time(), 3), 'stall_s': round(late, 3)}) + '\n')
        last = now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    out = open(args.out, 'a', buffering=1)
    lock = threading.Lock()
    threading.Thread(target=canary, args=(out, lock), daemon=True).start()
    a = sample()
    while True:
        time.sleep(2.0)
        b = sample()
        busy = 100.0 * (1 - (b[1] - a[1]) / max(1, b[0] - a[0]))
        load = open('/proc/loadavg').read().split()[0]
        with lock:
            out.write(json.dumps({'t': round(time.time(), 1), 'busy_pct': round(busy, 1), 'load1': float(load)}) + '\n')
        a = b


if __name__ == '__main__':
    main()

#!/usr/bin/python3
"""Keep automatically restarted ROS containers behind network clock sync."""

import argparse
import math
import subprocess
import sys
import time


def is_synchronized():
    try:
        result = subprocess.run(
            ['timedatectl', 'show', '--property=NTPSynchronized', '--value'],
            capture_output=True, text=True, timeout=3, check=False,
        )
        return result.returncode == 0 and result.stdout.strip() == 'yes'
    except (OSError, subprocess.TimeoutExpired):
        return False


def wait_for_sync(timeout=120.0, stable_for=3.0, interval=0.5, *,
                  synchronized=is_synchronized, wall=time.time,
                  monotonic=time.monotonic, sleep=time.sleep, emit=print):
    """Require synchronization plus a stable wall/monotonic clock relationship.

    A successful status check alone is insufficient if ntpd steps the clock
    while this process is waiting. All deadlines use the monotonic clock.
    """
    for value in (timeout, stable_for, interval):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('timeouts and intervals must be positive and finite')
    deadline = monotonic() + timeout
    previous_monotonic = monotonic()
    previous_wall = wall()
    stable_since = None
    emit('Waiting for network clock synchronization before starting Docker')
    while monotonic() < deadline:
        synced = synchronized()
        now = monotonic()
        wall_now = wall()
        step = (wall_now - previous_wall) - (now - previous_monotonic)
        previous_wall, previous_monotonic = wall_now, now
        if abs(step) > 0.5:
            stable_since = None
            emit(f'Clock stepped by {step:+.3f}s; restarting stability check')
        elif synced:
            if stable_since is None:
                stable_since = now
            if now < deadline and now - stable_since >= stable_for:
                emit(f'Network clock synchronized and stable for {stable_for:g}s')
                return True
        else:
            stable_since = None
        remaining = deadline - monotonic()
        if remaining > 0:
            sleep(min(interval, remaining))
    emit('Clock did not synchronize in time; refusing to start ROS containers')
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--stable-for', type=float, default=3.0)
    args = parser.parse_args()
    try:
        return 0 if wait_for_sync(args.timeout, args.stable_for) else 1
    except ValueError as error:
        parser.error(str(error))


if __name__ == '__main__':
    sys.exit(main())

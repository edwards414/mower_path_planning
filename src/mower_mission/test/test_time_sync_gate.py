"""Exercise the boot clock gate without changing the host clock or services."""

import importlib.util
from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[3] / 'deploy/host/mower-wait-time-sync.py'
SPEC = importlib.util.spec_from_file_location('mower_time_sync_gate', SCRIPT)
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


class Clock:
    def __init__(self, offset=lambda t: 0):
        self.now = 0.0
        self.offset = offset

    def monotonic(self):
        return self.now

    def wall(self):
        return 1_000_000 + self.now + self.offset(self.now)

    def sleep(self, seconds):
        self.now += seconds

    def wait(self, synchronized, **kwargs):
        return gate.wait_for_sync(
            synchronized=synchronized, wall=self.wall, monotonic=self.monotonic,
            sleep=self.sleep, emit=lambda _: None, **kwargs,
        )


def test_requires_sync_and_stability_not_only_a_plausible_date():
    clock = Clock()
    assert clock.wait(lambda: clock.now >= 2, timeout=10, stable_for=3)
    assert clock.now == 5


@pytest.mark.parametrize('step', [42_226_052.6, -42_226_052.6])
def test_forward_or_backward_step_restarts_stability_check(step):
    clock = Clock(lambda t: step if t >= 2 else 0)
    assert clock.wait(lambda: True, timeout=10, stable_for=3)
    # The step at t=2 invalidates the initial two seconds of stable status.
    assert clock.now == 5.5


def test_losing_sync_restarts_stability_check():
    clock = Clock()
    assert clock.wait(lambda: not (2 <= clock.now < 3), timeout=10, stable_for=3)
    assert clock.now == 6


def test_timeout_is_monotonic_even_when_wall_clock_jumps():
    clock = Clock(lambda t: -42_226_052.6 if t >= 1 else 0)
    assert not clock.wait(lambda: False, timeout=4, stable_for=3)
    assert clock.now == 4


def test_failed_status_command_does_not_allow_start(monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError('timedatectl')
    monkeypatch.setattr(subprocess, 'run', missing)
    assert not gate.is_synchronized()


@pytest.mark.parametrize('stdout,returncode,expected', [
    ('yes\n', 0, True), ('no\n', 0, False), ('yes\n', 1, False), ('', 0, False),
])
def test_checks_both_status_and_exit_code(monkeypatch, stdout, returncode, expected):
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw:
        subprocess.CompletedProcess([], returncode, stdout, ''))
    assert gate.is_synchronized() is expected

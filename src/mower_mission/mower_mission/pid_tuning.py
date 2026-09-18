"""Wheel-speed PID auto-tune maths: step-response identification + SIMC PI.

ROS-free so it can be unit tested. The node (pid_autotune_node.py) feeds it
the (t, pwm counts, rpm) samples the STM32 reports in 0x85 while the wheel is
driven open loop, gets a first-order-plus-dead-time (FOPDT) model back and
turns that into PI gains for the firmware's PID (error in rpm, output in PWM
counts, 20 ms tick).

Model:  y(t) = y0 + K * du * (1 - exp(-(t - t_step - L) / tau))  for t > t_step + L

Tuning (Skogestad SIMC, PI form):
    tau_c = max(L, tau_c_factor * tau)
    Kp    = tau / (K * (tau_c + L))
    Ti    = min(tau, 4 * (tau_c + L))
    Ki    = Kp / Ti
    Kd    = 0
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Iterable, Sequence


class TuningError(ValueError):
    """The recorded response cannot be turned into gains (with a user-facing reason)."""


@dataclass
class Sample:
    t: float    # s, any monotonic base
    pwm: float  # PWM counts actually applied (0x85 pid_output)
    rpm: float  # measured wheel speed (0x85 measured_rpm)


@dataclass
class FopdtModel:
    gain: float      # rpm per PWM count
    tau: float       # s
    delay: float     # s
    y0: float        # rpm before the step
    y_ss: float      # rpm after the step settled
    u0: float        # counts before the step
    u1: float        # counts after the step
    fit_r2: float    # 1 - SSE/SST over the post-step window (informational)
    fit_rmse: float = 0.0  # rpm, residual of the fit

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Gains:
    kp: float
    ki: float
    kd: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class StepMetrics:
    target: float          # rpm
    overshoot_pct: float   # (peak - target) / target * 100, 0 if none
    settle_s: float | None  # first time after which |y - target| <= band, None if never
    ss_error: float        # mean of the last window minus target, rpm
    rise_s: float | None   # 10 % -> 90 % of target

    def to_dict(self) -> dict:
        return asdict(self)


# Physical plausibility of a geared brushed DC wheel motor read at 50 ms.
GAIN_RANGE = (0.02, 5.0)     # rpm / count (58 rpm at 200 counts is 0.29)
TAU_RANGE = (0.02, 3.0)      # s
DELAY_MAX = 0.6              # s
MIN_RESPONSE_RPM = 2.0
MAX_FIT_RMSE_FRACTION = 0.10  # of the response amplitude, plus the pre-step noise


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values)


def _window(samples: Sequence[Sample], t_from: float, t_to: float) -> list[Sample]:
    return [s for s in samples if t_from <= s.t <= t_to]


def fit_fopdt(samples: Iterable[Sample], t_step: float, *, t_end: float | None = None,
              pre_window_s: float = 0.5, settle_window_s: float = 0.8) -> FopdtModel:
    """Fit a FOPDT model to one step in the applied PWM.

    ``samples`` must cover ``pre_window_s`` before ``t_step`` and enough after
    it for the speed to (nearly) settle; ``t_end`` cuts the record where the
    step stopped being held (anything after, e.g. the ramp down, is ignored).
    The step is located from the samples themselves, ``t_step`` only says
    where to look. The steady-state speed is a fitted parameter, not the last
    sample, so a slightly unsettled response still gives the right gain; one
    that is clearly still rising is rejected.
    """
    data = sorted((s for s in samples if t_end is None or s.t <= t_end), key=lambda s: s.t)
    if len(data) < 12:
        raise TuningError('not enough samples')
    pre = _window(data, t_step - pre_window_s, t_step - 1e-9)
    post = [s for s in data if s.t >= t_step]
    if len(pre) < 3 or len(post) < 8:
        raise TuningError('step is not inside the recorded window')

    t_end = post[-1].t
    tail = _window(post, t_end - settle_window_s, t_end)
    if len(tail) < 4:
        raise TuningError('response too short to judge steady state')

    u0 = _mean([s.pwm for s in pre])
    u1 = _mean([s.pwm for s in tail])
    du = u1 - u0
    if abs(du) < 1.0:
        raise TuningError('PWM did not change across the step')
    y0 = _mean([s.rpm for s in pre])
    pre_noise = math.sqrt(_mean([(s.rpm - y0) ** 2 for s in pre]))
    dy_tail = _mean([s.rpm for s in tail]) - y0
    if abs(dy_tail) < max(MIN_RESPONSE_RPM, 3.0 * pre_noise):
        raise TuningError('wheel speed did not respond to the step (no encoder signal, wheel blocked, or motor not driven)')
    if dy_tail / du < 0:
        raise TuningError('speed went the wrong way for the PWM step (encoder or motor sign)')

    # When did the PWM actually change? The first post sample whose pwm is
    # past the midpoint marks the step; the delay is measured from there.
    mid = u0 + 0.5 * du
    t0 = post[0].t
    for s in post:
        if (s.pwm - mid) * du >= 0:
            t0 = s.t
            break

    # Least squares over a (tau, delay) grid, coarse then refined; for each
    # pair the response amplitude dy is linear and solved in closed form.
    # Cheap (a few hundred samples) and immune to noisy 28 % / 63 % points.
    ts = [s.t - t0 for s in post]
    ys = [s.rpm - y0 for s in post]

    def solve(tau: float, delay: float) -> tuple[float, float]:
        fs = [0.0 if t <= delay else 1.0 - math.exp(-(t - delay) / tau) for t in ts]
        ff = sum(f * f for f in fs)
        if ff <= 0:
            return float('inf'), 0.0
        dy = sum(f * y for f, y in zip(fs, ys)) / ff
        err = sum((y - dy * f) ** 2 for f, y in zip(fs, ys))
        return err, dy

    best = (float('inf'), TAU_RANGE[0], 0.0, 0.0)
    taus = [TAU_RANGE[0] * (TAU_RANGE[1] / TAU_RANGE[0]) ** (i / 59) for i in range(60)]
    delays = [i * 0.01 for i in range(int(DELAY_MAX / 0.01) + 1)]
    for tau in taus:
        for delay in delays:
            e, dy = solve(tau, delay)
            if e < best[0]:
                best = (e, tau, delay, dy)
    for _ in range(2):  # refine around the coarse optimum
        _, tau, delay, _ = best
        for tau_c in [tau * (1 + 0.05 * k) for k in range(-6, 7)]:
            for delay_c in [max(0.0, delay + 0.002 * k) for k in range(-5, 6)]:
                e, dy = solve(tau_c, delay_c)
                if e < best[0]:
                    best = (e, tau_c, delay_c, dy)
    err, tau, delay, dy = best
    if dy / du < 0:
        raise TuningError('speed went the wrong way for the PWM step (encoder or motor sign)')

    # The record has to reach most of the way to steady state, otherwise the
    # gain is an extrapolation: ask for a longer hold instead of guessing.
    reached = 1.0 - math.exp(-max(0.0, ts[-1] - delay) / tau)
    if reached < 0.9:
        raise TuningError(f'speed was still rising at the end of the step (time constant ~{tau:.2f} s); hold the step longer')

    rmse = math.sqrt(err / len(ys))
    mean_y = _mean(ys)
    sst = sum((y - mean_y) ** 2 for y in ys) or 1e-9
    r2 = 1.0 - err / sst
    model = FopdtModel(gain=dy / du, tau=tau, delay=delay, y0=y0, y_ss=y0 + dy, u0=u0, u1=u1,
                       fit_r2=r2, fit_rmse=rmse)
    check_model(model, noise=pre_noise)
    return model


def check_model(m: FopdtModel, *, noise: float = 0.0) -> None:
    if not (GAIN_RANGE[0] <= m.gain <= GAIN_RANGE[1]):
        raise TuningError(f'plant gain {m.gain:.3f} rpm/count is outside {GAIN_RANGE}')
    if not (TAU_RANGE[0] <= m.tau <= TAU_RANGE[1]):
        raise TuningError(f'time constant {m.tau:.3f} s is outside {TAU_RANGE}')
    if m.delay > DELAY_MAX:
        raise TuningError(f'dead time {m.delay:.3f} s is implausibly long')
    allowed = MAX_FIT_RMSE_FRACTION * abs(m.y_ss - m.y0) + 2.0 * noise + 0.3
    if m.fit_rmse > allowed:
        raise TuningError(f'step response does not look first order (fit residual {m.fit_rmse:.2f} rpm)')


def simc_pi(m: FopdtModel, *, tau_c_factor: float = 1.0, kp_max: float = 50.0,
            ki_max: float = 100.0) -> Gains:
    """SIMC PI gains for the firmware loop (counts per rpm, counts per rpm*s)."""
    tau_c = max(m.delay, tau_c_factor * m.tau)
    if tau_c + m.delay <= 0:
        raise TuningError('degenerate model')
    kp = m.tau / (m.gain * (tau_c + m.delay))
    ti = min(m.tau, 4.0 * (tau_c + m.delay))
    ki = kp / ti
    kp = min(kp, kp_max)
    ki = min(ki, ki_max)
    return Gains(kp=round(kp, 4), ki=round(ki, 4), kd=0.0)


def step_metrics(samples: Iterable[Sample], t_step: float, target: float, *,
                 t_end: float | None = None, band_pct: float = 5.0,
                 settle_window_s: float = 0.8) -> StepMetrics:
    """Closed-loop quality of a 0 -> target step (samples in [t_step, t_end])."""
    post = sorted((s for s in samples if s.t >= t_step and (t_end is None or s.t <= t_end)),
                  key=lambda s: s.t)
    if len(post) < 8 or target <= 0:
        raise TuningError('not enough closed-loop samples')
    band = abs(target) * band_pct / 100.0
    peak = max(s.rpm for s in post)
    overshoot = max(0.0, (peak - target) / target * 100.0)
    settle = None
    for i, s in enumerate(post):
        if all(abs(x.rpm - target) <= band for x in post[i:]):
            settle = s.t - t_step
            break
    t_end = post[-1].t
    tail = _window(post, t_end - settle_window_s, t_end)
    ss_error = _mean([s.rpm for s in tail]) - target
    t10 = next((s.t for s in post if s.rpm >= 0.1 * target), None)
    t90 = next((s.t for s in post if s.rpm >= 0.9 * target), None)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    return StepMetrics(target=target, overshoot_pct=overshoot, settle_s=settle,
                       ss_error=ss_error, rise_s=rise)


def simulate_closed_loop(m: FopdtModel, g: Gains, target: float, *, dt: float = 0.02,
                         duration: float = 3.0, output_max: float = 200.0,
                         integral_max: float = 200.0) -> list[Sample]:
    """Discrete PI on a FOPDT plant, mirroring the firmware loop (used by tests
    and to sanity-check gains before they are sent to the robot)."""
    n = int(duration / dt)
    delay_steps = int(round(m.delay / dt))
    u_hist = [0.0] * (delay_steps + 1)
    y = 0.0
    integral = 0.0
    out = []
    for i in range(n):
        t = i * dt
        err = target - y
        integral = max(-integral_max, min(integral_max, integral + err * dt))
        u = g.kp * err + g.ki * integral
        u = max(-output_max, min(output_max, u))
        u_hist.append(u)
        u_delayed = u_hist[-1 - delay_steps]
        # first-order plant, exact discretisation
        a = math.exp(-dt / m.tau)
        y = a * y + (1 - a) * m.gain * u_delayed
        out.append(Sample(t=t, pwm=u, rpm=y))
    return out

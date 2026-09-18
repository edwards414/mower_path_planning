"""ROS-free tests for the wheel PID auto-tune maths (pid_tuning.py)."""

import math
import random

import pytest

from mower_mission import pid_tuning as pt


def open_loop_response(gain, tau, delay, u0, u1, *, dt=0.05, pre=0.5, post=2.5,
                       noise=0.0, seed=1):
    """Simulated STM32 0x85 stream for a u0 -> u1 PWM step at t=pre."""
    rng = random.Random(seed)
    y = gain * u0
    samples = []
    t = 0.0
    n = int((pre + post) / dt)
    hist = [u0] * (int(round(delay / dt)) + 1)
    for i in range(n):
        t = i * dt
        u = u1 if t >= pre else u0
        hist.append(u)
        u_d = hist[-1 - int(round(delay / dt))]
        a = math.exp(-dt / tau)
        y = a * y + (1 - a) * gain * u_d
        samples.append(pt.Sample(t=t, pwm=u, rpm=y + rng.gauss(0.0, noise)))
    return samples


def test_fit_recovers_plant_within_tolerance():
    # 58 rpm at 200 counts -> 0.29 rpm/count, typical geared DC motor
    s = open_loop_response(0.29, 0.25, 0.05, 80, 140, noise=0.3)
    m = pt.fit_fopdt(s, t_step=0.5)
    assert abs(m.gain - 0.29) / 0.29 < 0.10
    assert abs(m.tau - 0.25) / 0.25 < 0.15
    assert m.delay <= 0.12
    assert m.fit_rmse < 0.5


def test_fit_ignores_samples_after_t_end():
    s = open_loop_response(0.29, 0.25, 0.05, 80, 140, noise=0.2)
    t_end = s[-1].t
    # what the node records after the hold: PWM back to zero, speed decaying
    tail = [pt.Sample(t=t_end + 0.05 * i, pwm=0, rpm=40.6 * math.exp(-0.05 * i / 0.25)) for i in range(1, 8)]
    with pytest.raises(pt.TuningError):
        pt.fit_fopdt(s + tail, t_step=0.5)
    m = pt.fit_fopdt(s + tail, t_step=0.5, t_end=t_end)
    assert abs(m.gain - 0.29) / 0.29 < 0.10
    met = pt.step_metrics(s + tail, t_step=0.5, target=40.6, t_end=t_end)
    assert met.settle_s is not None


def test_fit_rejects_no_response():
    s = [pt.Sample(t=i * 0.05, pwm=(140 if i * 0.05 >= 0.5 else 80), rpm=0.0) for i in range(60)]
    with pytest.raises(pt.TuningError, match='did not respond'):
        pt.fit_fopdt(s, t_step=0.5)


def test_fit_rejects_wrong_sign():
    s = open_loop_response(0.29, 0.25, 0.05, 80, 140)
    flipped = [pt.Sample(t=x.t, pwm=x.pwm, rpm=-x.rpm) for x in s]
    with pytest.raises(pt.TuningError, match='wrong way'):
        pt.fit_fopdt(flipped, t_step=0.5)


def test_fit_rejects_missing_step():
    s = open_loop_response(0.29, 0.25, 0.05, 80, 140)
    with pytest.raises(pt.TuningError):
        pt.fit_fopdt(s, t_step=2.9)


def test_fit_rejects_unsettled_response():
    # tau 1.2 s needs ~4 s to settle; a 2.5 s hold must not silently extrapolate
    s = open_loop_response(0.29, 1.2, 0.2, 80, 140, noise=0.2)
    with pytest.raises(pt.TuningError, match='still rising'):
        pt.fit_fopdt(s, t_step=0.5)


def test_fit_tolerates_slightly_unsettled_response():
    # tau 0.9 s reaches 92 % in 2.5 s; the fitted steady state is still right
    s = open_loop_response(0.29, 0.9, 0.1, 80, 140, noise=0.2)
    m = pt.fit_fopdt(s, t_step=0.5)
    assert abs(m.gain - 0.29) / 0.29 < 0.10
    assert abs(m.tau - 0.9) / 0.9 < 0.20


@pytest.mark.parametrize('tau,delay', [(0.1, 0.0), (0.25, 0.05), (0.6, 0.1), (0.9, 0.15)])
def test_simc_gains_give_a_well_damped_loop(tau, delay):
    s = open_loop_response(0.29, tau, delay, 80, 140, noise=0.2)
    m = pt.fit_fopdt(s, t_step=0.5)
    g = pt.simc_pi(m)
    assert g.kp > 0 and g.ki > 0 and g.kd == 0.0
    truth = pt.FopdtModel(gain=0.29, tau=tau, delay=delay, y0=0, y_ss=0, u0=0, u1=0, fit_r2=1)
    sim = pt.simulate_closed_loop(truth, g, target=29.0, duration=4.0)
    met = pt.step_metrics(sim, t_step=0.0, target=29.0)
    assert met.overshoot_pct < 25.0
    assert met.settle_s is not None and met.settle_s < 3.0
    assert abs(met.ss_error) < 0.5


def test_step_metrics_on_ideal_response():
    s = [pt.Sample(t=i * 0.05, pwm=100, rpm=(29.0 if i >= 4 else 0.0)) for i in range(60)]
    met = pt.step_metrics(s, t_step=0.0, target=29.0)
    assert met.overshoot_pct == 0.0
    assert met.settle_s == pytest.approx(0.2)
    assert met.ss_error == 0.0


def test_step_metrics_detects_overshoot_and_never_settling():
    s = [pt.Sample(t=i * 0.05, pwm=100, rpm=29.0 * (1.3 if i % 2 else 0.7)) for i in range(60)]
    met = pt.step_metrics(s, t_step=0.0, target=29.0)
    assert met.overshoot_pct == pytest.approx(30.0)
    assert met.settle_s is None


def test_fit_of_a_sample_fast_wheel_stays_in_range():
    # the real base (2026-09-19 run): the speed settles within one 50 ms sample,
    # the refinement must not walk tau below TAU_RANGE and then reject it
    s = open_loop_response(0.3, 0.03, 0.04, 80, 140, dt=0.05, noise=0.3)
    m = pt.fit_fopdt(s, t_step=0.5)
    assert pt.TAU_RANGE[0] <= m.tau <= 0.12
    assert abs(m.gain - 0.3) / 0.3 < 0.1


def test_simc_designs_against_a_tau_floor():
    # the real base: tau at the fit's lower bound, 44 ms dead time, K 0.3
    m = pt.FopdtModel(gain=0.296, tau=0.0207, delay=0.044, y0=21, y_ss=39, u0=80, u1=140, fit_r2=0.6)
    g = pt.simc_pi(m)
    assert g.kp / g.ki == pytest.approx(pt.TAU_DESIGN_MIN, rel=1e-3)  # Ti = tau floor (gains are rounded)
    assert 1.0 < g.kp < 3.0 and 20 < g.ki < 60
    sim = pt.simulate_closed_loop(pt.FopdtModel(gain=0.296, tau=0.04, delay=0.04, y0=0, y_ss=0, u0=0, u1=0, fit_r2=1),
                                  g, target=29.0, duration=3.0)
    met = pt.step_metrics(sim, t_step=0.0, target=29.0)
    assert met.overshoot_pct < 25.0 and met.settle_s is not None and met.settle_s < 1.0


def test_gains_are_clamped():
    m = pt.FopdtModel(gain=0.02, tau=3.0, delay=0.0, y0=0, y_ss=0, u0=0, u1=0, fit_r2=1)
    g = pt.simc_pi(m, tau_c_factor=0.01, kp_max=10.0, ki_max=5.0)
    assert g.kp == 10.0 and g.ki == 5.0


def test_rosbridge_exposes_autotune_service_and_status():
    from pathlib import Path
    text = (Path(__file__).resolve().parents[1].parent / 'mower_bringup/config/rosbridge_params.yaml').read_text()
    websocket, rosapi = text.split('rosapi:')[0], text.split('rosapi:')[1]
    assert "'/pid_autotune/status'" in websocket.split('topics_sub_glob')[1].split('\n')[0]
    assert "'/pid_autotune'" in websocket.split('services_glob')[1].split('\n')[0]
    assert "'/pid_autotune/status'" in rosapi.split('topics_glob')[1].split('\n')[0]

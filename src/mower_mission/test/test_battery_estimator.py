import math

from mower_mission.battery_estimator import (
    BatteryEstimator,
    ocv_fraction,
    STATUS_CHARGING,
    STATUS_DISCHARGING,
    STATUS_FULL,
    STATUS_NOT_CHARGING,
)
import pytest


def test_ocv_table_endpoints_and_interpolation():
    assert ocv_fraction(2.5) == pytest.approx(0.0)
    assert ocv_fraction(3.0) == pytest.approx(0.0)
    assert ocv_fraction(4.2) == pytest.approx(1.0)
    assert ocv_fraction(4.5) == pytest.approx(1.0)
    # midway between (3.72, 0.40) and (3.78, 0.50)
    assert ocv_fraction(3.75) == pytest.approx(0.45)
    assert ocv_fraction(math.nan) == pytest.approx(0.0)


def test_first_sample_is_taken_as_is():
    est = BatteryEstimator(cell_count=6, filter_tau_s=20.0)
    frac = est.update(0.0, 6 * 3.85)
    assert frac == pytest.approx(0.60)
    assert est.percentage == pytest.approx(60.0)
    assert est.status == STATUS_DISCHARGING
    assert est.voltage_v == pytest.approx(23.1)


def test_ignores_invalid_voltage():
    est = BatteryEstimator()
    assert math.isnan(est.update(0.0, 0.0))
    assert math.isnan(est.update(1.0, math.nan))
    est.update(2.0, 24.0)
    assert not math.isnan(est.fraction)


def test_short_sag_barely_moves_filtered_estimate():
    est = BatteryEstimator(cell_count=6, filter_tau_s=20.0)
    est.update(0.0, 6 * 3.85)  # 60 %
    # 2 s wheel stall pulls the pack down by 1.2 V
    t = 0.0
    for _ in range(10):
        t += 0.2
        est.update(t, 6 * 3.65)
    assert est.fraction > 0.55
    assert est.voltage_v > 6 * 3.80


def test_sustained_drop_lowers_estimate_and_rebound_is_rate_limited():
    est = BatteryEstimator(cell_count=6, filter_tau_s=1.0,
                           recovery_rate_per_s=0.01 / 60.0)
    est.update(0.0, 6 * 3.85)
    t = 0.0
    for _ in range(60):
        t += 1.0
        est.update(t, 6 * 3.66)  # 30 % for a minute
    assert est.fraction == pytest.approx(0.30, abs=0.01)
    # voltage rebounds instantly; SOC may only climb 1 %/min
    for _ in range(60):
        t += 1.0
        est.update(t, 6 * 3.85)
    assert est.fraction == pytest.approx(0.31, abs=0.005)
    assert est.status == STATUS_DISCHARGING


def test_voltage_only_charging_ramps_from_last_estimate():
    """On the charger the terminal voltage is the CV: ramp, never look up."""
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           charge_rate_per_s=0.005 / 60.0)
    est.update(0.0, 6 * 3.72)  # 40 %
    est.update(1.0, 25.6, charger_present=True)
    assert est.status == STATUS_CHARGING
    # the 25.6 V terminal voltage would read as 100 % on the OCV table
    assert est.fraction == pytest.approx(0.40, abs=0.001)
    est.update(1.0 + 20 * 60, 25.6, charger_present=True)
    assert est.fraction == pytest.approx(0.50, abs=0.001)
    est.update(1.0 + 10 * 3600, 25.6, charger_present=True)
    assert est.fraction == pytest.approx(0.99)
    assert est.status == STATUS_CHARGING  # cannot prove full without current
    assert math.isnan(est.current_a)


def test_charger_removed_reanchors_to_ocv_after_settling():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           post_charge_settle_s=300.0)
    est.update(0.0, 6 * 3.72)  # 40 %
    est.update(1.0, 25.6, charger_present=True)
    est.update(3600.0, 25.6, charger_present=True)  # ramped to 70 %
    assert est.fraction == pytest.approx(0.70, abs=0.001)
    # unplugged: the pack relaxes from the CV towards 4.0 V/cell (80 %)
    est.update(3601.0, 25.2)
    assert est.status == STATUS_DISCHARGING
    assert est.fraction == pytest.approx(0.70, abs=0.001)  # inflated V ignored
    est.update(3601.0 + 200.0, 24.2)
    assert est.fraction == pytest.approx(0.70, abs=0.001)  # still settling
    est.update(3601.0 + 300.0, 24.04)
    assert est.fraction == pytest.approx(0.8, abs=0.01)   # snapped once
    est.update(3601.0 + 301.0, 24.04)
    assert est.fraction == pytest.approx(0.8, abs=0.01)


def test_first_sample_on_charger_starts_below_full():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0)
    est.update(0.0, 25.6, charger_present=True)
    assert est.fraction == pytest.approx(0.99)
    assert est.status == STATUS_CHARGING


def test_full_after_tail_current_held_at_cv():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 3.72)
    est.update(1.0, 6 * 4.20, charger_present=True, current_a=0.15)
    assert est.status == STATUS_CHARGING
    est.update(31.0, 6 * 4.20, charger_present=True, current_a=0.15)
    assert est.fraction < 1.0
    est.update(61.0, 6 * 4.20, charger_present=True, current_a=0.15)
    assert est.fraction == pytest.approx(1.0)
    assert est.status == STATUS_FULL
    # parked on the charger with the current gone: still full
    est.update(120.0, 6 * 4.20, charger_present=True, current_a=0.0)
    assert est.status == STATUS_FULL
    assert est.fraction == pytest.approx(1.0)
    # tail interrupted by a real top-up -> charging again, timer restarts
    est.update(121.0, 6 * 4.20, charger_present=True, current_a=0.5)
    assert est.status == STATUS_CHARGING


def test_charger_present_without_current_below_cv_is_not_charging():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0)
    est.update(0.0, 6 * 3.85)
    est.update(1.0, 6 * 3.90, charger_present=True, current_a=0.0)
    assert est.status == STATUS_NOT_CHARGING
    assert est.fraction == pytest.approx(0.60)


def test_unplugged_mid_charge_with_current_is_not_full():
    """Adapter pulled mid-charge: charger_present drops, current 0."""
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0, capacity_ah=20.0)
    est.update(0.0, 6 * 3.72)
    est.update(1.0, 25.6, charger_present=True, current_a=2.0)
    assert est.status == STATUS_CHARGING
    for t in (2.0, 30.0, 61.0, 120.0):
        est.update(t, 6 * 3.72, charger_present=False, current_a=0.0)
    assert est.status == STATUS_DISCHARGING
    assert est.fraction == pytest.approx(0.40, abs=0.01)


def test_coulomb_counting_discharge_and_rest_reanchor():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0, capacity_ah=10.0,
                           rest_current_a=0.1, rest_hold_s=300.0)
    est.update(0.0, 6 * 4.00)  # 80 %
    # 5 A for 30 min = 2.5 Ah = 25 %; voltage sagging under load is ignored
    t = 0.0
    for _ in range(30):
        t += 60.0
        est.update(t, 6 * 3.70, current_a=-5.0)
    assert est.fraction == pytest.approx(0.55, abs=0.005)
    assert est.status == STATUS_DISCHARGING
    # rest: after 5 min at ~0 A the OCV (3.80 V -> ~53 %) corrects the drift
    for _ in range(5):
        t += 60.0
        est.update(t, 6 * 3.80, current_a=0.02)
    assert est.fraction == pytest.approx(0.55, abs=0.005)
    t += 60.0
    est.update(t, 6 * 3.80, current_a=0.02)
    assert est.fraction == pytest.approx(ocv_fraction(3.80), abs=0.005)


def test_coulomb_counting_charge_then_full():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0, capacity_ah=10.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 3.72)  # 40 %
    t = 0.0
    for _ in range(60):
        t += 60.0
        est.update(t, 25.2, charger_present=True, current_a=2.0)  # 2 Ah = +20 %
    assert est.fraction == pytest.approx(0.60, abs=0.005)
    assert est.status == STATUS_CHARGING
    # long CV tail: integration would overshoot, cap holds at 99 %
    for _ in range(300):
        t += 60.0
        est.update(t, 25.2, charger_present=True, current_a=1.0)
    assert est.fraction == pytest.approx(0.99)
    est.update(t + 1.0, 25.2, charger_present=True, current_a=0.1)
    est.update(t + 61.0, 25.2, charger_present=True, current_a=0.1)
    assert est.status == STATUS_FULL
    assert est.fraction == pytest.approx(1.0)


def test_single_cell_aon_battery():
    est = BatteryEstimator(cell_count=1, filter_tau_s=0.0)
    assert est.update(0.0, 3.78) == pytest.approx(0.5)
    assert est.cell_voltage_v == pytest.approx(3.78)

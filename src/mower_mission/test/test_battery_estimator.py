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


def test_charging_only_rises_and_caps_below_full_until_tail_current():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 3.72)  # 40 %
    est.update(1.0, 6 * 3.60, charger_online=True, charging=True,
               charge_current_a=3.0)
    # a dip in terminal voltage must not lower the estimate while charging
    assert est.fraction == pytest.approx(0.40)
    assert est.status == STATUS_CHARGING
    est.update(2.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=1.0)
    assert est.fraction == pytest.approx(0.99)
    assert est.charge_current_a == pytest.approx(1.0)


def test_full_after_tail_current_held():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=0.15)
    assert est.status == STATUS_CHARGING
    est.update(30.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=0.15)
    assert est.fraction == pytest.approx(0.99)
    est.update(61.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=0.15)
    assert est.fraction == pytest.approx(1.0)
    assert est.status == STATUS_FULL
    # tail interrupted -> timer restarts
    est.update(62.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=0.5)
    assert est.status == STATUS_CHARGING


def test_tail_below_charging_threshold_still_completes_the_charge():
    """The meter's CHARGING flag drops at 0.05 A, before the tail ends."""
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 4.20, charger_online=True, charging=True,
               input_present=True, charge_current_a=0.5)
    assert est.status == STATUS_CHARGING
    est.update(1.0, 6 * 4.20, charger_online=True, charging=False,
               input_present=True, charge_current_a=0.02)
    assert est.status == STATUS_CHARGING
    est.update(62.0, 6 * 4.20, charger_online=True, charging=False,
               input_present=True, charge_current_a=0.0)
    assert est.status == STATUS_FULL
    assert est.fraction == pytest.approx(1.0)


def test_unplugged_charger_with_meter_still_reading_pack_is_not_full():
    """Adapter pulled mid-charge: current 0, meter still sees the pack."""
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0,
                           full_tail_current_a=0.2, full_hold_s=60.0)
    est.update(0.0, 6 * 3.72, charger_online=True, charging=True,
               input_present=True, charge_current_a=2.0)
    assert est.status == STATUS_CHARGING
    for t in (1.0, 30.0, 61.0, 120.0):
        est.update(t, 6 * 3.72, charger_online=True, charging=False,
                   input_present=True, charge_current_a=0.0)
    assert est.status == STATUS_NOT_CHARGING
    assert est.fraction == pytest.approx(0.40)


def test_charger_online_without_current_reports_not_charging():
    est = BatteryEstimator(cell_count=6, filter_tau_s=0.0)
    est.update(0.0, 6 * 3.85, charger_online=True, charging=False,
               charge_current_a=0.0)
    assert est.status == STATUS_NOT_CHARGING
    # an already-full pack parked on the charger (CHARGING flag dropped)
    full = BatteryEstimator(cell_count=6, filter_tau_s=0.0)
    full.update(0.0, 6 * 4.20, charger_online=True, charging=False,
                charge_current_a=0.0)
    assert full.status == STATUS_FULL
    # current is only meaningful while the charger answers
    est.update(2.0, 6 * 3.85, charger_online=False)
    assert math.isnan(est.charge_current_a)
    assert est.status == STATUS_DISCHARGING


def test_single_cell_aon_battery():
    est = BatteryEstimator(cell_count=1, filter_tau_s=0.0)
    assert est.update(0.0, 3.78) == pytest.approx(0.5)
    assert est.cell_voltage_v == pytest.approx(3.78)

#!/usr/bin/env python3
"""Voltage-only state-of-charge estimate for the 6S Li-ion main pack.

The base has no pack current sensor yet (only the MG996 servo current), so
this is an OCV lookup with just enough filtering to survive motor sag:

* pack voltage goes through a first-order low-pass (``filter_tau_s``);
* while discharging the estimate may fall freely but rise only at
  ``recovery_rate_per_s`` — a wheel stall pulls the voltage down for a few
  seconds and the reading must not bounce back up as if we had gained
  charge, yet a real rebound after a long sag should still be recovered;
* while the RS485 charger reports CHARGING the estimate may only rise;
* CV phase with the charge current below ``full_tail_current_a`` for
  ``full_hold_s`` means the pack is full, which pins the estimate to 100 %
  and re-anchors it for the next discharge.

Accuracy is roughly +-10 % under load, better at rest. See
docs/BATTERY.md for the coulomb-counting upgrade this is meant to grow into.
"""

import math

# Per-cell open-circuit voltage -> fraction of charge, typical NMC 18650
# (3.0 V cut-off, 4.2 V full). Must be sorted by voltage.
DEFAULT_OCV_TABLE = (
    (3.00, 0.00),
    (3.30, 0.03),
    (3.50, 0.10),
    (3.60, 0.20),
    (3.66, 0.30),
    (3.72, 0.40),
    (3.78, 0.50),
    (3.85, 0.60),
    (3.92, 0.70),
    (4.00, 0.80),
    (4.10, 0.90),
    (4.20, 1.00),
)

STATUS_UNKNOWN = 'unknown'
STATUS_DISCHARGING = 'discharging'
STATUS_CHARGING = 'charging'
STATUS_FULL = 'full'
STATUS_NOT_CHARGING = 'not_charging'  # charger present but no current


def ocv_fraction(cell_v: float, table=DEFAULT_OCV_TABLE) -> float:
    """Interpolate the OCV table; clamps outside its range."""
    if not math.isfinite(cell_v):
        return 0.0
    if cell_v <= table[0][0]:
        return table[0][1]
    if cell_v >= table[-1][0]:
        return table[-1][1]
    for (v0, f0), (v1, f1) in zip(table, table[1:]):
        if v0 <= cell_v <= v1:
            if v1 == v0:
                return f1
            return f0 + (f1 - f0) * (cell_v - v0) / (v1 - v0)
    return table[-1][1]


class BatteryEstimator:
    """Track SOC from pack voltage plus the charger's flags."""

    def __init__(
        self,
        cell_count: int = 6,
        ocv_table=DEFAULT_OCV_TABLE,
        filter_tau_s: float = 20.0,
        recovery_rate_per_s: float = 0.01 / 60.0,  # 1 %/min while discharging
        full_tail_current_a: float = 0.2,
        full_hold_s: float = 60.0,
    ):
        if cell_count < 1:
            raise ValueError('cell_count must be >= 1')
        if filter_tau_s < 0.0:
            raise ValueError('filter_tau_s must be non-negative')
        self.cell_count = int(cell_count)
        self.ocv_table = tuple(ocv_table)
        self.filter_tau_s = float(filter_tau_s)
        self.recovery_rate_per_s = float(recovery_rate_per_s)
        self.full_tail_current_a = float(full_tail_current_a)
        self.full_hold_s = float(full_hold_s)
        self.reset()

    def reset(self):
        self.voltage_v = math.nan          # filtered pack voltage
        self.raw_voltage_v = math.nan
        self.fraction = math.nan           # 0..1
        self.status = STATUS_UNKNOWN
        self.charge_current_a = math.nan   # from the charger, only while online
        self._last_t = None
        self._tail_since = None

    @property
    def percentage(self) -> float:
        return math.nan if math.isnan(self.fraction) else self.fraction * 100.0

    @property
    def cell_voltage_v(self) -> float:
        return self.voltage_v / self.cell_count

    def update(
        self,
        t: float,
        pack_voltage_v: float,
        charger_online: bool = False,
        charging: bool = False,
        cv_phase: bool = False,
        charge_current_a: float = math.nan,
    ) -> float:
        """Feed one sample; returns the new fraction (0..1)."""
        if not math.isfinite(pack_voltage_v) or pack_voltage_v <= 0.0:
            return self.fraction

        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        self.raw_voltage_v = float(pack_voltage_v)

        if math.isnan(self.voltage_v) or self.filter_tau_s <= 0.0:
            self.voltage_v = self.raw_voltage_v
        else:
            alpha = dt / (self.filter_tau_s + dt) if dt > 0.0 else 0.0
            self.voltage_v += alpha * (self.raw_voltage_v - self.voltage_v)

        ocv = ocv_fraction(self.cell_voltage_v, self.ocv_table)
        self.charge_current_a = (
            float(charge_current_a) if charger_online else math.nan
        )

        if charger_online and charging:
            self._update_charging(t, ocv, cv_phase)
        else:
            self._tail_since = None
            self._update_discharging(dt, ocv)
            if charger_online:
                self.status = STATUS_NOT_CHARGING
                if self.fraction >= 0.995:
                    self.status = STATUS_FULL
            else:
                self.status = STATUS_DISCHARGING
        return self.fraction

    def _update_charging(self, t: float, ocv: float, cv_phase: bool):
        # Terminal voltage under charge sits above OCV, so the lookup
        # over-reads; keep it below 100 % until the tail current confirms.
        estimate = min(ocv, 0.99)
        if math.isnan(self.fraction):
            self.fraction = estimate
        else:
            self.fraction = max(self.fraction, estimate)
        self.status = STATUS_CHARGING

        tail = (
            cv_phase
            and math.isfinite(self.charge_current_a)
            and self.charge_current_a <= self.full_tail_current_a
        )
        if not tail:
            self._tail_since = None
            return
        if self._tail_since is None:
            self._tail_since = t
        if t - self._tail_since >= self.full_hold_s:
            self.fraction = 1.0
            self.status = STATUS_FULL

    def _update_discharging(self, dt: float, ocv: float):
        if math.isnan(self.fraction):
            self.fraction = ocv
            return
        if ocv < self.fraction:
            self.fraction = ocv
        else:
            allowed = self.fraction + self.recovery_rate_per_s * dt
            self.fraction = min(ocv, allowed)

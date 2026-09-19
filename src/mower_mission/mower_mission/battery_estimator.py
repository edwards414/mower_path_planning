#!/usr/bin/env python3
"""State-of-charge estimate for the 6S Li-ion main pack.

Inputs per sample: the pack terminal voltage (from the RS485 meter in the
pack lead), whether the charger is connected, and — once the meter's shunt
is wired into the lead — the signed pack current (ROS convention: positive
= charging, negative = discharging, NaN = not measured).

Without current (today) this is an OCV lookup with just enough filtering to
survive motor sag:

* pack voltage goes through a first-order low-pass (``filter_tau_s``);
* while discharging the estimate may fall freely but rise only at
  ``recovery_rate_per_s`` — a wheel stall pulls the voltage down for a few
  seconds and the reading must not bounce back up as if we had gained
  charge, yet a real rebound after a long sag should still be recovered;
* while the charger is connected the terminal voltage is the charger's CV,
  which says nothing about the charge, so the estimate climbs from where it
  was at ``charge_rate_per_s`` (the charger's C-rate) and is capped at 99 %;
* after the charger is removed the pack relaxes for ``post_charge_settle_s``
  and the estimate is then re-anchored to the OCV once.

With current and ``capacity_ah`` known it becomes a coulomb counter:

* ``fraction += I * dt / capacity`` both ways;
* at rest (|I| below ``rest_current_a`` for ``rest_hold_s``) the estimate is
  re-anchored to the OCV, which is how the integration drift is bled off;
* end of charge: charger present, current at or below ``full_tail_current_a``
  and the pack sitting at CV (``full_min_cell_v``) for ``full_hold_s`` →
  100 %, ``FULL``.

Accuracy is roughly +-10 % under load on voltage alone, better at rest;
+-2-3 % once the current is in. See docs/BATTERY.md.
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
    """Track SOC from pack voltage, charger presence and (optionally) current."""

    def __init__(
        self,
        cell_count: int = 6,
        ocv_table=DEFAULT_OCV_TABLE,
        filter_tau_s: float = 20.0,
        recovery_rate_per_s: float = 0.01 / 60.0,  # 1 %/min while discharging
        charge_rate_per_s: float = 0.005 / 60.0,   # 0.5 %/min = ~3.3 h charge
        post_charge_settle_s: float = 300.0,
        capacity_ah: float = math.nan,
        rest_current_a: float = 0.1,
        rest_hold_s: float = 300.0,
        full_tail_current_a: float = 0.2,
        full_hold_s: float = 60.0,
        full_min_cell_v: float = 4.10,
    ):
        if cell_count < 1:
            raise ValueError('cell_count must be >= 1')
        if filter_tau_s < 0.0:
            raise ValueError('filter_tau_s must be non-negative')
        self.cell_count = int(cell_count)
        self.ocv_table = tuple(ocv_table)
        self.filter_tau_s = float(filter_tau_s)
        self.recovery_rate_per_s = float(recovery_rate_per_s)
        self.charge_rate_per_s = float(charge_rate_per_s)
        self.post_charge_settle_s = float(post_charge_settle_s)
        self.capacity_ah = float(capacity_ah)
        self.rest_current_a = float(rest_current_a)
        self.rest_hold_s = float(rest_hold_s)
        self.full_tail_current_a = float(full_tail_current_a)
        self.full_hold_s = float(full_hold_s)
        self.full_min_cell_v = float(full_min_cell_v)
        self.reset()

    def reset(self):
        self.voltage_v = math.nan          # filtered pack voltage
        self.raw_voltage_v = math.nan
        self.fraction = math.nan           # 0..1
        self.status = STATUS_UNKNOWN
        self.current_a = math.nan          # signed, + = charging
        self.charger_present = False
        self._last_t = None
        self._tail_since = None
        self._rest_since = None
        self._charger_removed_t = None     # pending post-charge re-anchor

    @property
    def percentage(self) -> float:
        return math.nan if math.isnan(self.fraction) else self.fraction * 100.0

    @property
    def cell_voltage_v(self) -> float:
        return self.voltage_v / self.cell_count

    @property
    def coulomb_counting(self) -> bool:
        """True when the last sample carried a current and the capacity is known."""
        return math.isfinite(self.current_a) and self.capacity_ah > 0.0

    def update(
        self,
        t: float,
        pack_voltage_v: float,
        charger_present: bool = False,
        current_a: float = math.nan,
    ) -> float:
        """Feed one sample; returns the new fraction (0..1)."""
        if not math.isfinite(pack_voltage_v) or pack_voltage_v <= 0.0:
            return self.fraction

        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        self.raw_voltage_v = float(pack_voltage_v)
        self.current_a = float(current_a) if math.isfinite(current_a) else math.nan

        if math.isnan(self.voltage_v) or self.filter_tau_s <= 0.0:
            self.voltage_v = self.raw_voltage_v
        else:
            alpha = dt / (self.filter_tau_s + dt) if dt > 0.0 else 0.0
            self.voltage_v += alpha * (self.raw_voltage_v - self.voltage_v)

        ocv = ocv_fraction(self.cell_voltage_v, self.ocv_table)

        charger_present = bool(charger_present)
        if charger_present and not self.charger_present:
            self._charger_removed_t = None
        elif self.charger_present and not charger_present:
            self._charger_removed_t = t
        self.charger_present = charger_present

        if self.coulomb_counting and dt > 0.0 and not math.isnan(self.fraction):
            self.fraction += self.current_a * dt / (self.capacity_ah * 3600.0)

        if charger_present:
            self._rest_since = None
            self._update_charging(t, dt, ocv)
        else:
            self._tail_since = None
            self._update_discharging(t, dt, ocv)
        if not math.isnan(self.fraction):
            self.fraction = min(1.0, max(0.0, self.fraction))
        return self.fraction

    # ------------------------------------------------------------------
    def _update_charging(self, t: float, dt: float, ocv: float):
        # Terminal voltage under charge is the charger's CV and says nothing
        # about the charge state, so the OCV is never looked up here.
        if math.isnan(self.fraction):
            # First sample ever and already on the charger: the OCV is an
            # over-estimate, so start below it.
            self.fraction = min(ocv, 0.99)
        elif not self.coulomb_counting and (
            not math.isfinite(self.current_a) or self.current_a >= 0.05
        ):
            # No capacity to integrate against: assume the charger's C-rate.
            self.fraction += self.charge_rate_per_s * dt

        tail = (
            math.isfinite(self.current_a)
            and self.current_a <= self.full_tail_current_a
            and self.cell_voltage_v >= self.full_min_cell_v
        )
        if tail:
            if self._tail_since is None:
                self._tail_since = t
            if t - self._tail_since >= self.full_hold_s:
                self.fraction = 1.0
                self.status = STATUS_FULL
                return
        else:
            self._tail_since = None

        if self.status == STATUS_FULL and (not math.isfinite(self.current_a) or tail):
            # Stay full while parked on the charger.
            return
        if math.isfinite(self.current_a) and self.current_a < 0.05 and not tail:
            # Charger connected but nothing flowing and not at CV: a charger
            # that has cut out, or one that has not started.
            self.status = STATUS_NOT_CHARGING
        else:
            self.status = STATUS_CHARGING
        self.fraction = min(self.fraction, 0.99)

    def _update_discharging(self, t: float, dt: float, ocv: float):
        self.status = STATUS_DISCHARGING
        if math.isnan(self.fraction):
            self.fraction = ocv
            return

        if self.coulomb_counting:
            # Drift correction: re-anchor to the OCV once the pack has rested.
            if abs(self.current_a) < self.rest_current_a:
                if self._rest_since is None:
                    self._rest_since = t
                if t - self._rest_since >= self.rest_hold_s:
                    self.fraction = ocv
            else:
                self._rest_since = None
            self._charger_removed_t = None
            return

        # Voltage only. Right after the charger is removed the terminal
        # voltage is still inflated; wait for it to relax, then snap once.
        if self._charger_removed_t is not None:
            if t - self._charger_removed_t >= self.post_charge_settle_s:
                self.fraction = ocv
                self._charger_removed_t = None
            return

        if ocv < self.fraction:
            self.fraction = ocv
        else:
            self.fraction = min(ocv, self.fraction + self.recovery_rate_per_s * dt)

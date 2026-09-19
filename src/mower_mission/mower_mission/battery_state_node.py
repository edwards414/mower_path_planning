#!/usr/bin/env python3
"""Publish sensor_msgs/BatteryState for the real base from STM32 telemetry.

Reads the ``charger`` (0x89: RS485 voltage / current / temperature meter in
the battery pack lead) and ``analog`` (0x8A: STM32 ADC, only when those
channels are populated) objects of ``/mower_base/telemetry``
(mower_hardware) and runs
:class:`mower_mission.battery_estimator.BatteryEstimator` on them:

* ``/battery_state``      — 24 V main pack (6S): voltage, percentage,
  charging / discharging / full, signed current once the meter's shunt is
  wired into the lead (``meter_current_wired``)
* ``/aon_battery_state``  — 3.7 V always-on cell; ``present`` only when the
  STM32 ADC divider for it exists

Pack voltage: the meter when it answers, else the STM32 ADC main-battery
channel when valid. Charger presence: the meter cannot see the charger
(it is on the pack side), so a pack voltage at or above
``charger_present_min_v`` — the charger's CV, above any resting OCV — means
the charger is connected. Replace with a sense line when one exists.

Replaces ``battery_simulator_node`` on the real robot; the simulator keeps
its place in simulation launch files. ``percentage`` is NaN and ``present``
false until a valid reading arrives, and again if telemetry goes stale for
``stale_timeout_s``.
"""

import json
import math

from mower_mission.battery_estimator import (
    BatteryEstimator,
    STATUS_CHARGING,
    STATUS_FULL,
    STATUS_NOT_CHARGING,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

_STATUS_TO_MSG = {
    STATUS_CHARGING: BatteryState.POWER_SUPPLY_STATUS_CHARGING,
    STATUS_FULL: BatteryState.POWER_SUPPLY_STATUS_FULL,
    STATUS_NOT_CHARGING: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
}


class BatteryStateNode(Node):

    def __init__(self):
        super().__init__('battery_state')

        self.declare_parameter('base_telemetry_topic', '/mower_base/telemetry')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('aon_battery_topic', '/aon_battery_state')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('stale_timeout_s', 5.0)
        self.declare_parameter('main_cell_count', 6)
        self.declare_parameter('filter_tau_s', 20.0)
        self.declare_parameter('recovery_rate_pct_per_min', 1.0)
        # Voltage-only charging ramp: 100 % / (capacity_ah / charger_a) / 60.
        self.declare_parameter('charge_rate_pct_per_min', 0.5)
        self.declare_parameter('post_charge_settle_s', 300.0)
        # Pack voltage at or above this = charger connected (its CV).
        self.declare_parameter('charger_present_min_v', 25.0)
        # Set once the meter's shunt carries the pack current. The meter's
        # register is unsigned: the sign is taken from charger presence
        # unless meter_current_signed says the register wraps for discharge.
        self.declare_parameter('meter_current_wired', False)
        self.declare_parameter('meter_current_signed', False)
        # Pack capacity for coulomb counting; 0 = unknown (voltage only).
        self.declare_parameter('capacity_ah', 0.0)
        self.declare_parameter('rest_current_a', 0.1)
        self.declare_parameter('rest_hold_s', 300.0)
        self.declare_parameter('full_tail_current_a', 0.2)
        self.declare_parameter('full_hold_s', 60.0)
        self.declare_parameter('full_min_cell_v', 4.10)
        self.declare_parameter('low_battery_pct', 20.0)
        self.declare_parameter('frame_id', 'base_footprint')

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.stale_timeout_s = float(p('stale_timeout_s'))
        self.low_battery_pct = float(p('low_battery_pct'))
        self.frame_id = str(p('frame_id'))
        self.charger_present_min_v = float(p('charger_present_min_v'))
        self.meter_current_wired = bool(p('meter_current_wired'))
        self.meter_current_signed = bool(p('meter_current_signed'))
        self.capacity_ah = float(p('capacity_ah'))
        recovery = float(p('recovery_rate_pct_per_min')) / 100.0 / 60.0
        charge_rate = float(p('charge_rate_pct_per_min')) / 100.0 / 60.0

        self.main = BatteryEstimator(
            cell_count=int(p('main_cell_count')),
            filter_tau_s=float(p('filter_tau_s')),
            recovery_rate_per_s=recovery,
            charge_rate_per_s=charge_rate,
            post_charge_settle_s=float(p('post_charge_settle_s')),
            capacity_ah=self.capacity_ah if self.capacity_ah > 0.0 else math.nan,
            rest_current_a=float(p('rest_current_a')),
            rest_hold_s=float(p('rest_hold_s')),
            full_tail_current_a=float(p('full_tail_current_a')),
            full_hold_s=float(p('full_hold_s')),
            full_min_cell_v=float(p('full_min_cell_v')),
        )
        # The AON cell is charged by its own on-board charger the STM32 does
        # not see, so it only gets the voltage lookup.
        self.aon = BatteryEstimator(
            cell_count=1,
            filter_tau_s=float(p('filter_tau_s')),
            recovery_rate_per_s=recovery,
        )
        self.last_main_time = None
        self.last_aon_time = None
        self.low_warned = False

        self.battery_pub = self.create_publisher(BatteryState, p('battery_topic'), 10)
        self.aon_pub = self.create_publisher(BatteryState, p('aon_battery_topic'), 10)

        best_effort = QoSProfile(depth=1)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            String, p('base_telemetry_topic'), self._on_base_telemetry, best_effort
        )

        rate = float(p('publish_rate_hz'))
        self.create_timer(1.0 / rate if rate > 0.0 else 1.0, self._publish)
        mode = 'coulomb counting' if (self.meter_current_wired and self.capacity_ah > 0.0) else 'OCV estimate'
        self.get_logger().info(
            f'battery_state: {self.main.cell_count}S {mode} from '
            f"{p('base_telemetry_topic')} -> {p('battery_topic')}"
        )

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_base_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        analog = data.get('analog') or {}
        charger = data.get('charger') or {}
        now = self._now_s()

        meter_online = bool(charger.get('valid')) and bool(charger.get('online'))
        voltage = math.nan
        if meter_online:
            voltage = float(charger.get('voltage_v', math.nan))
        elif analog.get('valid') and analog.get('main_battery_valid'):
            voltage = float(analog.get('main_battery_v', math.nan))

        if math.isfinite(voltage) and voltage > 0.0:
            charger_present = voltage >= self.charger_present_min_v
            current = math.nan
            if meter_online and self.meter_current_wired:
                current = self._signed_current(float(charger.get('current_a', math.nan)), charger_present)
            self.main.update(now, voltage, charger_present=charger_present, current_a=current)
            self.last_main_time = now
            self._check_low(now)

        if analog.get('valid') and analog.get('aon_battery_valid'):
            self.aon.update(now, float(analog.get('aon_battery_v', math.nan)))
            self.last_aon_time = now

    def _signed_current(self, magnitude_a: float, charger_present: bool) -> float:
        """Meter register -> ROS sign convention (+ charging, - discharging)."""
        if not math.isfinite(magnitude_a):
            return math.nan
        if self.meter_current_signed:
            # uint16 x0.01 A that wraps for negative values
            return magnitude_a - 655.36 if magnitude_a > 327.67 else magnitude_a
        return abs(magnitude_a) if charger_present else -abs(magnitude_a)

    def _check_low(self, now: float):
        pct = self.main.percentage
        if math.isnan(pct):
            return
        if pct <= self.low_battery_pct and self.main.status not in (STATUS_CHARGING, STATUS_FULL):
            if not self.low_warned:
                self.get_logger().warn(
                    f'main battery low: {pct:.0f}% ({self.main.voltage_v:.2f} V)'
                )
                self.low_warned = True
        elif pct > self.low_battery_pct + 5.0:
            self.low_warned = False

    # ------------------------------------------------------------------
    def _publish(self):
        now = self._now_s()
        self.battery_pub.publish(self._message(self.main, self.last_main_time, now))
        self.aon_pub.publish(self._message(self.aon, self.last_aon_time, now))

    def _message(self, est: BatteryEstimator, last_time, now: float) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.design_capacity = math.nan
        msg.capacity = math.nan
        msg.charge = math.nan
        msg.temperature = math.nan

        fresh = last_time is not None and (now - last_time) <= self.stale_timeout_s
        if not fresh or math.isnan(est.fraction):
            msg.present = False
            msg.voltage = math.nan
            msg.current = math.nan
            msg.percentage = math.nan
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            return msg

        msg.present = True
        msg.voltage = float(est.voltage_v)
        # ROS convention: positive current = charging; NaN until the meter's
        # shunt is wired into the pack lead (docs/BATTERY.md).
        msg.current = float(est.current_a)
        msg.percentage = float(est.fraction)
        if est.capacity_ah > 0.0:
            msg.design_capacity = float(est.capacity_ah)
            msg.capacity = float(est.capacity_ah)
            msg.charge = float(est.capacity_ah * est.fraction)
        msg.power_supply_status = _STATUS_TO_MSG.get(
            est.status, BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        if est.cell_voltage_v < 3.0:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif est.cell_voltage_v > 4.3:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        msg.cell_voltage = [msg.voltage / est.cell_count] * est.cell_count
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

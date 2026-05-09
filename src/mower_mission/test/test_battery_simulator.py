import pytest

from mower_mission.battery_simulator import BatteryDistanceSimulator


def test_starts_at_full_battery():
    simulator = BatteryDistanceSimulator()

    assert simulator.percentage == pytest.approx(100.0)
    assert simulator.percentage_fraction == pytest.approx(1.0)


def test_first_position_sets_baseline_without_drain():
    simulator = BatteryDistanceSimulator()

    simulator.update_position(10.0, 20.0)

    assert simulator.distance_m == pytest.approx(0.0)
    assert simulator.percentage == pytest.approx(100.0)


def test_drains_one_percent_per_100_meters():
    simulator = BatteryDistanceSimulator()
    simulator.update_position(0.0, 0.0)

    simulator.update_position(60.0, 0.0)
    simulator.update_position(60.0, 80.0)

    assert simulator.distance_m == pytest.approx(140.0)
    assert simulator.percentage == pytest.approx(98.6)
    assert simulator.percentage_fraction == pytest.approx(0.986)


def test_battery_never_goes_below_zero():
    simulator = BatteryDistanceSimulator()
    simulator.update_position(0.0, 0.0)

    simulator.update_position(20000.0, 0.0)

    assert simulator.percentage == pytest.approx(0.0)
    assert simulator.percentage_fraction == pytest.approx(0.0)


def test_ignores_non_finite_positions():
    simulator = BatteryDistanceSimulator()
    simulator.update_position(0.0, 0.0)

    simulator.update_position(float('nan'), 100.0)
    simulator.update_position(100.0, 0.0)

    assert simulator.distance_m == pytest.approx(100.0)
    assert simulator.percentage == pytest.approx(99.0)

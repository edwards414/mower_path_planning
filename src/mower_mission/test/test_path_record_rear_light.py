"""Rear-light keepalive decisions of the zone / risk / channel recorder."""

from mower_mission.utils.path_record_utils import rear_light_update

PERIOD = 2.0


def test_first_recording_tick_turns_it_on():
    """The first tick of a recording lights it at once."""
    assert rear_light_update(True, False, 0.0, PERIOD) == 'recording'


def test_recording_is_refreshed_every_period_and_not_between():
    """The keepalive goes out once per period, not every tick."""
    assert rear_light_update(True, True, 0.1, PERIOD) is None
    assert rear_light_update(True, True, 1.99, PERIOD) is None
    assert rear_light_update(True, True, 2.0, PERIOD) == 'recording'


def test_off_goes_out_once_after_the_recording_ends():
    """The "off" goes out once, on the first idle tick."""
    assert rear_light_update(False, True, 0.3, PERIOD) == 'off'
    assert rear_light_update(False, False, 0.1, PERIOD) is None


def test_idle_publishes_nothing():
    """An idle recorder stays silent."""
    assert rear_light_update(False, False, 1e6, PERIOD) is None

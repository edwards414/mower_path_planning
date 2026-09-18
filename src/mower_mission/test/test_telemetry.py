"""ROS-free tests for the /robot/telemetry helpers and the rosbridge allow-list."""

import json
import math
from pathlib import Path

from mower_mission import telemetry_node

PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROSBRIDGE_CONFIG = PACKAGE_DIR.parent / 'mower_bringup/config/rosbridge_params.yaml'


def test_quat_to_euler_yaw_only():
    # 90 deg about Z
    s = math.sin(math.radians(45))
    c = math.cos(math.radians(45))
    roll, pitch, yaw = telemetry_node.quat_to_euler_deg(0.0, 0.0, s, c)
    assert abs(roll) < 1e-6 and abs(pitch) < 1e-6
    assert abs(yaw - 90.0) < 1e-6


def test_finite_replaces_nan_and_inf_recursively():
    doc = {'a': float('nan'), 'b': [1.0, float('inf'), {'c': -float('inf')}], 'd': 'x'}
    out = telemetry_node._finite(doc)
    assert out == {'a': None, 'b': [1.0, None, {'c': None}], 'd': 'x'}
    json.dumps(out, allow_nan=False)  # must not raise


def test_sample_rate_window():
    s = telemetry_node._Sample()
    for i in range(11):
        s.set(i, 100.0 + i * 0.1)
    assert abs(s.rate_hz(101.0) - 10.0) < 0.2
    assert s.age(101.5) == 0.5


def test_rosbridge_exposes_telemetry_topic():
    text = ROSBRIDGE_CONFIG.read_text(encoding='utf-8')
    assert "'/robot/telemetry'" in text.split('topics_sub_glob')[1].split('\n')[0]
    rosapi_topics = text.split('rosapi:')[1].split('topics_glob')[1].split('\n')[0]
    assert "'/robot/telemetry'" in rosapi_topics


def test_finite_unwraps_numpy_scalars():
    import numpy as np

    out = telemetry_node._finite({'b': np.bool_(True), 'f': np.float64(1.5), 'n': np.float32('nan')})
    assert out == {'b': True, 'f': 1.5, 'n': None}
    json.dumps(out, allow_nan=False)


def test_battery_fields_map_status_and_absent_pack():
    from sensor_msgs.msg import BatteryState

    absent = telemetry_node.battery_fields(None)
    assert absent['present'] is False and absent['pct'] is None

    msg = BatteryState()
    msg.present = True
    msg.percentage = 0.6321
    msg.voltage = 23.456
    msg.current = math.nan
    msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
    fields = telemetry_node._finite(telemetry_node.battery_fields(msg))
    assert fields == {'present': True, 'pct': 0.632, 'voltage_v': 23.46,
                      'current_a': None, 'status': 'charging'}

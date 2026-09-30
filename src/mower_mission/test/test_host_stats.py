"""ROS-free tests for the telemetry ``host`` block (host_stats.py).

The same vectors as the tests in mower_rs/crates/robot_status/src/host.rs.
"""

import sys
import time

import pytest

from mower_mission import host_stats as hs

STAT_A = ('cpu  100 0 50 800 10 0 5 0 0 0\n'
          'cpu0 25 0 12 200 3 0 1 0 0 0\n'
          'cpu1 25 0 13 200 2 0 1 0 0 0\n'
          'intr 1 2 3\n'
          'procs_running 3\n')
STAT_B = ('cpu  160 0 70 900 20 0 10 0 0 0\n'
          'cpu0 55 0 22 250 8 0 5 0 0 0\n'
          'cpu1 55 0 23 250 7 0 1 0 0 0\n'
          'procs_running 2\n')


def test_stat_lines_and_top_percentages():
    a, b = hs.parse_stat(STAT_A), hs.parse_stat(STAT_B)
    assert len(a['cores']) == 2
    assert a['procs_running'] == 3
    assert hs.cpu_total(a['all']) == 965
    p = hs.cpu_pct(a['all'], b['all'])
    assert p['user'] == pytest.approx(100 * 60 / 195)
    assert p['busy'] == pytest.approx(100 * 85 / 195)
    assert p['iowait'] == pytest.approx(100 * 10 / 195)
    assert p['irq'] == pytest.approx(100 * 5 / 195)
    assert hs.cpu_pct(a['all'], a['all']) is None


def test_meminfo_like_free():
    m = hs.parse_meminfo(
        'MemTotal:        3988480 kB\nMemFree:         2444288 kB\nMemAvailable:    3362816 kB\n'
        'Buffers:           40960 kB\nCached:           880640 kB\nSwapCached:            0 kB\n'
        'SReclaimable:      51200 kB\nSwapTotal:             0 kB\nSwapFree:              0 kB\n')
    b = hs.mem_block(m)
    assert b['total_mb'] == 3895.0
    assert b['used_mb'] == 558.0
    assert b['cache_mb'] == 950.0
    assert b['avail_mb'] == 3284.0
    assert b['swap_total_mb'] == 0.0
    assert hs.mem_block({}) is None


def test_loadavg_fields():
    assert hs.parse_loadavg('2.53 2.65 2.23 3/412 12345\n') == (2.53, 2.65, 2.23, 412)
    assert hs.parse_loadavg('') == (None, None, None, None)


def test_proc_stat_with_awkward_comm():
    line = ('1234 (a b) c)) S 1 1234 1234 0 -1 4194560 100 0 0 0 250 50 0 0 20 0 7 0 4242 '
            '123456 789 1844674 0')
    ps = hs.parse_proc_stat(line)
    assert ps == {'comm': 'a b) c)', 'state': 'S', 'ticks': 300, 'threads': 7, 'start': 4242}
    assert hs.parse_proc_stat('garbage') is None


def test_labels_prefer_node_script_and_full_binary_name():
    def cmd(*args):
        return '\0'.join(args).encode()

    assert hs.process_label('ros2_control_no', cmd(
        '/opt/ros/jazzy/lib/controller_manager/ros2_control_node', '--ros-args')) == ('ros2_control_node', None)
    assert hs.process_label('ekf_node', cmd(
        '/opt/ros/jazzy/lib/robot_localization/ekf_node', '--ros-args', '-r',
        '__node:=ekf_filter_node_odom')) == ('ekf_node', 'ekf_filter_node_odom')
    assert hs.process_label('python3', cmd(
        '/usr/bin/python3', '-u', '/ws/install/lib/mower_mission/telemetry_node.py',
        '--ros-args')) == ('telemetry_node', None)
    assert hs.process_label('python3', cmd('python3', '-m', 'http.server')) == ('http.server', None)
    assert hs.process_label('kworker/0:1', b'') == ('kworker/0:1', None)
    assert hs.process_label('sshd', cmd('sshd: cat@pts/0')) == ('sshd', None)


def test_status_rss():
    assert hs.parse_status_rss_kb('Name:\tx\nVmRSS:\t   51200 kB\nThreads:\t3\n') == 51200
    assert hs.parse_status_rss_kb('Name:\tkthreadd\n') == 0


def test_diskstats_and_filter():
    d = hs.parse_diskstats(
        ' 179       0 mmcblk0 33002 8399 1561004 100336 6290 12544 194456 66661 0 18780 172692 '
        '1548 138 43822680 5474 429 220\n'
        ' 179       1 mmcblk0p1 10 0 80 1 0 0 0 0 0 1 1\n')
    assert len(d) == 2
    assert d[0] == ('mmcblk0', (1561004, 194456, 18780))
    assert hs.is_shown_disk('mmcblk0') and hs.is_shown_disk('nvme0n1')
    for name in ('mmcblk0boot0', 'mmcblk0rpmb', 'loop3', 'zram0'):
        assert not hs.is_shown_disk(name)


def test_mountinfo_longest_prefix():
    m = hs.parse_mountinfo(
        '600 500 0:52 / / rw,relatime - overlay overlay rw,lowerdir=/x\n'
        '601 600 179:3 /home/cat/.mower /home/mower/.mower rw,relatime - ext4 /dev/mmcblk0p3 rw\n'
        '602 601 259:1 / /home/mower/.mower/bags rw,relatime - ext4 /dev/nvme0n1p1 rw\n'
        '603 600 0:60 / /mnt/my\\040disk rw - vfat /dev/sda1 rw\n')
    assert len(m) == 4
    assert hs.mount_for(m, '/home/mower/.mower')[2] == '/dev/mmcblk0p3'
    assert hs.mount_for(m, '/home/mower/.mower/bags')[2] == '/dev/nvme0n1p1'
    assert hs.mount_for(m, '/home/mower/.mowerx')[1] == 'overlay'
    assert hs.mount_for(m, '/')[1] == 'overlay'
    assert hs.mount_for(m, '/mnt/my disk/a')[2] == '/dev/sda1'


def test_df_entry():
    e = hs.disk_entry('state', '/home/mower/.mower', 4096, 7_500_000, 5_600_000, 5_225_000,
                      'ext4', '/dev/mmcblk0p3')
    assert e['dev'] == 'mmcblk0p3'
    assert e['total_gb'] == 30.72
    assert e['used_gb'] == 7.78
    assert e['used_pct'] == 26.7


def test_cpu_lists():
    assert hs.parse_cpu_list('0 1 2 3') == [0, 1, 2, 3]
    assert hs.parse_cpu_list('0-3') == [0, 1, 2, 3]
    assert hs.parse_cpu_list('0-1,4\n') == [0, 1, 4]


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='reads /proc')
def test_live_sample_has_the_whole_shape(tmp_path):
    s = hs.HostSampler(str(tmp_path))
    first = s.poll(0.0)
    assert first['cpu']['pct'] is None
    assert s.poll(0.5) is None
    end = time.monotonic() + 0.3
    while time.monotonic() < end:
        pass
    doc = s.poll(hs.PERIOD_S)
    assert 0.0 <= doc['cpu']['pct'] <= 100.0
    assert doc['cpu']['cores']
    assert doc['mem']['total_mb'] > 0
    assert doc['mem_used_pct'] is not None
    assert doc['load1'] is not None
    assert doc['disks']
    procs = doc['procs']
    assert 0 < len(procs) <= hs.TOP_PROCESSES
    assert procs[0]['cpu'] is not None
    assert doc['tasks']['procs'] >= len(procs)

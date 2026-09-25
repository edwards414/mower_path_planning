# Copyright 2024 fxrbindi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Recorder manager: lifecycle + rosbag2(mcap) + run metadata + fault snapshot.

Three planes (see the package README):
  ① data     : a curated allowlist -> mcap + zstd, split by size/time, with QoS
               overrides so latched maps (/risk_map_inflated ...) replay right.
  ② graph    : recorded via graph_snapshot_node's /graph_snapshot topic.
  ③ metadata : run_id, robot_id, git sha, params dump, GPS start -> run_metadata.yaml

Lifecycle: autostart on launch, plus /mower_recorder/{start,stop,snapshot}
services. Recorders are stopped with SIGINT so the mcap is finalized/indexed.
A fault (Bool on fault_topic) flushes the snapshot buffer of heavy topics.
"""
import datetime
import json
import os
import signal
import subprocess

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

FULL_RECORDER_NODE = 'mower_full_recorder'
SNAPSHOT_RECORDER_NODE = 'mower_snapshot_recorder'


def _latched():
    q = QoSProfile(depth=1)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return q


class RecorderManager(Node):
    def __init__(self):
        super().__init__('recorder_manager')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('robot_id', 'mower')
        self.declare_parameter('output_root', '~/mower_bags')
        self.declare_parameter('config_file', '')
        self.declare_parameter('qos_overrides_path', '')
        self.declare_parameter('autostart', True)
        self.declare_parameter('git_repo_dir', '')
        self.declare_parameter('params_dump_nodes', [''])
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('fault_topic', '/mower_recorder/fault')
        # red breathing rear light while recording (mower_hardware driver ->
        # STM32 0x03 overlay); '' disables
        self.declare_parameter('rear_light_topic', '/mower_base/rear_light')

        self._robot_id = self.get_parameter('robot_id').value
        self._output_root = os.path.expanduser(
            self.get_parameter('output_root').value)
        self._qos_path = self.get_parameter('qos_overrides_path').value
        self._git_dir = os.path.expanduser(self.get_parameter('git_repo_dir').value)
        self._params_nodes = [
            n for n in self.get_parameter('params_dump_nodes').value if n]
        self._cfg = self._load_config(self.get_parameter('config_file').value)

        self._full_proc = None
        self._snap_proc = None
        self._run_id = None
        self._run_dir = None
        self._last_fix = None

        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value,
            self._on_fix, 1, callback_group=cb)
        self.create_subscription(
            Bool, self.get_parameter('fault_topic').value,
            self._on_fault, 10, callback_group=cb)
        self.create_service(
            Trigger, '/mower_recorder/start', self._srv_start, callback_group=cb)
        self.create_service(
            Trigger, '/mower_recorder/stop', self._srv_stop, callback_group=cb)
        self.create_service(
            Trigger, '/mower_recorder/snapshot', self._srv_snapshot,
            callback_group=cb)

        # Live recording status for the app (latched). bag_store also reads it
        # to know which run is in-progress and must not be uploaded/deleted.
        self._start_time = None
        self._status_pub = self.create_publisher(
            String, '/mower_recorder/status', _latched())
        # Re-sent with every status tick: the driver drops the overlay after
        # ~6 s without a refresh, so a dead recorder cannot leave it on.
        rear_topic = self.get_parameter('rear_light_topic').value
        self._rear_pub = (self.create_publisher(String, rear_topic, _latched())
                          if rear_topic else None)
        self.create_timer(2.0, self._publish_status, callback_group=cb)
        self._publish_status()

        if bool(self.get_parameter('autostart').value):
            # Delay so the graph is populated before we snapshot params/metadata.
            self._start_timer = self.create_timer(3.0, self._autostart_once)

    # ── config ───────────────────────────────────────────────────────────────
    def _load_config(self, path):
        cfg = {
            'topics': [],
            'snapshot_topics': [],
            'compression': True,
            'max_bag_size_mb': 512,
            'max_bag_duration_s': 3600,
            'enable_snapshot': True,
            'snapshot_max_cache_mb': 256,
        }
        if path and os.path.isfile(path):
            try:
                with open(path) as f:
                    cfg.update(yaml.safe_load(f) or {})
            except Exception as e:
                self.get_logger().error(f'load config failed: {e}')
        else:
            self.get_logger().warn(f'config_file not found: {path!r}')
        return cfg

    # ── lifecycle ────────────────────────────────────────────────────────────
    def _autostart_once(self):
        self._start_timer.cancel()
        ok, msg = self._start_recording()
        (self.get_logger().info if ok else self.get_logger().error)(
            f'autostart: {msg}')

    def _start_recording(self):
        if self._full_proc and self._full_proc.poll() is None:
            return False, '已在錄製中'
        if not self._cfg['topics']:
            return False, 'record_topics.yaml 沒有指定 topics'

        ts = datetime.datetime.now().strftime('%Y%m%dT%H%M%S')
        self._run_id = f'{self._robot_id}_{ts}'
        self._run_dir = os.path.join(self._output_root, self._run_id)
        os.makedirs(self._run_dir, exist_ok=True)
        self._start_time = datetime.datetime.now()
        self._write_metadata()

        bag_dir = os.path.join(self._run_dir, 'bag')
        cmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', bag_dir,
               '--node-name', FULL_RECORDER_NODE]
        if self._cfg.get('compression', True):
            cmd += ['--compression-mode', 'message',
                    '--compression-format', 'zstd']
        cmd += ['--max-bag-size',
                str(int(self._cfg['max_bag_size_mb']) * 1024 * 1024)]
        cmd += ['--max-bag-duration', str(int(self._cfg['max_bag_duration_s']))]
        if self._qos_path and os.path.isfile(self._qos_path):
            cmd += ['--qos-profile-overrides-path', self._qos_path]
        cmd += list(self._cfg['topics'])  # positional topics must come last
        self._full_proc = subprocess.Popen(cmd)
        self.get_logger().info(
            f'full record -> {bag_dir} ({len(self._cfg["topics"])} topics)')

        if self._cfg.get('enable_snapshot', True) and self._cfg.get(
                'snapshot_topics'):
            snap_dir = os.path.join(self._run_dir, 'snapshots')
            scmd = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', snap_dir,
                    '--node-name', SNAPSHOT_RECORDER_NODE, '--snapshot-mode',
                    '--max-cache-size',
                    str(int(self._cfg['snapshot_max_cache_mb']) * 1024 * 1024)]
            if self._qos_path and os.path.isfile(self._qos_path):
                scmd += ['--qos-profile-overrides-path', self._qos_path]
            scmd += list(self._cfg['snapshot_topics'])  # positional topics last
            self._snap_proc = subprocess.Popen(scmd)
            self.get_logger().info(
                f'snapshot buffer -> {snap_dir} '
                f'({len(self._cfg["snapshot_topics"])} topics)')

        self._publish_status()
        return True, f'開始錄製 run={self._run_id}'

    def _stop_recording(self):
        stopped = []
        for name, proc in (('full', self._full_proc), ('snapshot', self._snap_proc)):
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)  # finalize + index the mcap
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    proc.kill()
                stopped.append(name)
        self._full_proc = None
        self._snap_proc = None
        self._start_time = None
        self._publish_status()
        if stopped:
            return True, f'已停止並封存: {", ".join(stopped)} (run={self._run_id})'
        return False, '目前沒有在錄製'

    def _publish_status(self):
        recording = bool(self._full_proc and self._full_proc.poll() is None)
        elapsed, bag_bytes = 0.0, 0
        if recording and self._start_time is not None:
            elapsed = (datetime.datetime.now() - self._start_time).total_seconds()
            bag_bytes = self._dir_size(os.path.join(self._run_dir, 'bag'))
        st = {
            'recording': recording,
            'run_id': self._run_id if recording else None,
            'start_time': (self._start_time.isoformat()
                           if recording and self._start_time else None),
            'elapsed_s': round(elapsed, 1),
            'bag_bytes': bag_bytes,
            'num_topics': len(self._cfg.get('topics', [])),
        }
        m = String()
        m.data = json.dumps(st, ensure_ascii=False)
        self._status_pub.publish(m)
        if self._rear_pub is not None:
            self._rear_pub.publish(String(
                data=json.dumps({'effect': 'recording' if recording else 'off'})))

    @staticmethod
    def _dir_size(path):
        total = 0
        for dp, _d, files in os.walk(path):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(dp, f))
                except OSError:
                    pass
        return total

    # ── metadata (③) ─────────────────────────────────────────────────────────
    def _write_metadata(self):
        meta = {
            'run_id': self._run_id,
            'robot_id': self._robot_id,
            'start_time': datetime.datetime.now().isoformat(),
            'ros_distro': os.environ.get('ROS_DISTRO', ''),
            'git': self._git_info(),
            'gps_start': self._fix_to_dict(self._last_fix),
            'recorded_topics': list(self._cfg['topics']),
            'snapshot_topics': list(self._cfg.get('snapshot_topics', [])),
        }
        path = os.path.join(self._run_dir, 'run_metadata.yaml')
        try:
            with open(path, 'w') as f:
                yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)
            self.get_logger().info(f'metadata -> {path}')
        except Exception as e:
            self.get_logger().error(f'write metadata failed: {e}')
        self._dump_params()

    def _git_info(self):
        if not self._git_dir:
            return {'sha': 'unknown'}
        try:
            sha = subprocess.check_output(
                ['git', '-C', self._git_dir, 'rev-parse', 'HEAD'],
                stderr=subprocess.DEVNULL, timeout=5).decode().strip()
            desc = subprocess.check_output(
                ['git', '-C', self._git_dir, 'describe', '--always', '--dirty',
                 '--tags'], stderr=subprocess.DEVNULL, timeout=5).decode().strip()
            return {'sha': sha, 'describe': desc}
        except Exception:
            return {'sha': 'unknown'}

    def _dump_params(self):
        if not self._params_nodes:
            return
        pdir = os.path.join(self._run_dir, 'params')
        os.makedirs(pdir, exist_ok=True)
        for node in self._params_nodes:
            try:
                out = subprocess.check_output(
                    ['ros2', 'param', 'dump', node],
                    stderr=subprocess.DEVNULL, timeout=15).decode()
                safe = node.strip('/').replace('/', '__') or 'node'
                with open(os.path.join(pdir, f'{safe}.yaml'), 'w') as f:
                    f.write(out)
            except Exception as e:
                self.get_logger().warn(f'param dump {node} failed: {e}')

    # ── fault -> snapshot ────────────────────────────────────────────────────
    def _on_fault(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('fault received -> flushing snapshot buffer')
            self._trigger_snapshot()

    def _trigger_snapshot(self):
        if not (self._snap_proc and self._snap_proc.poll() is None):
            self.get_logger().warn('no snapshot recorder running')
            return False
        try:
            subprocess.run(
                ['ros2', 'service', 'call',
                 f'/{SNAPSHOT_RECORDER_NODE}/snapshot',
                 'rosbag2_interfaces/srv/Snapshot', '{}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
            return True
        except Exception as e:
            self.get_logger().error(f'snapshot trigger failed: {e}')
            return False

    def _on_fix(self, msg: NavSatFix):
        self._last_fix = msg

    @staticmethod
    def _fix_to_dict(fix):
        if fix is None:
            return None
        return {'lat': fix.latitude, 'lon': fix.longitude, 'alt': fix.altitude}

    # ── services ─────────────────────────────────────────────────────────────
    def _srv_start(self, req, res):
        res.success, res.message = self._start_recording()
        return res

    def _srv_stop(self, req, res):
        res.success, res.message = self._stop_recording()
        return res

    def _srv_snapshot(self, req, res):
        ok = self._trigger_snapshot()
        res.success = ok
        res.message = '已觸發 snapshot' if ok else 'snapshot 觸發失敗'
        return res

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # stops recorders, finalizes the mcap
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

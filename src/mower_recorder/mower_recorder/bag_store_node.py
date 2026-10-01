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
"""Bag store: list / rename / delete + WiFi-gated auto-upload to Cloudflare R2.

Everything is topic-based so the Flutter app can drive it over rosbridge with
plain std_msgs/String (no custom srv, no mower_interface rebuild):

  publishes (latched):
    /mower_recorder/bags            JSON list of all runs on disk (+ upload state)
    /mower_recorder/command_result  {req_id, ok, message} for the last command
  subscribes:
    /mower_recorder/command         {req_id, action, run_id?, new_name?}
                                    action ∈ refresh|rename|delete|upload_now
    /mower_recorder/status          (from recorder_manager) — to know which run
                                    is *currently recording* and must not be
                                    uploaded/deleted.

Upload runs to R2 (S3 API via boto3) only when a good network is up (a WiFi
interface has an IPv4, or an optional dock Bool topic is true). 4G runs stay
queued on disk and go up when the robot is back on WiFi / docked.

Upload contract (run_manifest.py, spec 2.1): all files first, then
``_manifest.json`` (key/size/sha256 + topic counts) as the completion marker;
an interrupted upload resumes from ``<run>/.upload_state.json`` on the next
tick. A run whose bag has no metadata.yaml (recorder not finalized) is not
uploaded; its reason shows up as ``upload_error`` in /mower_recorder/bags.

R2 credentials come from env vars (optionally loaded from a gitignored file):
  R2_ACCOUNT_ID  R2_BUCKET  R2_ACCESS_KEY_ID  R2_SECRET_ACCESS_KEY  R2_PREFIX
"""
import json
import os
import shutil
import subprocess
import threading

import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String

from mower_recorder import run_manifest

UPLOADED_MARKER = run_manifest.UPLOADED_MARKER


def _latched(depth=1):
    q = QoSProfile(depth=depth)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    return q


class BagStore(Node):
    def __init__(self):
        super().__init__('bag_store')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('output_root', '~/mower_bags')
        self.declare_parameter('robot_id', 'mower')
        self.declare_parameter('r2_env_file', '')          # gitignored KEY=VALUE
        self.declare_parameter('upload_policy', 'wifi_or_dock')  # or always/manual
        self.declare_parameter('wifi_interfaces', ['wlan0'])
        self.declare_parameter('dock_topic', '')           # optional std_msgs/Bool
        self.declare_parameter('scan_period_s', 5.0)
        self.declare_parameter('upload_period_s', 30.0)

        self._root = os.path.expanduser(self.get_parameter('output_root').value)
        self._robot_id = self.get_parameter('robot_id').value
        self._policy = self.get_parameter('upload_policy').value
        self._wifi_ifaces = list(self.get_parameter('wifi_interfaces').value)
        self._r2 = self._load_r2_config(self.get_parameter('r2_env_file').value)

        self._recording_run = None    # run_id currently being written (do not touch)
        self._docked = False
        self._uploading = set()       # run_ids mid-upload
        self._upload_errors = {}      # run_id -> last reason (retried next tick)
        self._lock = threading.Lock()

        os.makedirs(self._root, exist_ok=True)

        self._bags_pub = self.create_publisher(
            String, '/mower_recorder/bags', _latched())
        self._result_pub = self.create_publisher(
            String, '/mower_recorder/command_result', 10)

        self.create_subscription(
            String, '/mower_recorder/command', self._on_command, 10,
            callback_group=cb)
        self.create_subscription(
            String, '/mower_recorder/status', self._on_status, _latched(),
            callback_group=cb)
        dock_topic = self.get_parameter('dock_topic').value
        if dock_topic:
            self.create_subscription(
                Bool, dock_topic, self._on_dock, 10, callback_group=cb)

        self.create_timer(
            float(self.get_parameter('scan_period_s').value),
            self._publish_bags, callback_group=cb)
        self.create_timer(
            float(self.get_parameter('upload_period_s').value),
            self._upload_tick, callback_group=cb)

        self.get_logger().info(
            f'bag_store up: root={self._root} policy={self._policy} '
            f'r2={"configured" if self._r2.get("bucket") else "NOT configured"}')
        self._publish_bags()

    # ── R2 config ────────────────────────────────────────────────────────────
    def _load_r2_config(self, env_file):
        cfg = {
            'account_id': os.environ.get('R2_ACCOUNT_ID', ''),
            'bucket': os.environ.get('R2_BUCKET', ''),
            'access_key_id': os.environ.get('R2_ACCESS_KEY_ID', ''),
            'secret_access_key': os.environ.get('R2_SECRET_ACCESS_KEY', ''),
            'prefix': os.environ.get('R2_PREFIX', 'bags'),
        }
        env_file = os.path.expanduser(env_file or '')
        if env_file and os.path.isfile(env_file):
            try:
                for line in open(env_file):
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    k, v = line.split('=', 1)
                    key = k.strip().lower().replace('r2_', '')
                    if key in cfg:
                        cfg[key] = v.strip().strip('"').strip("'")
            except Exception as e:
                self.get_logger().error(f'load r2_env_file failed: {e}')
        return cfg

    def _r2_client(self):
        if not self._r2.get('bucket') or not self._r2.get('account_id'):
            return None
        try:
            import boto3
            return boto3.client(
                's3',
                endpoint_url=(
                    f'https://{self._r2["account_id"]}.r2.cloudflarestorage.com'),
                aws_access_key_id=self._r2['access_key_id'],
                aws_secret_access_key=self._r2['secret_access_key'],
                region_name='auto',
            )
        except Exception as e:
            self.get_logger().error(f'boto3/R2 client init failed: {e}')
            return None

    # ── inbound state ────────────────────────────────────────────────────────
    def _on_status(self, msg: String):
        try:
            st = json.loads(msg.data)
            self._recording_run = st.get('run_id') if st.get('recording') else None
        except Exception:
            self._recording_run = None

    def _on_dock(self, msg: Bool):
        self._docked = bool(msg.data)

    # ── the bag list (published latched) ─────────────────────────────────────
    def _list_runs(self):
        runs = []
        for name in sorted(os.listdir(self._root)):
            run_dir = os.path.join(self._root, name)
            if not os.path.isdir(run_dir):
                continue
            meta = self._read_meta(run_dir)
            runs.append({
                'run_id': name,
                'display_name': meta.get('display_name') or name,
                'robot_id': meta.get('robot_id', self._robot_id),
                'start_time': meta.get('start_time', ''),
                'size_bytes': self._dir_size(run_dir),
                'uploaded': os.path.isfile(os.path.join(run_dir, UPLOADED_MARKER)),
                'uploading': name in self._uploading,
                'recording': name == self._recording_run,
                'profile': meta.get('profile') or 'default',
                'upload_error': self._upload_errors.get(name),
            })
        return runs

    def _publish_bags(self):
        payload = {
            'robot_id': self._robot_id,
            'network_ok': self._network_ok(),
            'r2_configured': bool(self._r2.get('bucket')),
            'bags': self._list_runs(),
        }
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False)
        self._bags_pub.publish(m)

    # ── commands from the app ────────────────────────────────────────────────
    def _on_command(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            return self._result('', False, f'bad command json: {e}')
        action = cmd.get('action', '')
        run_id = cmd.get('run_id', '')
        req_id = cmd.get('req_id', '')

        if action == 'refresh':
            self._publish_bags()
            return self._result(req_id, True, 'refreshed')
        if action == 'rename':
            return self._result(req_id, *self._rename(run_id, cmd.get('new_name', '')))
        if action == 'delete':
            return self._result(req_id, *self._delete(run_id))
        if action == 'upload_now':
            threading.Thread(target=self._upload_tick, args=(True,),
                             daemon=True).start()
            return self._result(req_id, True, '已排入上傳')
        return self._result(req_id, False, f'unknown action: {action}')

    def _result(self, req_id, ok, message):
        m = String()
        m.data = json.dumps(
            {'req_id': req_id, 'ok': ok, 'message': message}, ensure_ascii=False)
        self._result_pub.publish(m)
        (self.get_logger().info if ok else self.get_logger().warn)(
            f'command_result: {message}')

    def _rename(self, run_id, new_name):
        run_dir = self._safe_run_dir(run_id)
        if not run_dir:
            return False, f'找不到 run: {run_id}'
        if not new_name.strip():
            return False, '新名稱不可空白'
        meta = self._read_meta(run_dir)
        meta['display_name'] = new_name.strip()
        self._write_meta(run_dir, meta)
        self._publish_bags()
        return True, f'已更名為「{new_name.strip()}」'

    def _delete(self, run_id):
        if run_id == self._recording_run:
            return False, '這個 run 正在錄製中,無法刪除'
        run_dir = self._safe_run_dir(run_id)
        if not run_dir:
            return False, f'找不到 run: {run_id}'
        # .upload_state.json = a partial upload may have left objects on R2.
        on_r2 = any(os.path.isfile(os.path.join(run_dir, f)) for f in (
            UPLOADED_MARKER, run_manifest.UPLOAD_STATE))
        base_key = self._base_key(run_id, run_dir)  # before the dir is gone
        try:
            shutil.rmtree(run_dir)
        except Exception as e:
            return False, f'刪除本機失敗: {e}'
        if on_r2:
            self._delete_r2(base_key)
        self._upload_errors.pop(run_id, None)
        self._publish_bags()
        return True, f'已刪除 {run_id}'

    # ── upload ───────────────────────────────────────────────────────────────
    def _network_ok(self):
        if self._policy == 'always':
            return True
        if self._policy == 'manual':
            return False
        # wifi_or_dock
        if self._docked:
            return True
        return any(self._iface_up(i) for i in self._wifi_ifaces)

    @staticmethod
    def _iface_up(iface):
        try:
            if open(f'/sys/class/net/{iface}/operstate').read().strip() != 'up':
                return False
            out = subprocess.check_output(
                ['ip', '-4', 'addr', 'show', iface],
                stderr=subprocess.DEVNULL).decode()
            return 'inet ' in out
        except Exception:
            return False

    def _upload_tick(self, force=False):
        if not (force or self._network_ok()):
            return
        client = self._r2_client()
        if client is None:
            return
        for run in self._list_runs():
            rid = run['run_id']
            if run['uploaded'] or run['recording'] or rid in self._uploading:
                continue
            with self._lock:
                if rid in self._uploading:
                    continue
                self._uploading.add(rid)
            try:
                self._publish_bags()
                self._upload_run(client, rid)
                self._upload_errors.pop(rid, None)
            except run_manifest.RunNotReady as e:
                if self._upload_errors.get(rid) != str(e):
                    self.get_logger().warn(f'not uploading {rid}: {e}')
                self._upload_errors[rid] = str(e)
            except Exception as e:  # noqa: BLE001 — network etc.: retry next tick
                self.get_logger().error(
                    f'upload {rid} interrupted ({type(e).__name__}: {e}); '
                    'will resume on the next tick')
                self._upload_errors[rid] = f'{type(e).__name__}: {e}'
            finally:
                self._uploading.discard(rid)
            self._publish_bags()

    def _base_key(self, run_id, run_dir=None):
        meta = self._read_meta(run_dir or os.path.join(self._root, run_id))
        return run_manifest.run_key(
            self._r2['prefix'], meta.get('robot_id') or self._robot_id, run_id)

    def _upload_run(self, client, run_id):
        run_dir = os.path.join(self._root, run_id)
        base_key = self._base_key(run_id, run_dir)
        meta = self._read_meta(run_dir)
        res = run_manifest.upload_run(
            client, self._r2['bucket'], base_key, run_dir, run_id,
            meta.get('robot_id') or self._robot_id,
            log=self.get_logger().debug)
        self.get_logger().info(
            f'uploaded {run_id} -> r2://{self._r2["bucket"]}/{base_key} '
            f'({res["files"]} files, {res["uploaded"]} new, '
            f'{res["skipped"]} resumed) + {run_manifest.MANIFEST_NAME}')

    def _delete_r2(self, base_key):
        client = self._r2_client()
        if client is None:
            return
        try:
            n = run_manifest.delete_run_objects(
                client, self._r2['bucket'], base_key)  # manifest first
            self.get_logger().info(f'deleted r2 {base_key} ({n} objects)')
        except Exception as e:
            self.get_logger().error(f'delete r2 failed: {e}')

    # ── helpers ──────────────────────────────────────────────────────────────
    def _safe_run_dir(self, run_id):
        if not run_id or '/' in run_id or run_id.startswith('.'):
            return None
        run_dir = os.path.join(self._root, run_id)
        return run_dir if os.path.isdir(run_dir) else None

    @staticmethod
    def _read_meta(run_dir):
        path = os.path.join(run_dir, 'run_metadata.yaml')
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}

    @staticmethod
    def _write_meta(run_dir, meta):
        path = os.path.join(run_dir, 'run_metadata.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)

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


def main(args=None):
    rclpy.init(args=args)
    node = BagStore()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Wheel-speed PID auto-tune for the STM32 base.

Drives both wheels open loop through two PWM steps (0 -> low -> high), reads
the speed response the STM32 reports in 0x85 (via the mower_hardware
``/mower_base/telemetry`` JSON), fits a first-order-plus-dead-time model per
wheel, turns it into SIMC PI gains (pid_tuning.py), verifies them with a
closed-loop step and then waits for the operator to write them to flash or
throw them away. Nothing is persisted without an explicit "apply".

The wheels spin at up to ``step_high_permille`` duty while this runs: the
vehicle has to be jacked up with both wheels free. The dashboard asks the
operator to confirm that before it calls "start"; this node cannot check it.

Interface
---------
service ``/pid_autotune`` (mower_interface/srv/PidAutotune):
    op = "start" | "abort" | "apply" | "discard"
topic  ``/pid_autotune/status`` (std_msgs/String JSON, latched, 5 Hz while active)::

    {"state": "idle|precheck|open_loop|fitting|verify|review|saving|done|failed|aborted",
     "progress": 0.0-1.0, "message": "...", "error": null|"...",
     "started_at": unix_s, "updated_at": unix_s,
     "old_gains": {"left": {"kp","ki","kd"}, "right": {...}} | null,
     "new_gains": {...} | null,
     "model": {"left": {"gain","tau","delay",...}, "right": {...}} | null,
     "verify": {"left": {"target","overshoot_pct","settle_s","ss_error","rise_s"}, "right": {...}} | null,
     "samples": {"open_loop": {"left": [[t, pwm, rpm], ...], "right": [...]},
                 "verify":    {"left": [...], "right": [...]}},
     "marks": {"open_loop_low": t, "open_loop_high": t, "verify": t}}

``t`` in samples/marks is seconds since ``started_at``.

Uses the mower_hardware side channels ``/mower_base/pid_command`` (0x04) and
``/mower_base/wheel_override`` (raw permille, bypasses diff_drive_controller's
acceleration limits so a step is a step).
"""

from __future__ import annotations

import json
import threading
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String

from mower_interface.srv import MissionOperationLock, PidAutotune
from mower_mission import pid_tuning as pt

# telemetry pid.flags (UART 0x84)
PID_FLAG_CLOSED_LOOP = 0x01
PID_FLAG_FLASH_VALID = 0x02
PID_FLAG_LAST_SAVE_OK = 0x04
PID_FLAG_LAST_APPLY_OK = 0x08
# telemetry motor.flags (0x81)
MOTOR_FLAG_DRIVER_ALARM = 0x04
# telemetry power.state (0x86)
POWER_STATE_RUNNING = 0

WHEELS = ('left', 'right')


class Aborted(Exception):
    pass


class Precondition(Exception):
    pass


class PidAutotuneNode(Node):

    def __init__(self, **node_kwargs):
        super().__init__('pid_autotune', **node_kwargs)
        p = self.declare_parameter
        p('telemetry_topic', '/mower_base/telemetry')
        p('pid_topic', '/mower_base/pid_command')
        p('override_topic', '/mower_base/wheel_override')
        p('led_topic', '/mower_base/led_command')
        p('status_topic', '/pid_autotune/status')
        p('nav_active_topic', '/nav_operation_active')
        p('lock_service', '/mission_operation_lock')
        # open-loop identification: two forward steps so static friction at
        # standstill does not end up in the model
        p('step_low_permille', 400)
        p('step_high_permille', 700)
        p('hold_s', 2.5)
        # closed-loop check with the new gains
        p('verify_permille', 500)
        p('verify_hold_s', 3.0)
        p('tau_c_factor', 1.0)          # SIMC closed-loop time constant = factor * tau (>= dead time)
        p('max_overshoot_pct', 25.0)
        p('max_settle_s', 1.5)
        p('max_ss_error_rpm', 1.5)
        p('telemetry_timeout_s', 0.5)   # abort if the base goes quiet for this long
        p('review_timeout_s', 300.0)    # no apply/discard within this = discard
        p('led_busy_rgb', [255, 180, 0])
        p('led_busy_period_ms', 1600)
        p('led_normal_rgb', [110, 110, 110])

        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        best_effort = QoSProfile(depth=1)
        best_effort.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self._status_pub = self.create_publisher(String, self._p('status_topic'), latched)
        self._pid_pub = self.create_publisher(String, self._p('pid_topic'), QoSProfile(depth=4))
        self._override_pub = self.create_publisher(String, self._p('override_topic'), best_effort)
        led_topic = self._p('led_topic')
        self._led_pub = self.create_publisher(String, led_topic, latched) if led_topic else None
        # /mower_base/telemetry runs at ~17 Hz and rclpy costs ~6 ms per
        # message on the LubanCat, so the subscription only exists while a
        # tuning session runs (created on `start`, dropped by _tick once the
        # worker has finished). Idle, this node then costs nothing.
        self._telemetry_qos = best_effort
        self._telemetry_sub = None
        self.create_subscription(Bool, self._p('nav_active_topic'), self._on_nav_active, 10)
        self.create_service(PidAutotune, '/pid_autotune', self._on_service)
        self._lock_client = self.create_client(MissionOperationLock, self._p('lock_service'))
        self._lock_owner = f'{self.get_fully_qualified_name()}:{uuid4().hex}'
        self._lock_held = False

        self._tel = None
        self._tel_time = 0.0
        self._tel_seq = None
        self._alarm_logged = False
        self._nav_active = False
        self._recording = None          # dict wheel -> list[Sample] while recording

        self._mutex = threading.Lock()
        self._worker = None
        self._abort = threading.Event()
        self._decision = None            # 'apply' | 'discard'
        self._decision_event = threading.Event()
        self._started_mono = 0.0
        self._status = self._blank_status('idle', 'ready')
        # Progress is republished at 5 Hz only while a session is active; the
        # timer stays cancelled otherwise so the executor never wakes for it.
        self._tick_timer = self.create_timer(0.2, self._tick)
        self._tick_timer.cancel()
        self._publish_status()
        self.get_logger().info('pid_autotune ready: service /pid_autotune, status on ' + self._p('status_topic'))

    # ------------------------------------------------------------------ helpers
    def _p(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _blank_status(state, message):
        return {
            'state': state, 'progress': 0.0, 'message': message, 'error': None,
            'started_at': None, 'updated_at': time.time(),
            'old_gains': None, 'new_gains': None, 'model': None, 'verify': None,
            'samples': {'open_loop': {'left': [], 'right': []}, 'verify': {'left': [], 'right': []}},
            'marks': {},
        }

    def _active(self):
        return self._worker is not None and self._worker.is_alive()

    def _set(self, state=None, progress=None, message=None, **fields):
        with self._mutex:
            if state is not None:
                self._status['state'] = state
            if progress is not None:
                self._status['progress'] = float(progress)
            if message is not None:
                self._status['message'] = message
            self._status.update(fields)
            self._status['updated_at'] = time.time()
        if state is not None or message is not None:
            self.get_logger().info(f'{self._status["state"]}: {self._status["message"]}')
        self._publish_status()

    def _publish_status(self):
        with self._mutex:
            data = json.dumps(self._status, separators=(',', ':'), allow_nan=False)
        self._status_pub.publish(String(data=data))

    def _tick(self):
        if self._active():
            self._publish_status()
            return
        # Session over: stop listening to the base and go back to sleep.
        self._stop_telemetry()
        self._tick_timer.cancel()

    def _start_telemetry(self):
        if self._telemetry_sub is None:
            self._tel = None
            self._tel_time = 0.0
            self._telemetry_sub = self.create_subscription(
                String, self._p('telemetry_topic'), self._on_telemetry, self._telemetry_qos)

    def _stop_telemetry(self):
        sub, self._telemetry_sub = self._telemetry_sub, None
        if sub is not None:
            self.destroy_subscription(sub)

    def _rel(self, mono):
        return round(mono - self._started_mono, 3)

    # ---------------------------------------------------------------- callbacks
    def _on_nav_active(self, msg: Bool):
        self._nav_active = bool(msg.data)

    def _on_telemetry(self, msg: String):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        self._tel = d
        self._tel_time = now
        rec = self._recording
        if rec is None:
            return
        wheel = d.get('wheel') or {}
        seq = wheel.get('seq')
        if seq == self._tel_seq:
            return  # driver republished the same 0x85
        self._tel_seq = seq
        for w in WHEELS:
            side = wheel.get(w) or {}
            try:
                rec[w].append(pt.Sample(t=now, pwm=float(side['pid_output']), rpm=float(side['measured_rpm'])))
            except (KeyError, TypeError, ValueError):
                pass

    def _on_service(self, req, res):
        op = str(req.op).strip().lower()
        if op == 'start':
            if self._active():
                res.success, res.message = False, f'auto-tune already running ({self._status["state"]})'
                return res
            self._abort.clear()
            self._decision = None
            self._decision_event.clear()
            self._start_telemetry()
            self._tick_timer.reset()
            self._worker = threading.Thread(target=self._run, name='pid_autotune', daemon=True)
            self._worker.start()
            res.success, res.message = True, 'auto-tune started'
        elif op == 'abort':
            if not self._active():
                res.success, res.message = False, 'nothing to abort'
            else:
                self._abort.set()
                self._decision_event.set()
                res.success, res.message = True, 'aborting'
        elif op in ('apply', 'discard'):
            if not self._active() or self._status['state'] != 'review':
                res.success, res.message = False, 'no tuned gains waiting for review'
            else:
                self._decision = op
                self._decision_event.set()
                res.success, res.message = True, op
        else:
            res.success, res.message = False, f'unknown op {req.op!r} (start|abort|apply|discard)'
        return res

    # ------------------------------------------------------------ base helpers
    def _telemetry(self, timeout_s=None):
        """Latest base telemetry, or raise when the base has gone quiet."""
        if self._abort.is_set():
            raise Aborted()
        if timeout_s is None:
            timeout_s = float(self._p('telemetry_timeout_s'))
        # The subscription is created on `start`, so the first frame of a
        # session may still be in flight: give the base one timeout window
        # (worker thread; the executor keeps delivering meanwhile).
        deadline = time.monotonic() + timeout_s
        while self._tel is None and time.monotonic() < deadline:
            if self._abort.is_set():
                raise Aborted()
            time.sleep(0.02)
        tel = self._tel
        if tel is None or time.monotonic() - self._tel_time > timeout_s:
            raise Precondition('no /mower_base/telemetry (is the base driver running?)')
        # motor.flags DRIVER_ALARM is the BTS7960 IS pin read as a GPIO: an
        # analog current-sense output that goes high with normal drive current
        # (firmware motor.hpp MOTOR_ALARM_DISABLES_OUTPUT 0). Advisory only, so
        # it is logged, never a reason to stop; the BTS7960 protects itself.
        motor = tel.get('motor') or {}
        if motor.get('valid') and int(motor.get('flags', 0)) & MOTOR_FLAG_DRIVER_ALARM:
            if not self._alarm_logged:
                self._alarm_logged = True
                self.get_logger().info('driver alarm flag set (BTS7960 IS current sense); ignored')
        power = tel.get('power') or {}
        if power.get('valid') and int(power.get('state', 0)) != POWER_STATE_RUNNING:
            raise Precondition(f'power state {power.get("state")} is not RUNNING')
        return tel

    def _pid_status(self, timeout_s=None):
        pid = self._telemetry(timeout_s).get('pid') or {}
        if not pid.get('valid'):
            raise Precondition('base has not reported its PID settings yet')
        return pid

    def _gains_of(self, pid):
        return {w: {k: float(pid[w][k]) for k in ('kp', 'ki', 'kd')} for w in WHEELS}

    def _send_pid(self, gains, *, closed_loop, persist=False):
        """Publish a 0x04 request; returns the 0x84 last_rx_seq seen *before* it
        went out, so _wait_pid can tell a fresh acknowledgement from the old state."""
        seq0 = ((self._tel or {}).get('pid') or {}).get('last_rx_seq')
        req = {
            'left': gains['left'], 'right': gains['right'],
            'persist': 1 if persist else 0, 'closed_loop': 1 if closed_loop else 0,
        }
        self._pid_pub.publish(String(data=json.dumps(req, separators=(',', ':'))))
        return seq0

    @staticmethod
    def _gains_close(a, b, tol=1e-3):
        return all(abs(float(a[w][k]) - float(b[w][k])) <= tol for w in WHEELS for k in ('kp', 'ki', 'kd'))

    def _wait_pid(self, gains, seq0, *, closed_loop, timeout_s=1.5, persisted=False):
        """Wait until the STM32 reports the requested settings (0x84 in telemetry)."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            # a flash sector erase stalls the STM32 (and its status frames) for
            # up to a couple of seconds; do not call that a dead base
            pid = self._pid_status(timeout_s=timeout_s if persisted else None)
            flags = int(pid.get('flags', 0))
            seen = seq0 is None or pid.get('last_rx_seq') != seq0
            if (seen and self._gains_close(self._gains_of(pid), gains)
                    and bool(flags & PID_FLAG_CLOSED_LOOP) == bool(closed_loop)
                    and flags & PID_FLAG_LAST_APPLY_OK
                    and (not persisted or flags & PID_FLAG_FLASH_VALID)):
                return pid
            self._sleep(0.05)
        # last_rx_seq is a uint8 of the driver's frame counter, so once in 256
        # runs the acknowledgement is indistinguishable from the previous one;
        # matching gains and mode are then the best evidence there is.
        pid = self._pid_status(timeout_s=timeout_s if persisted else None)
        flags = int(pid.get('flags', 0))
        if (self._gains_close(self._gains_of(pid), gains)
                and bool(flags & PID_FLAG_CLOSED_LOOP) == bool(closed_loop)
                and flags & PID_FLAG_LAST_APPLY_OK):
            self.get_logger().warning('PID settings match but no fresh 0x84 seq was seen; accepting')
            return pid
        raise Precondition('base did not acknowledge the PID settings')

    def _persist(self, gains):
        """0x04 with persist=1, confirmed by FLASH_VALID + LAST_APPLY_OK. The
        sector erase stalls the STM32 for ~1 s. One retry: firmware before
        cef0d1e fails the first save after a bootloader jump because of a
        stale FLASH_SR error bit, and the failed attempt clears it."""
        last = None
        for attempt in (1, 2):
            seq0 = self._send_pid(gains, closed_loop=True, persist=True)
            try:
                self._wait_pid(gains, seq0, closed_loop=True, timeout_s=4.0, persisted=True)
                return
            except Precondition as e:
                diag = ((self._tel or {}).get('pid') or {}).get('flash_diag', 0)
                last = Precondition(f'{e} (flash save attempt {attempt}, flash_diag=0x{int(diag or 0):04x})')
                self.get_logger().warning(str(last))
        raise last

    def _override(self, left, right, ttl_ms=300):
        self._override_pub.publish(String(data=json.dumps(
            {'left_permille': int(left), 'right_permille': int(right), 'ttl_ms': int(ttl_ms)},
            separators=(',', ':'))))

    def _hold(self, permille, seconds):
        """Keep both wheels at `permille` for `seconds`, watching the base."""
        end = time.monotonic() + seconds
        while True:
            self._telemetry()
            self._override(permille, permille)
            if time.monotonic() >= end:
                return
            self._sleep(0.1)

    def _sleep(self, seconds):
        if self._abort.wait(seconds):
            raise Aborted()

    def _record(self, on):
        if on:
            self._tel_seq = None
            self._recording = {w: [] for w in WHEELS}
            return self._recording
        rec, self._recording = self._recording, None
        return rec

    def _samples_json(self, rec):
        return {w: [[self._rel(s.t), round(s.pwm, 1), round(s.rpm, 2)] for s in rec[w]] for w in WHEELS}

    def _lights(self, busy):
        if self._led_pub is None:
            return
        if busy:
            r, g, b = (int(v) for v in self._p('led_busy_rgb'))
            req = {'mode': 6, 'r': r, 'g': g, 'b': b, 'period_ms': int(self._p('led_busy_period_ms'))}
        else:
            r, g, b = (int(v) for v in self._p('led_normal_rgb'))
            req = {'mode': 1, 'r': r, 'g': g, 'b': b, 'period_ms': 0}
        self._led_pub.publish(String(data=json.dumps(req, separators=(',', ':'))))

    def _lock(self, acquire):
        """Mission mutation lease from nav_action_server; refused while navigation or
        manual motion is active. Missing service = development bench, carry on."""
        if not self._lock_client.wait_for_service(timeout_sec=1.0):
            if acquire:
                self.get_logger().warning('mission operation lock service unavailable, continuing without it')
            return
        req = MissionOperationLock.Request()
        req.owner = self._lock_owner
        req.operation = 'pid auto-tune'
        req.acquire = acquire
        fut = self._lock_client.call_async(req)
        deadline = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not fut.done():
            raise Precondition('mission operation lock did not answer')
        res = fut.result()
        if acquire and not res.success:
            raise Precondition(f'robot is busy: {res.message}')
        self._lock_held = acquire

    # ---------------------------------------------------------------- the run
    def _run(self):
        self._started_mono = time.monotonic()
        self._alarm_logged = False
        with self._mutex:
            self._status = self._blank_status('precheck', 'checking the base')
            self._status['started_at'] = time.time()
        self._publish_status()
        old_gains = None
        old_closed_loop = True
        outcome = ('failed', None)
        try:
            # 1. preconditions
            pid = self._pid_status()
            old_gains = self._gains_of(pid)
            old_closed_loop = bool(int(pid.get('flags', 0)) & PID_FLAG_CLOSED_LOOP)
            if self._nav_active:
                raise Precondition('navigation is active')
            self._lock(True)
            self._lights(True)
            self._set(progress=0.05, message='base ok, switching to open loop', old_gains=old_gains)

            # 2. open-loop steps
            hold = float(self._p('hold_s'))
            low = int(self._p('step_low_permille'))
            high = int(self._p('step_high_permille'))
            seq0 = self._send_pid(old_gains, closed_loop=False)
            self._wait_pid(old_gains, seq0, closed_loop=False)
            rec = self._record(True)
            self._set('open_loop', 0.10, f'open loop: 0 -> {low / 10:.0f} % -> {high / 10:.0f} % duty')
            self._hold(0, 0.6)
            t_low = time.monotonic()
            self._hold(low, hold)
            t_high = time.monotonic()
            self._set(progress=0.30)
            self._hold(high, hold)
            t_high_end = time.monotonic()
            self._hold(0, 0.3)
            self._record(False)
            with self._mutex:
                self._status['samples']['open_loop'] = self._samples_json(rec)
                self._status['marks'].update({'open_loop_low': self._rel(t_low), 'open_loop_high': self._rel(t_high)})

            # 3. fit + tune (the low->high step; the 0->low one carries stiction)
            self._set('fitting', 0.50, 'fitting the step response')
            models, new_gains = {}, {}
            for w in WHEELS:
                try:
                    models[w] = pt.fit_fopdt(rec[w], t_step=t_high, t_end=t_high_end)
                except pt.TuningError as e:
                    raise pt.TuningError(f'{w} wheel: {e}')
                new_gains[w] = pt.simc_pi(models[w], tau_c_factor=float(self._p('tau_c_factor'))).to_dict()
            self._set(progress=0.55, message='gains computed, verifying closed loop',
                      model={w: models[w].to_dict() for w in WHEELS}, new_gains=new_gains)

            # 4. closed-loop verification with the new gains (RAM only)
            seq0 = self._send_pid(new_gains, closed_loop=True)
            self._wait_pid(new_gains, seq0, closed_loop=True)
            rec = self._record(True)
            self._set('verify', 0.60, f'closed loop step to {int(self._p("verify_permille")) / 10:.0f} % speed')
            self._hold(0, 0.5)
            t_v = time.monotonic()
            verify_hold = float(self._p('verify_hold_s'))
            self._hold(int(self._p('verify_permille')), verify_hold * 0.5)
            targets = {w: float(((self._telemetry().get('wheel') or {}).get(w) or {}).get('target_rpm', 0.0)) for w in WHEELS}
            self._hold(int(self._p('verify_permille')), verify_hold * 0.5)
            t_v_end = time.monotonic()
            self._hold(0, 0.3)
            self._record(False)
            with self._mutex:
                self._status['samples']['verify'] = self._samples_json(rec)
                self._status['marks']['verify'] = self._rel(t_v)
            verify = {}
            problems = []
            for w in WHEELS:
                m = pt.step_metrics(rec[w], t_step=t_v, target=targets[w], t_end=t_v_end)
                verify[w] = m.to_dict()
                if m.overshoot_pct > float(self._p('max_overshoot_pct')):
                    problems.append(f'{w} overshoot {m.overshoot_pct:.0f} %')
                if m.settle_s is None or m.settle_s > float(self._p('max_settle_s')):
                    problems.append(f'{w} settle {"never" if m.settle_s is None else f"{m.settle_s:.2f} s"}')
                if abs(m.ss_error) > float(self._p('max_ss_error_rpm')):
                    problems.append(f'{w} steady-state error {m.ss_error:+.2f} rpm')
            self._set(progress=0.85, verify=verify)
            if problems:
                raise pt.TuningError('verification failed: ' + ', '.join(problems))

            # 5. review: operator decides
            self._set('review', 0.90, 'tuned gains verified; apply to flash or discard')
            if not self._decision_event.wait(float(self._p('review_timeout_s'))):
                self._decision = 'discard'
            if self._abort.is_set():
                raise Aborted()
            if self._decision == 'apply':
                self._set('saving', 0.95, 'writing gains to STM32 flash')
                self._persist(new_gains)
                outcome = ('done', 'new gains saved to flash')
                old_gains = None  # keep the new ones
            else:
                outcome = ('idle', 'tuned gains discarded, previous gains restored')
        except Aborted:
            outcome = ('aborted', 'aborted by operator')
        except (Precondition, pt.TuningError) as e:
            outcome = ('failed', str(e))
        except Exception as e:  # noqa: BLE001 - whatever happens, the wheels stop and gains come back
            self.get_logger().error(f'auto-tune crashed: {e!r}')
            outcome = ('failed', f'internal error: {e!r}')
        finally:
            self._recording = None
            self._override(0, 0, ttl_ms=0)
            if old_gains is not None:
                try:
                    self._abort.clear()  # the restore must go through even after an abort
                    seq0 = self._send_pid(old_gains, closed_loop=old_closed_loop)
                    self._wait_pid(old_gains, seq0, closed_loop=old_closed_loop)
                except (Precondition, Aborted) as e:
                    self.get_logger().error(f'could not confirm the previous gains were restored: {e}')
            self._lights(False)
            if self._lock_held:
                try:
                    self._lock(False)
                except Precondition as e:
                    self.get_logger().warning(str(e))
            state, msg = outcome
            self._set(state, 1.0 if state == 'done' else None, msg,
                      error=None if state in ('done', 'idle') else msg)


def main(args=None):
    rclpy.init(args=args)
    node = PidAutotuneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

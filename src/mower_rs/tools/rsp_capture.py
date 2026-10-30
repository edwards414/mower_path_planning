#!/usr/bin/env python3
"""What robot_state_publisher publishes for the real robot's URDF, captured
for the static_tf module's golden tests (step E4 of docs/ROS_FREE_PLAN.md).

Runs inside the runtime image, offline, on a DDS domain of its own::

    docker run --rm -u 0 --network none --cap-add NET_ADMIN -e ROS_DOMAIN_ID=73 \\
      -v <repo>:/r:ro -v <out>:/o --entrypoint bash mower_path_planning:ros-free-test -c \\
      'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && \\
       python3 /r/src/mower_rs/tools/rsp_capture.py --out /o/rsp_capture.json'

It runs xacro on mower_description's real_robot.xacro and starts the image's
robot_state_publisher on it, with the parameters mower.launch.py gives it,
then records, as a late-joining DDS peer:

- ``/tf_static`` and ``/robot_description`` (transient local);
- ``/tf``, for a script of ``/joint_states`` sent at mower_base's 25 Hz:
  wall-clock stamps (rsp keeps every other one at its 20 Hz
  ``publish_frequency``), then hand-made stamps around the 50 ms interval,
  joints alone and together, the three joints production never sends
  (caster yaw and wheel, the blade), a fixed and an unknown joint, large
  angles, two invalid messages, a zero stamp and, last, a stamp 10 s ahead
  (rsp then publishes nothing until the wall clock gets there).

Every float is written with ``repr``, so the file holds the exact doubles.
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import threading
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(TOOLS, '../../..'))
XACRO = os.path.join(REPO, 'src/mower_description/mower_robot/real_robot.xacro')

WHEELS = ['left_wheel_joint', 'right_wheel_joint']


def script():
    """(kind, names, positions, stamp offset in ns or None for "now") in the
    order they are sent, 40 ms apart. Hand-made stamps are offsets from the
    first one, which is the wall clock when that part starts."""
    out = []
    # A: wall-clock stamps, both wheels
    for k in range(50):
        out.append(('now', WHEELS, [0.05 + 0.1 * k, -0.13 * k], None))
    ms = 1_000_000
    # B: around the 50 ms interval (>=), from the last published stamp
    b = [0, 49 * ms, 50 * ms - 1, 50 * ms, 99 * ms, 100 * ms, 149 * ms, 150 * ms, 150 * ms, 200 * ms + 1,
         250 * ms, 260 * ms]
    for k, off in enumerate(b):
        out.append(('synthetic', WHEELS, [1.0 + 0.01 * k, -1.0 - 0.02 * k], off))
    # one joint alone, then both: the least recently published one decides
    out.append(('synthetic', ['left_wheel_joint'], [2.0], 400 * ms))
    out.append(('synthetic', WHEELS, [2.1, -2.1], 420 * ms))     # right last at 260: 420 >= 310, published
    out.append(('synthetic', WHEELS, [2.2, -2.2], 460 * ms))     # both last at 420: not
    out.append(('synthetic', ['right_wheel_joint', 'left_wheel_joint'], [-2.3, 2.3], 470 * ms))  # not
    out.append(('synthetic', ['right_wheel_joint', 'left_wheel_joint'], [-2.4, 2.4], 520 * ms))  # name order out
    # the joints production does not send, a fixed joint and an unknown one
    out.append(('synthetic', ['caster_yaw_joint', 'caster_wheel_joint', 'mower_joint'], [0.7, -12.5, 3.0],
                600 * ms))
    out.append(('synthetic', ['gps_joint', 'left_wheel_joint'], [1.0, 3.3], 700 * ms))
    out.append(('synthetic', ['no_such_joint'], [1.0], 800 * ms))  # an empty /tf message
    # large and special angles
    for k, (l, r) in enumerate([(1000.0, -1000.0), (-7.5, 7.5), (3.141592653589793, -3.141592653589793),
                                (0.0, -0.0), (1e-9, -1e-9), (123456.789, -98765.4321)]):
        out.append(('synthetic', WHEELS, [l, r], (900 + 60 * k) * ms))
    out.append(('synthetic', ['caster_yaw_joint', 'mower_joint'], [-3.14159, 6.2], 1300 * ms))
    # invalid: name and position lengths differ; no positions at all
    out.append(('bad_length', WHEELS, [1.0], 1400 * ms))
    out.append(('no_position', WHEELS, [], 1450 * ms))
    out.append(('synthetic', WHEELS, [4.0, -4.0], 1500 * ms))
    # a zero stamp: last_published 0 + 50 ms > 0, never published
    out.append(('zero', ['caster_wheel_joint'], [0.5], None))
    out.append(('synthetic', WHEELS, [4.5, -4.5], 1600 * ms))
    # last: 10 s ahead of the wall clock. Published; afterwards rsp waits for
    # stamps 50 ms past whichever is earlier, that or the wall clock
    out.append(('future', WHEELS, [5.0, -5.0], None))
    for k in range(5):
        out.append(('now', WHEELS, [5.1 + 0.1 * k, -5.1 - 0.1 * k], None))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--xacro', default=XACRO)
    args = ap.parse_args()

    import fcntl
    import socket
    import struct
    SIOCGIFFLAGS, SIOCSIFFLAGS, IFF_MULTICAST = 0x8913, 0x8914, 0x1000
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    flags = struct.unpack('16sH', fcntl.ioctl(s, SIOCGIFFLAGS, struct.pack('16sH', b'lo', 0))[:18])[1]
    fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack('16sH', b'lo', flags | IFF_MULTICAST))

    import xacro
    import yaml
    urdf = xacro.process_file(args.xacro).toxml()
    work = os.path.dirname(os.path.abspath(args.out))
    params = os.path.join(work, 'rsp_params.yaml')
    # what launch_ros writes for mower.launch.py's parameters= dicts
    with open(params, 'w') as f:
        yaml.dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf, 'use_sim_time': False}}},
                  f, default_flow_style=False)
    log = open(os.path.join(work, 'rsp_capture_rsp.log'), 'w')
    rsp = subprocess.Popen(['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher', '--ros-args',
                            '-r', '__node:=robot_state_publisher', '--params-file', params],
                           stdout=log, stderr=subprocess.STDOUT)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    def qos(depth, tl=False):
        return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if tl else DurabilityPolicy.VOLATILE)

    def tf_json(m):
        return [{'parent': t.header.frame_id, 'child': t.child_frame_id,
                 'stamp': [t.header.stamp.sec, t.header.stamp.nanosec],
                 'xyz': [repr(t.transform.translation.x), repr(t.transform.translation.y),
                         repr(t.transform.translation.z)],
                 'q': [repr(t.transform.rotation.x), repr(t.transform.rotation.y), repr(t.transform.rotation.z),
                       repr(t.transform.rotation.w)]} for t in m.transforms]

    rclpy.init()
    node = Node('e4_rsp_capture')
    lk = threading.Lock()
    rec = {'tf_static': [], 'tf': [], 'robot_description': []}

    def on(key, conv):
        def f(m):
            with lk:
                rec[key].append([time.monotonic(), conv(m)])
        return f
    node.create_subscription(TFMessage, '/tf_static', on('tf_static', tf_json), qos(100, tl=True))
    node.create_subscription(TFMessage, '/tf', on('tf', tf_json), qos(100))
    node.create_subscription(String, '/robot_description', on('robot_description', lambda m: m.data), qos(1, tl=True))
    # mower_base's writer: SystemDefaults, R V 1 on Cyclone
    js_pub = node.create_publisher(JointState, '/joint_states', qos(1))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline and (js_pub.get_subscription_count() < 1 or node.count_publishers('/tf') < 1):
        time.sleep(0.05)
    time.sleep(1.0)

    sent = []
    base = None
    for kind, names, positions, off in script():
        now = node.get_clock().now().nanoseconds
        if kind in ('synthetic', 'bad_length', 'no_position'):
            if base is None:
                base = now
            stamp = base + off
        elif kind == 'zero':
            stamp = 0
        elif kind == 'future':
            stamp = now + 10_000_000_000
        else:
            stamp = now
        m = JointState()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp // 10**9, stamp % 10**9
        m.header.frame_id = 'base_link'
        m.name = list(names)
        m.position = [float(p) for p in positions]
        m.velocity = [0.0] * len(positions)
        with lk:
            sent.append({'kind': kind, 't': time.monotonic(), 'stamp': [m.header.stamp.sec, m.header.stamp.nanosec],
                         'name': list(names), 'position': [repr(float(p)) for p in positions]})
        js_pub.publish(m)
        time.sleep(0.04)
    time.sleep(1.0)
    rsp.send_signal(2)
    try:
        code = rsp.wait(10)
    except subprocess.TimeoutExpired:
        rsp.kill()
        code = 'killed'
    log.close()
    with lk:
        desc = rec['robot_description'][-1][1] if rec['robot_description'] else None
        doc = {
            'source': 'robot_state_publisher 3.3.4 (mower_path_planning:ros-free-test), tools/rsp_capture.py',
            'parameters': {'publish_frequency': 20.0, 'ignore_timestamp': False, 'frame_prefix': ''},
            'urdf': urdf,
            'robot_description_matches_urdf': desc == urdf,
            'robot_description_sha256': hashlib.sha256(desc.encode()).hexdigest() if desc else None,
            'robot_description_messages': len(rec['robot_description']),
            'tf_static': [m for _, m in rec['tf_static']],
            'joint_states': sent,
            'tf': [m for _, m in rec['tf']],
            'tf_receive_s': [t for t, _ in rec['tf']],
            'rsp_exit': code,
        }
    with open(args.out, 'w') as f:
        json.dump(doc, f, indent=1, ensure_ascii=False)
        f.write('\n')
    print(f'tf_static {len(doc["tf_static"])} msgs, tf {len(doc["tf"])} msgs for {len(sent)} joint_states, '
          f'robot_description {doc["robot_description_messages"]} (matches the xacro output: '
          f'{doc["robot_description_matches_urdf"]}), rsp exit {code}')
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()

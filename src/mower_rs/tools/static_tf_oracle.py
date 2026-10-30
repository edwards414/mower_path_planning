#!/usr/bin/env python3
"""The static_tf module against robot_state_publisher, side by side on DDS
(step E4 of docs/ROS_FREE_PLAN.md): the image's rsp 3.3.4 and
`mower_rsd --modules static_tf` on the same URDF and the same /joint_states,
every message of both compared.

Runs inside the runtime image, offline, on a DDS domain of its own::

    docker run --rm -u 0 --network none --cap-add NET_ADMIN -e ROS_DOMAIN_ID=75 \\
      -v <repo>:/r:ro -v <out>:/o -v <dir with mower_rsd>:/b:ro \\
      --entrypoint bash mower_path_planning:ros-free-test -c \\
      'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && \\
       python3 /r/src/mower_rs/tools/static_tf_oracle.py --rust /b/mower_rsd --out /o/static_tf.json'

- rsp runs as mower.launch.py runs it, its outputs remapped under
  `/oracle/` (`/tf`, `/tf_static`, `/robot_description`); the module runs as
  robot.launch.py runs it, on the real names, through mower_rsd's bridge.
- One rclpy probe sends tools/rsp_capture.py's script of joint states at
  25 Hz (mower_base's writer, R V 1) and records both sides.
- Compared: `/tf` message by message (the same messages out of the same
  joint states, frames and stamps equal, translations equal, quaternions
  within 1e-15) and the latency of each against rsp's; `/tf_static` (the
  same frames and values, one stamp per message); `/robot_description`
  (the same text); late joiners (a reader made after the script gets each
  side's latched messages, a tf2 Buffer fed from each looks the production
  chains up within 1e-12); the writers' QoS as `ros2 topic info -v` shows
  them; and, with the data collection's two static_transform_publishers
  next to the module, check_run.py's TF_CHAIN.
"""

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import threading
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import rsp_capture  # noqa: E402  (the script of joint states, the xacro path)

LOOKUPS = [('base_footprint', 'imu_link'), ('base_footprint', 'gps_link'),
           ('base_footprint', 'back_camera_rotated_frame'), ('base_link', 'depth_camera_link'),
           ('connector', 'blade_hub')]
# check_run.py: base_footprint -> base_link -> camera_front_link -> camera_front_optical_frame
TF_CHAIN = ('base_footprint', 'base_link', 'camera_front_link', 'camera_front_optical_frame')


def pct(xs, p):
    if not xs:
        return float('nan')
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(math.ceil(p / 100.0 * len(xs))) - 1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rust', required=True, help='mower_rsd')
    ap.add_argument('--out', required=True)
    ap.add_argument('--xacro', default=rsp_capture.XACRO)
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
    rsp_params = os.path.join(work, 'oracle_rsp_params.yaml')
    with open(rsp_params, 'w') as f:
        yaml.dump({'robot_state_publisher': {'ros__parameters': {'robot_description': urdf, 'use_sim_time': False}}},
                  f, default_flow_style=False)
    stf_params = os.path.join(work, 'oracle_static_tf_publisher.yaml')
    with open(stf_params, 'w') as f:
        # robot.launch.py's _node_params_file
        yaml.safe_dump({'static_tf_publisher': {'ros__parameters': {'robot_description': urdf}}}, f)
    logs = []
    procs = []

    def start(name, cmd):
        log = open(os.path.join(work, f'oracle_{name}.log'), 'w')
        logs.append(log)
        procs.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                      env=dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='1')))

    start('rsp', ['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher', '--ros-args',
                  '-r', '__node:=robot_state_publisher', '--params-file', rsp_params,
                  '-r', '/tf:=/oracle/tf', '-r', '/tf_static:=/oracle/tf_static',
                  '-r', '/robot_description:=/oracle/robot_description'])
    start('mower_rsd', [args.rust, '--modules', 'static_tf', '--ros-args', '--params-file', stf_params])

    import rclpy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    def qos(depth, tl=False):
        return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if tl else DurabilityPolicy.VOLATILE)

    def tf_rows(m):
        return [{'parent': t.header.frame_id, 'child': t.child_frame_id,
                 'stamp': [t.header.stamp.sec, t.header.stamp.nanosec],
                 'xyz': [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                 'q': [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                       t.transform.rotation.w]} for t in m.transforms]

    rclpy.init()
    node = Node('e4_static_tf_oracle')
    lk = threading.Lock()
    sides = {'cpp': '/oracle', 'rust': ''}
    rec = {side: {'tf': [], 'tf_static': [], 'robot_description': []} for side in sides}

    def on(side, key, conv):
        def f(m):
            with lk:
                rec[side][key].append([time.monotonic(), conv(m)])
        return f
    for side, prefix in sides.items():
        node.create_subscription(TFMessage, prefix + '/tf', on(side, 'tf', tf_rows), qos(100))
        node.create_subscription(TFMessage, prefix + '/tf_static', on(side, 'tf_static', tf_rows), qos(100, tl=True))
        node.create_subscription(String, prefix + '/robot_description', on(side, 'robot_description',
                                                                             lambda m: m.data), qos(1, tl=True))
    js_pub = node.create_publisher(JointState, '/joint_states', qos(1))
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and not (js_pub.get_subscription_count() >= 2
                                                and node.count_publishers('/tf') >= 1
                                                and node.count_publishers('/oracle/tf') >= 1):
        time.sleep(0.05)
    time.sleep(1.5)

    sent = []
    base = None
    for kind, names, positions, off in rsp_capture.script():
        now = node.get_clock().now().nanoseconds
        if kind in ('synthetic', 'bad_length', 'no_position'):
            base = base if base is not None else now
            stamp = base + off
        else:
            stamp = {'zero': 0, 'future': now + 10_000_000_000}.get(kind, now)
        m = JointState()
        m.header.stamp.sec, m.header.stamp.nanosec = stamp // 10**9, stamp % 10**9
        m.header.frame_id = 'base_link'
        m.name = list(names)
        m.position = [float(p) for p in positions]
        m.velocity = [0.0] * len(positions)
        t = time.monotonic()
        js_pub.publish(m)
        sent.append({'kind': kind, 't': t, 'stamp': [m.header.stamp.sec, m.header.stamp.nanosec]})
        time.sleep(0.04)
    time.sleep(1.0)

    # late joiners: a reader made now gets the latched messages of each side
    late = {side: [] for side in sides}
    for side, prefix in sides.items():
        node.create_subscription(TFMessage, prefix + '/tf_static',
                                 (lambda s: lambda m: late[s].append(tf_rows(m)))(side), qos(100, tl=True))
    # the writers as `ros2 topic info -v` shows them
    writers = {}
    for topic in ('/tf_static', '/tf', '/robot_description', '/oracle/tf_static', '/oracle/tf',
                  '/oracle/robot_description'):
        writers[topic] = [[i.node_name, ReliabilityPolicy(i.qos_profile.reliability).name,
                           DurabilityPolicy(i.qos_profile.durability).name, i.qos_profile.depth]
                          for i in node.get_publishers_info_by_topic(topic)]
    readers = {'/joint_states': [[i.node_name, ReliabilityPolicy(i.qos_profile.reliability).name,
                                  i.qos_profile.depth] for i in node.get_subscriptions_info_by_topic('/joint_states')]}
    time.sleep(1.0)
    # what the two sides published, before the chain's publishers below join /tf_static
    with lk:
        recorded = json.loads(json.dumps(rec))
        late_recorded = json.loads(json.dumps(late))
    # the data collection's two static_transform_publishers next to the module
    for name, x, parent, child in (('camera_front_link_tf', '0.2', 'base_link', 'camera_front_link'),
                                   ('camera_front_optical_tf', '0.0', 'camera_front_link',
                                    'camera_front_optical_frame')):
        start(name, ['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher', '--x', x, '--frame-id', parent,
                     '--child-frame-id', child, '--ros-args', '-r', f'__node:={name}'])
    chain_msgs = []
    node.create_subscription(TFMessage, '/tf_static', lambda m: chain_msgs.append(tf_rows(m)), qos(100, tl=True))
    time.sleep(3.0)

    with lk:
        doc = {'source': 'tools/static_tf_oracle.py: robot_state_publisher 3.3.4 against mower_rsd static_tf',
               'urdf_sha256': hashlib.sha256(urdf.encode()).hexdigest(), 'sent': sent,
               'cpp': recorded['cpp'], 'rust': recorded['rust'], 'late': late_recorded, 'writers': writers, 'readers': readers,
               'chain_msgs': chain_msgs}
    for p in procs:
        p.send_signal(2)
    for p in procs:
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
    for log in logs:
        log.close()
    failures = compare(doc, urdf, tf2_ros, TransformStamped, Time)
    doc['failures'] = failures
    with open(args.out, 'w') as f:
        json.dump(doc, f, indent=1)
    print('STATIC_TF_ORACLE_OK' if not failures else f'STATIC_TF_ORACLE_FAIL ({len(failures)})')
    for x in failures:
        print('  ' + x)
    sys.stdout.flush()
    os._exit(1 if failures else 0)


def compare(doc, urdf, tf2_ros, TransformStamped, Time):
    failures = []
    cpp, rust = doc['cpp'], doc['rust']

    def same_transforms(a, b, what, check_stamp=True):
        worst, exact, total = 0.0, 0, 0
        if [(t['parent'], t['child']) for t in a] != [(t['parent'], t['child']) for t in b]:
            failures.append(f'{what}: frames {[(t["parent"], t["child"]) for t in b]} vs '
                            f'{[(t["parent"], t["child"]) for t in a]}')
            return worst, exact, total
        for ta, tb in zip(a, b):
            if check_stamp and ta['stamp'] != tb['stamp']:
                failures.append(f'{what} {tb["child"]}: stamp {tb["stamp"]} vs {ta["stamp"]}')
            if ta['xyz'] != tb['xyz']:
                failures.append(f'{what} {tb["child"]}: translation {tb["xyz"]} vs {ta["xyz"]}')
            for qa, qb in zip(ta['q'], tb['q']):
                d = abs(qa - qb)
                worst = max(worst, d)
                exact += int(qa == qb and math.copysign(1, qa) == math.copysign(1, qb))
                total += 1
                if d > 1e-15:
                    failures.append(f'{what} {tb["child"]}: quaternion {tb["q"]} vs {ta["q"]}')
        return worst, exact, total

    # /tf, message by message
    tc, tr = cpp['tf'], rust['tf']
    print(f'/tf: rsp {len(tc)} messages, static_tf {len(tr)} for {len(doc["sent"])} joint states')
    if len(tc) != len(tr):
        failures.append(f'/tf: {len(tr)} messages against rsp\'s {len(tc)}')

    def stamps(msgs):
        return [m[0]['stamp'] if m else None for _, m in msgs]
    sc, sr = stamps(tc), stamps(tr)
    if sc != sr:
        k = next((k for k, (a, b) in enumerate(zip(sc, sr)) if a != b), min(len(sc), len(sr)))
        failures.append(f'/tf: the stamps part at message #{k}: {sr[k:k + 3]} vs {sc[k:k + 3]}')
        return failures
    worst, exact, total, lat = 0.0, 0, 0, []
    for k, ((t_c, a), (t_r, b)) in enumerate(zip(tc, tr)):
        w, e, n = same_transforms(a, b, f'/tf #{k}')
        worst, exact, total = max(worst, w), exact + e, total + n
        lat.append(t_r - t_c)
    print(f'  quaternions: {exact} of {total} components bit for bit, worst {worst:.3g}')
    print(f'  latency static_tf - rsp per message: p50 {pct(lat, 50)*1e3:+.2f} p99 {pct(lat, 99)*1e3:+.2f} '
          f'max {max(lat, default=float("nan"))*1e3:+.2f} ms')
    doc['tf_latency_diff_ms'] = {'p50': pct(lat, 50) * 1e3, 'p99': pct(lat, 99) * 1e3}
    # /tf_static
    for side in ('cpp', 'rust'):
        if len(doc[side]['tf_static']) != 1:
            failures.append(f'/tf_static {side}: {len(doc[side]["tf_static"])} messages, not one')
        if len(doc['late'][side]) != 1:
            failures.append(f'/tf_static {side}: a late joiner got {len(doc["late"][side])} messages')
    if cpp['tf_static'] and rust['tf_static']:
        a, b = cpp['tf_static'][0][1], rust['tf_static'][0][1]
        w, e, n = same_transforms(a, b, '/tf_static', check_stamp=False)
        print(f'/tf_static: {len(b)} transforms, quaternions {e} of {n} bit for bit, worst {w:.3g}')
        for side, msg in (('cpp', a), ('rust', b)):
            if len({tuple(t['stamp']) for t in msg}) != 1:
                failures.append(f'/tf_static {side}: more than one stamp')
        # a tf2 Buffer fed from each
        bufs = {}
        for side, msg in (('cpp', a), ('rust', b)):
            buf = tf2_ros.Buffer()
            for t in msg:
                ts = TransformStamped()
                ts.header.frame_id, ts.child_frame_id = t['parent'], t['child']
                ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = t['xyz']
                (ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z,
                 ts.transform.rotation.w) = t['q']
                buf.set_transform_static(ts, side)
            bufs[side] = buf
        worst = 0.0
        for target, source in LOOKUPS:
            ta = bufs['cpp'].lookup_transform(target, source, Time())
            tb = bufs['rust'].lookup_transform(target, source, Time())
            for x, y in ((ta.transform.translation.x, tb.transform.translation.x),
                         (ta.transform.translation.y, tb.transform.translation.y),
                         (ta.transform.translation.z, tb.transform.translation.z),
                         (ta.transform.rotation.x, tb.transform.rotation.x),
                         (ta.transform.rotation.y, tb.transform.rotation.y),
                         (ta.transform.rotation.z, tb.transform.rotation.z),
                         (ta.transform.rotation.w, tb.transform.rotation.w)):
                worst = max(worst, abs(x - y))
        print(f'tf2 lookups {LOOKUPS}: worst difference {worst:.3g}')
        if worst > 1e-12:
            failures.append(f'tf2 lookups differ by {worst}')
    # /robot_description
    for side in ('cpp', 'rust'):
        texts = [m for _, m in doc[side]['robot_description']]
        if len(texts) != 1 or texts[0] != urdf:
            failures.append(f'/robot_description {side}: {len(texts)} messages, the URDF: '
                            f'{bool(texts) and texts[0] == urdf}')
    dc, dr = [m for _, m in cpp['robot_description']], [m for _, m in rust['robot_description']]
    print(f'/robot_description: rsp {len(dc)}, static_tf {len(dr)} messages, '
          f'the same text: {bool(dc) and dc[:1] == dr[:1]} ({len(urdf)} characters)')
    # the writers on DDS
    for topic, ws in doc['writers'].items():
        print(f'writers {topic}: {ws}')
    print(f'readers /joint_states: {doc["readers"]["/joint_states"]}')
    for topic, want in (('/tf_static', ['RELIABLE', 'TRANSIENT_LOCAL', 1]), ('/tf', ['RELIABLE', 'VOLATILE', 100]),
                        ('/robot_description', ['RELIABLE', 'TRANSIENT_LOCAL', 1])):
        ours = [w[1:] for w in doc['writers'][topic] if w[0] == 'mower_bus_bridge']
        theirs = [w[1:] for w in doc['writers']['/oracle' + topic]]
        if ours != [want] or theirs != [want]:
            failures.append(f'{topic}: writer QoS {ours} (static_tf) vs {theirs} (rsp), want {want}')
    # check_run.py's chain: latest transform per child over the latched messages
    latest = {}
    for msg in doc['chain_msgs']:
        for t in msg:
            latest[t['child']] = t['parent']
    chain_ok = all(latest.get(c) == p for p, c in zip(TF_CHAIN, TF_CHAIN[1:]))
    print(f'TF_CHAIN {" -> ".join(TF_CHAIN)} on /tf_static with the module: {chain_ok}')
    if not chain_ok:
        failures.append(f'TF_CHAIN broken: {latest}')
    return failures


if __name__ == '__main__':
    main()

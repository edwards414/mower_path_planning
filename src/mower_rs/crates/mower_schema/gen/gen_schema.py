#!/usr/bin/env python3
"""Regenerates mower_schema's committed tables from a ROS Jazzy install.

Run it in the robot image, whose .msg files are the ones rosbag2 embeds in
the robot's bags:

    docker run --rm --network none -v "$PWD/src/mower_rs/crates/mower_schema":/x \\
      --entrypoint bash mower_path_planning:ros-free-test \\
      -c 'source /opt/ros/jazzy/setup.bash && source /mower_ws/install/setup.bash && python3 -I /x/gen/gen_schema.py /x'

For every root type below and everything it depends on, it copies the
package's `msg/<Type>.msg` verbatim to `msg/<pkg>/<Type>.msg` (where rosbag2
finds it: the `rosidl_interfaces` ament index entry of the package) and takes
the type's RIHS01 hash from `msg/<Type>.json` (`type_hashes`). It writes
`src/table.rs` (the type list, sorted by name) and `msg/PROVENANCE.txt` (the
Debian package versions the files came from). Dependencies are found with
rosbag2's own rule (rosbag2_cpp local_message_definition_source.cpp,
MSG_FIELD_TYPE_REGEX: the first word of a line, bare names in the package of
the file that names them), so the closure is the set of types rosbag2 would
open.

`INTERFACE_ROOTS` (E9-22) are every service, action and further message the
crates name, for the generated plain types (`mower_msgs`). Their `.srv` and
`.action` files go to `srv/<pkg>/` and `action/<pkg>/`, the messages only they
need to `msg/<pkg>/`, and the hash of every part rosidl makes of them (`_Request`,
`_SendGoal_Response`, `_FeedbackMessage`, ...) comes from the same `<Type>.json`.
They are kept apart in `src/table.rs` (`MORE_TYPES`, `INTERFACES`, feature
`interfaces`), so that `TYPES`, and with it what the recorder writes and what
`cdr_vectors.txt` holds, does not move. `mower_ros_facts/tests/plain_types.rs` fails
when a crate names a type that is not here.

Then run `gen/gen_types.py` (no ROS needed), which writes the Rust.

Stdlib only.
"""
import json
import os
import re
import subprocess
import sys

# The message types on the bus (mower_bridge's manifest) and in the record
# profiles (mower_recorder/config/*.yaml); E5 design 1.8, 3.4.
ROOTS = [
    'diagnostic_msgs/msg/DiagnosticArray',
    'geometry_msgs/msg/PoseStamped',
    'geometry_msgs/msg/Twist',
    'geometry_msgs/msg/TwistStamped',
    'mower_interface/msg/CoverageProgress',
    'nav_msgs/msg/OccupancyGrid',
    'nav_msgs/msg/Odometry',
    'nav_msgs/msg/Path',
    'rcl_interfaces/msg/Log',
    'sensor_msgs/msg/BatteryState',
    'sensor_msgs/msg/CameraInfo',
    'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/Imu',
    'sensor_msgs/msg/JointState',
    'sensor_msgs/msg/NavSatFix',
    'std_msgs/msg/Bool',
    'std_msgs/msg/Header',
    'std_msgs/msg/String',
    'tf2_msgs/msg/TFMessage',
    'visualization_msgs/msg/Marker',
    'visualization_msgs/msg/MarkerArray',
]

# E9-22: every r2r::<pkg>::{msg,srv,action}::<Type> named under crates/, and what
# the graph needs of parameters and actions on top of them (ParameterEvent,
# action_msgs). The rest of graph.json's types are nav2's and rcl's own endpoints,
# which no crate names: mower_ros_facts/tests/plain_types.rs lists them.
INTERFACE_ROOTS = [
    'action_msgs/msg/GoalStatusArray',
    'action_msgs/srv/CancelGoal',
    'builtin_interfaces/msg/Duration',
    'builtin_interfaces/msg/Time',
    'diagnostic_msgs/msg/DiagnosticArray',
    'diagnostic_msgs/msg/DiagnosticStatus',
    'diagnostic_msgs/msg/KeyValue',
    'geographic_msgs/msg/GeoPoint',
    'geometry_msgs/msg/Point',
    'geometry_msgs/msg/Pose',
    'geometry_msgs/msg/PoseStamped',
    'geometry_msgs/msg/PoseWithCovariance',
    'geometry_msgs/msg/Quaternion',
    'geometry_msgs/msg/Transform',
    'geometry_msgs/msg/TransformStamped',
    'geometry_msgs/msg/Twist',
    'geometry_msgs/msg/TwistStamped',
    'geometry_msgs/msg/TwistWithCovariance',
    'geometry_msgs/msg/Vector3',
    'lifecycle_msgs/srv/GetState',
    'mower_interface/action/Waypoint',
    'mower_interface/msg/CoverageProgress',
    'mower_interface/msg/ZoneMap',
    'mower_interface/srv/CancelNavigationDispatch',
    'mower_interface/srv/ChannelPathList',
    'mower_interface/srv/ChannelRoute',
    'mower_interface/srv/ChennalPathList',
    'mower_interface/srv/ConfirmNavigationDispatch',
    'mower_interface/srv/EditZone',
    'mower_interface/srv/GetCoverageProgress',
    'mower_interface/srv/GetZoneList',
    'mower_interface/srv/ImportImageMask',
    'mower_interface/srv/MissionOperationLock',
    'mower_interface/srv/PidAutotune',
    'mower_interface/srv/ResumeCoverage',
    'mower_interface/srv/SiteOp',
    'mower_interface/srv/ZoneExecPath',
    'mower_interface/srv/ZoneMapList',
    'mower_interface/srv/ZoneSequence',
    'nav2_msgs/action/FollowPath',
    'nav2_msgs/action/NavigateToPose',
    'nav_msgs/msg/MapMetaData',
    'nav_msgs/msg/OccupancyGrid',
    'nav_msgs/msg/Odometry',
    'nav_msgs/msg/Path',
    'nav_msgs/srv/SetMap',
    'rcl_interfaces/msg/ListParametersResult',
    'rcl_interfaces/msg/Log',
    'rcl_interfaces/msg/Parameter',
    'rcl_interfaces/msg/ParameterDescriptor',
    'rcl_interfaces/msg/ParameterEvent',
    'rcl_interfaces/msg/ParameterValue',
    'rcl_interfaces/msg/SetParametersResult',
    'rcl_interfaces/srv/DescribeParameters',
    'rcl_interfaces/srv/GetParameterTypes',
    'rcl_interfaces/srv/GetParameters',
    'rcl_interfaces/srv/ListParameters',
    'rcl_interfaces/srv/SetParameters',
    'rcl_interfaces/srv/SetParametersAtomically',
    'robot_localization/srv/FromLL',
    'robot_localization/srv/FromLLArray',
    'robot_localization/srv/GetState',
    'robot_localization/srv/SetDatum',
    'robot_localization/srv/ToLL',
    'sensor_msgs/msg/BatteryState',
    'sensor_msgs/msg/CameraInfo',
    'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/Imu',
    'sensor_msgs/msg/JointState',
    'sensor_msgs/msg/NavSatFix',
    'sensor_msgs/msg/NavSatStatus',
    'sensor_msgs/msg/RegionOfInterest',
    'std_msgs/msg/Bool',
    'std_msgs/msg/ColorRGBA',
    'std_msgs/msg/Empty',
    'std_msgs/msg/Header',
    'std_msgs/msg/String',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
    'tf2_msgs/msg/TFMessage',
    'visualization_msgs/msg/Marker',
    'visualization_msgs/msg/MarkerArray',
    'visualization_msgs/msg/MeshFile',
    'visualization_msgs/msg/UVCoordinate',
]

# What rosidl makes of a `.srv` and of an `.action` file, past the file's own type.
SERVICE_PARTS = ['_Request', '_Response', '_Event']
ACTION_PARTS = ['_Goal', '_Result', '_Feedback', '_SendGoal', '_SendGoal_Request', '_SendGoal_Response',
                '_SendGoal_Event', '_GetResult', '_GetResult_Request', '_GetResult_Response', '_GetResult_Event',
                '_FeedbackMessage']
# What every action uses by rosidl's own rule, though its file names none of them.
ACTION_DEPS = ['unique_identifier_msgs/UUID', 'builtin_interfaces/Time']

FIELD_TYPE = re.compile(r'(?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+')
PRIMITIVES = {'bool', 'byte', 'char', 'float32', 'float64', 'int8', 'uint8', 'int16', 'uint16',
              'int32', 'uint32', 'int64', 'uint64', 'string'}


def prefix_of(pkg):
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if os.path.isfile(os.path.join(prefix, 'share', 'ament_index', 'resource_index',
                                       'rosidl_interfaces', pkg)):
            return prefix
    raise SystemExit(f'no rosidl_interfaces entry for {pkg}')


def interface_path(pkg, name, ext):
    prefix = prefix_of(pkg)
    index = open(os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'rosidl_interfaces',
                              pkg)).read()
    for line in index.split('\n'):
        if line and os.path.basename(line) == name + ext:
            return os.path.join(prefix, 'share', pkg, line)
    raise SystemExit(f'{pkg}/{name}{ext} not in the index')


def msg_path(pkg, name):
    return interface_path(pkg, name, '.msg')


def deps(text, pkg):
    out = set()
    for m in FIELD_TYPE.finditer(text):
        t = m.group(1)
        if t in PRIMITIVES:
            continue
        out.add(t if '/' in t else f'{pkg}/{t}')
    return out


def deb_version(pkg):
    deb = 'ros-jazzy-' + pkg.replace('_', '-')
    try:
        return subprocess.run(['dpkg-query', '-W', '-f', '${Version}', deb], capture_output=True,
                              text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def message_closure(roots):
    """`pkg/Type` -> (.msg text, hash), for `roots` and all they depend on."""
    todo = list(roots)
    seen = {}
    while todo:
        t = todo.pop()
        if t in seen:
            continue
        pkg, name = t.split('/')
        path = msg_path(pkg, name)
        text = open(path, 'rb').read()
        desc = json.load(open(path[:-4] + '.json'))
        full = f'{pkg}/msg/{name}'
        hashes = [h['hash_string'] for h in desc['type_hashes'] if h['type_name'] == full]
        if len(hashes) != 1:
            raise SystemExit(f'no hash for {full}')
        seen[t] = (path, text, hashes[0])
        todo.extend(sorted(deps(text.decode('utf-8'), pkg)))
    return seen


def interface_entry(root):
    """A `.srv` or `.action` root: its text, each part's hash, the messages it names."""
    pkg, kind, name = root.split('/')
    ext, parts = ('.srv', SERVICE_PARTS) if kind == 'srv' else ('.action', ACTION_PARTS)
    path = interface_path(pkg, name, ext)
    text = open(path, 'rb').read()
    desc = json.load(open(path[:-len(ext)] + '.json'))
    by_name = {h['type_name']: h['hash_string'] for h in desc['type_hashes']}
    hashes = []
    for part in [''] + parts:
        full = f'{pkg}/{kind}/{name}{part}'
        if full not in by_name:
            raise SystemExit(f'no hash for {full}')
        hashes.append((full, by_name[full]))
    named = deps(text.decode('utf-8'), pkg)
    if kind == 'action':
        named |= set(ACTION_DEPS)
    return text, hashes, named


def main():
    out = sys.argv[1]
    seen = message_closure([r.replace('/msg/', '/') for r in ROOTS])
    interface_roots = [r for r in INTERFACE_ROOTS if '/msg/' not in r]
    interfaces = {r: interface_entry(r) for r in interface_roots}
    more_roots = [r.replace('/msg/', '/') for r in INTERFACE_ROOTS if '/msg/' in r]
    for _text, _hashes, named in interfaces.values():
        more_roots.extend(sorted(named))
    everything = message_closure(more_roots)
    for t in seen:
        everything.setdefault(t, seen[t])
    more = {t: v for t, v in everything.items() if t not in seen}
    for t, (_path, text, _h) in everything.items():
        pkg, name = t.split('/')
        os.makedirs(os.path.join(out, 'msg', pkg), exist_ok=True)
        with open(os.path.join(out, 'msg', pkg, name + '.msg'), 'wb') as f:
            f.write(text)
    for r, (text, _hashes, _named) in interfaces.items():
        pkg, kind, name = r.split('/')
        os.makedirs(os.path.join(out, kind, pkg), exist_ok=True)
        with open(os.path.join(out, kind, pkg, f'{name}.{kind}'), 'wb') as f:
            f.write(text)
    lines = [
        '// @generated by gen/gen_schema.py from the robot image: do not edit.',
        '// The .msg files are copies of the ROS Jazzy packages listed in',
        '// msg/PROVENANCE.txt; the hashes are their `<Type>.json` type hashes.',
        '',
        '/// Every message type mower_schema knows: `pkg/msg/Type`, its `.msg` text',
        '/// exactly as installed, and its RIHS01 type hash.',
        'pub(crate) static TYPES: &[(&str, &str, &str)] = &[',
    ]
    for t in sorted(seen):
        pkg, name = t.split('/')
        lines.append(f'    ("{pkg}/msg/{name}", include_str!("../msg/{pkg}/{name}.msg"), "{seen[t][2]}"),')
    lines += ['];', '', '/// The root types the table was generated for (gen/gen_schema.py `ROOTS`).',
              'pub(crate) static ROOTS: &[&str] = &[']
    lines += [f'    "{r}",' for r in ROOTS]
    lines += ['];', '', '/// The messages only the crates\' other interfaces need (E9-22): as [`TYPES`], but',
              '/// neither the recorder nor `cdr_vectors.txt` sees them.',
              '#[cfg(feature = "interfaces")]',
              'pub(crate) static MORE_TYPES: &[(&str, &str, &str)] = &[']
    for t in sorted(more):
        pkg, name = t.split('/')
        lines.append(f'    ("{pkg}/msg/{name}", include_str!("../msg/{pkg}/{name}.msg"), "{more[t][2]}"),')
    lines += ['];', '', '/// Every service and action the crates name (E9-22): `pkg/srv/Type` or',
              '/// `pkg/action/Type`, its file exactly as installed, and the RIHS01 hash of the',
              '/// type and of each part rosidl makes of it.',
              '#[cfg(feature = "interfaces")]',
              'pub(crate) static INTERFACES: &[(&str, &str, &[(&str, &str)])] = &[']
    for r in sorted(interfaces):
        pkg, kind, name = r.split('/')
        lines.append(f'    ("{r}", include_str!("../{kind}/{pkg}/{name}.{kind}"), &[')
        lines += [f'        ("{part}", "{h}"),' for part, h in interfaces[r][1]]
        lines.append('    ]),')
    lines += ['];', '', '/// The roots of [`MORE_TYPES`] and [`INTERFACES`] (gen/gen_schema.py `INTERFACE_ROOTS`).',
              '#[cfg(feature = "interfaces")]',
              'pub(crate) static INTERFACE_ROOTS: &[&str] = &[']
    lines += [f'    "{r}",' for r in INTERFACE_ROOTS]
    lines += ['];', '']
    open(os.path.join(out, 'src', 'table.rs'), 'w').write('\n'.join(lines))
    pkgs = sorted({t.split('/')[0] for t in everything} | {r.split('/')[0] for r in interfaces})
    prov = ['# The files under msg/, srv/ and action/ and the hashes in src/table.rs, as gen/gen_schema.py found them.',
            '# package  version (Debian package ros-jazzy-<package>, or the path for a source build)']
    for p in pkgs:
        v = deb_version(p)
        prov.append(f'{p} {v if v else os.path.relpath(prefix_of(p), "/")}')
    open(os.path.join(out, 'msg', 'PROVENANCE.txt'), 'w').write('\n'.join(prov) + '\n')
    print(f'{len(seen)} types from {len(ROOTS)} roots in {len(pkgs)} packages; '
          f'{len(more)} more messages and {len(interfaces)} services and actions from {len(INTERFACE_ROOTS)} roots')


if __name__ == '__main__':
    main()

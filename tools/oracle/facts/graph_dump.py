#!/usr/bin/env python3
"""The DDS graph of a running stack from outside, in the image, with rclpy.

    graph_dump.py --out graph.json [--stable 20] [--timeout 240]

From a participant of its own, so every endpoint of the stack is a remote one,
as nav2, ws_bridge's r2r clients or the recorder see it. It polls once a second
and writes the graph once it has stayed the same for --stable seconds (nav2
makes endpoints while its lifecycle nodes activate). Read-only: it publishes
nothing, subscribes to nothing and calls nothing. This tool's own node and
ros2cli's are left out. What freeze_stack.py normalises afterwards is said
there. From e2-verify's graph_dump.py (E2), its sampling left out.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node

SELF = 'ros_facts_graph_dump'


def fq(name, ns):
    return (ns if ns.endswith('/') else ns + '/') + name


def own(node):
    return node.startswith(('ros_facts_', '_ros2cli'))


def enum_name(v):
    return getattr(v, 'name', str(v))


def dur(d):
    try:
        return d.nanoseconds
    except AttributeError:
        return str(d)


def qos_dict(q):
    return {'reliability': enum_name(q.reliability), 'durability': enum_name(q.durability),
            'history': enum_name(q.history), 'depth': q.depth, 'deadline_ns': dur(q.deadline),
            'lifespan_ns': dur(q.lifespan), 'liveliness': enum_name(q.liveliness),
            'liveliness_lease_ns': dur(q.liveliness_lease_duration)}


def endpoint(info):
    return {'node': fq(info.node_name, info.node_namespace), 'type': info.topic_type,
            'hash': str(info.topic_type_hash), 'qos': qos_dict(info.qos_profile)}


def graph(node):
    key = lambda e: json.dumps(e, sort_keys=True)
    topics = {}
    for name, types in sorted(node.get_topic_names_and_types()):
        pubs = [endpoint(i) for i in node.get_publishers_info_by_topic(name) if not own(i.node_name)]
        subs = [endpoint(i) for i in node.get_subscriptions_info_by_topic(name) if not own(i.node_name)]
        if pubs or subs:
            topics[name] = {'types': sorted(types), 'publishers': sorted(pubs, key=key),
                            'subscribers': sorted(subs, key=key)}
    services = {}
    nodes = []
    for n, ns in sorted(node.get_node_names_and_namespaces()):
        if own(n):
            continue
        nodes.append(fq(n, ns))
        for query, side in ((node.get_service_names_and_types_by_node, 'servers'),
                            (node.get_client_names_and_types_by_node, 'clients')):
            try:
                pairs = query(n, ns)
            except Exception as e:  # the node went between the two queries
                print(f'{side} of {fq(n, ns)}: {e}', file=sys.stderr)
                continue
            for sname, stypes in pairs:
                entry = services.setdefault(sname, {'types': sorted(stypes), 'servers': [], 'clients': []})
                entry[side].append(fq(n, ns))
    for entry in services.values():
        entry['servers'].sort()
        entry['clients'].sort()
    return {'nodes': sorted(nodes), 'topics': topics, 'services': dict(sorted(services.items()))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--stable', type=float, default=20.0)
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args()
    rclpy.init()
    node = Node(SELF)
    t0 = last_change = time.monotonic()
    last = None
    while True:
        rclpy.spin_once(node, timeout_sec=1.0)
        now = graph(node)
        if now != last:
            last, last_change = now, time.monotonic()
        if time.monotonic() - last_change >= args.stable:
            break
        if time.monotonic() - t0 > args.timeout:
            sys.exit(f'the graph did not settle within {args.timeout} s')
    with open(args.out, 'w') as f:
        json.dump(last, f, indent=1, sort_keys=True)
    print(f'graph after {time.monotonic() - t0:.0f} s: {len(last["nodes"])} nodes, '
          f'{len(last["topics"])} topics, {len(last["services"])} services')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

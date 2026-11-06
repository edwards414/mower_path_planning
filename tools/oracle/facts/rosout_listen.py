#!/usr/bin/env python3
"""The name of every logger that writes to /rosout, until told to stop.

    rosout_listen.py --out names.txt --stop-file FILE

Started right after the container (freeze_stack.py), so it sees the stack's
start. Reliable and transient local, /rosout's own QoS, so what was logged just
before it matched comes too. Read-only.
"""
import argparse
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--stop-file', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Node('ros_facts_rosout_listen', enable_rosout=False)
    names = set()
    qos = QoSProfile(depth=1000, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Log, '/rosout', lambda m: names.add(m.name), qos)
    while not os.path.exists(args.stop_file):
        rclpy.spin_once(node, timeout_sec=0.2)
    with open(args.out + '.tmp', 'w') as f:
        f.writelines(n + '\n' for n in sorted(names))
    os.rename(args.out + '.tmp', args.out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

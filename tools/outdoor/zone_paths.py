#!/usr/bin/env python3
"""Dump each zone's planned coverage path (/get_zone_map_list_srv) and the
coverage strip width (boustrophedon_coverage's strip_width_m) as JSON, for
truth.py. Runs in the observer sidecar (DDS on the sim's lo). Read-only.

    zone_paths.py --out /tmp/obs/zone_paths.json
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from mower_interface.srv import ZoneMapList
from rcl_interfaces.srv import GetParameters


def wait(node, fut, s):
    end = time.monotonic() + s
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return fut.result() if fut.done() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = Node('outdoor_zone_paths')
    doc = {'t': time.time(), 'zones': {}, 'strip_width_m': None}
    c = node.create_client(ZoneMapList, '/get_zone_map_list_srv')
    if c.wait_for_service(timeout_sec=10.0):
        r = wait(node, c.call_async(ZoneMapList.Request()), 20)
        for z in (r.zone_map_list if r else []):
            doc['zones'][str(z.zone_id)] = {
                'frame': z.path.header.frame_id,
                'path': [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)] for p in z.path.poses],
                'splits': [[round(p.position.x, 4), round(p.position.y, 4)] for p in z.coverage_split_points],
            }
    p = node.create_client(GetParameters, '/boustrophedon_coverage/get_parameters')
    if p.wait_for_service(timeout_sec=5.0):
        r = wait(node, p.call_async(GetParameters.Request(names=['strip_width_m'])), 10)
        if r and r.values:
            doc['strip_width_m'] = r.values[0].double_value
    json.dump(doc, open(args.out, 'w'))
    print('zone paths:', {k: len(v['path']) for k, v in doc['zones'].items()}, 'strip', doc['strip_width_m'])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

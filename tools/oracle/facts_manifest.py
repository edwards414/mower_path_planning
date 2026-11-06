#!/usr/bin/env python3
"""Writes deploy/test/golden/ros_facts/SHA256SUMS: the sha256 of every frozen
file, E3's JSON corpus included, which mower_ros_facts/tests/frozen.rs checks
with no ROS. Run it after whatever wrote them (tools/oracle/regen.sh facts,
tools/oracle/freeze_stack.py do). Stdlib only.

    tools/oracle/facts_manifest.py
"""
import hashlib
import os

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FACTS = os.path.join('deploy', 'test', 'golden', 'ros_facts')
CORPUS = os.path.join('src', 'mower_rs', 'crates', 'mower_ws_bridge', 'tests', 'golden')
# README: what they are, not what ROS said. PROVENANCE: which image made them,
# which a regeneration from a newer image changes and nothing else does.
LEFT_OUT = {'README.md', 'PROVENANCE.txt', 'SHA256SUMS'}


def main():
    paths = []
    for root, _, names in os.walk(os.path.join(REPO, FACTS)):
        paths += [os.path.relpath(os.path.join(root, n), REPO) for n in names if n not in LEFT_OUT]
    corpus = os.path.join(REPO, CORPUS)
    paths += [os.path.join(CORPUS, n) for n in os.listdir(corpus) if n.endswith(('.txt', '.frame'))]
    lines = []
    for path in sorted(paths):
        with open(os.path.join(REPO, path), 'rb') as f:
            lines.append(f'{hashlib.sha256(f.read()).hexdigest()}  {path}\n')
    with open(os.path.join(REPO, FACTS, 'SHA256SUMS'), 'w') as f:
        f.writelines(lines)
    print(f'SHA256SUMS: {len(lines)} files')


if __name__ == '__main__':
    main()

"""Make the pure-Python modules importable without a ROS workspace.

    /path/to/venv/bin/python -m pytest src/mower_recorder/test -q

The package has no ROS nodes since E5-7 (camera_extrinsics, run_manifest,
check_run, cli). Needs pytest, PyYAML, mcap, mcap-ros2-support, zstandard,
moto, boto3, Pillow.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
for p in (PKG_ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

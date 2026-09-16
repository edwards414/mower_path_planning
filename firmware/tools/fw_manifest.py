#!/usr/bin/env python3
"""Write the manifest that sits next to a built firmware .bin.

firmware-sync (mower_flash.py sync) compares this against the 0x87
FIRMWARE_INFO frame the running application sends, and /robot/info reports
it as the bundled firmware.

    python3 tools/fw_manifest.py build/mower_robot_firmware.bin \
        --version 0.6.0 --semver 0.6.0 --git-sha abc... --build-unix 1780000000 --dirty 0 \
        -o build/mower_robot_firmware.json
"""

import argparse
import hashlib
import json
import os
import zlib


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("image")
    ap.add_argument("--version", required=True, help="full version string (git describe)")
    ap.add_argument("--semver", required=True, help="MAJOR.MINOR.PATCH as embedded in the firmware")
    ap.add_argument("--git-sha", required=True)
    ap.add_argument("--build-unix", type=int, required=True)
    ap.add_argument("--dirty", type=int, default=0)
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    major, minor, patch = (int(x) for x in args.semver.split("."))
    sha = args.git_sha.lower()
    manifest = {
        "file": os.path.basename(args.image),
        "size": len(image),
        "crc32": "%08x" % (zlib.crc32(image) & 0xFFFFFFFF),
        "sha256": hashlib.sha256(image).hexdigest(),
        "version": args.version,
        "semver": [major, minor, patch],
        "git_sha": sha,
        # what the 0x87 frame carries: first 4 bytes of the commit as a number
        "git_sha32": int(sha[:8], 16) if len(sha) >= 8 else 0,
        "build_unix": args.build_unix,
        "dirty": bool(args.dirty),
    }
    with open(args.output, "w") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")
    print(f"{args.output}: {args.version} sha={sha[:8]} {len(image)} bytes crc32={manifest['crc32']}")


if __name__ == "__main__":
    main()

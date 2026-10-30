#!/usr/bin/env bash
# Rebuilds tests/vectors/boot_oracle.json and tests/vectors/pop_frame_oracle.json.
#
#   tests/oracle/boot_generate.sh [workdir]
#
# boot_oracle.c is compiled against the firmware's own boot_shared.h and the
# unmodified bootloader/Src/bl_flash.c (stub_hal/ stands in for the STM32
# HAL); boot_vectors.py adds zlib's CRC-32 vectors and runs the original
# FrameReader of firmware/tools/mower_flash.py. Needs cc and python3; no
# ROS, no pyserial.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
crate="$(dirname "$(dirname "$here")")"      # .../crates/mower_base_core
repo="$(cd "$crate/../../../.." && pwd)"     # repo root
work="${1:-$(mktemp -d)}"
out="$crate/tests/vectors"
cc="${CC:-cc}"
fw="$repo/firmware"

mkdir -p "$work" "$out"
# bl_flash.c casts 32-bit addresses to pointers; harmless here, nothing it
# would dereference is called
"$cc" -std=c11 -O1 -Wall -Wextra -Wno-int-to-pointer-cast \
  -I "$here/stub_hal" -I "$fw/bootloader/Src" -I "$fw/bootloader/Inc" -I "$fw/Module/Inc" \
  "$here/boot_oracle.c" -o "$work/boot_oracle"
"$work/boot_oracle" > "$work/boot_layout.json"
python3 "$here/boot_vectors.py" "$work/boot_layout.json" "$fw/tools/mower_flash.py" "$out"
echo "boot vectors regenerated in $out"

#!/bin/sh
# The outdoor harness's CPU keeper: one privileged container (cgroupns=host)
# for every running stack. Docker Desktop re-applies a container's CFS
# settings now and then with another period (cpu.max "11060 20000" where
# "5530 10000" was written), so every 0.5 s this puts back the value each
# stack asked for: one file per sim container in /targets, named by its
# container id, holding "QUOTA PERIOD". Each correction is one line on
# stdout with the time and what it found. Started by outdoor.sh.
cd /targets || exit 1
while true; do
  for id in *; do
    [ -f "$id" ] || continue
    want=$(cat "$id")
    f=/host_cg/docker/$id/cpu.max
    [ -f "$f" ] || continue
    got=$(cat "$f")
    if [ "$got" != "$want" ]; then
      echo "$want" > "$f"
      echo "$(date +%s.%N) ${id%"${id#????????????}"} drift '$got' -> '$want'"
    fi
  done
  sleep 0.5
done
